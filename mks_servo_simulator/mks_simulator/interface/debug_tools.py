"""
Debug tools for the MKS servo simulator.

Provides command injection, testing utilities, and debugging features.
"""

import asyncio
import logging
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Dict, List, Optional, Tuple

from mks_servo_can import constants as const
from mks_servo_can import get_manual_commands

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from ..virtual_can_bus import VirtualCANBus
    from .llm_debug_interface import LLMDebugInterface


def _payload_length(dlc: int) -> int:
    """
    Convert a manual DLC into a count of payload bytes.

    The manual quotes DLC for the whole frame - command code, payload, CRC - so
    a command that carries no arguments still reads as DLC 2.
    """
    return max(dlc - 2, 0)


@dataclass
class CommandSpec:
    """Specification for a MKS servo command"""
    code: int
    name: str
    description: str
    data_length: int
    response_length: int
    category: str
    parameters: List[Dict[str, Any]]


@dataclass
class InjectedCommand:
    """Record of an injected command"""
    timestamp: float
    motor_id: int
    command_code: int
    command_name: str
    data_bytes: bytes
    response_data: Optional[bytes]
    execution_time_ms: float
    success: bool
    error_message: Optional[str] = None


class CommandInjector:
    """
    Handles command injection and testing for the simulator.
    
    Features:
    - Raw command injection
    - Pre-defined command templates
    - Command validation
    - Response tracking
    - Test scenario execution
    """

    def __init__(
        self,
        virtual_can_bus: "VirtualCANBus",
        debug_interface: Optional["LLMDebugInterface"] = None
    ):
        """
        Initialize the command injector.
        
        Args:
            virtual_can_bus: The virtual CAN bus instance
            debug_interface: Optional debug interface for additional data
        """
        self.virtual_can_bus = virtual_can_bus
        self.debug_interface = debug_interface

        # Command history and tracking
        self.injected_commands: List[InjectedCommand] = []
        self.max_history = 1000

        # Load command specifications
        self.command_specs: Dict[int, CommandSpec] = {}
        self._load_command_specs()

        # Pre-defined command templates
        self.command_templates: Dict[str, Dict[str, Any]] = {}
        self._setup_command_templates()

    def _load_command_specs(self):
        """
        Loads command specifications from the manual's transcription.

        The specification ships with the library as package data, so this works
        from an installed wheel. It previously hunted for the file along three
        relative paths under `tests/` - which an installed package does not have
        - and then read `commands` as a list of dicts when it is a mapping keyed
        by hex code, so every lookup failed and the hard-coded fallback below was
        what actually ran. That fallback misnames several commands, so it is now
        genuinely a last resort and says so when it is used.

        `data_length` and `response_length` count *payload* bytes - what the
        caller of `inject_command` supplies and what comes back after the
        command code. The manual's DLC counts the command code and the CRC as
        well, so it is two larger; taking it verbatim made `validate_command`
        demand two bytes for every command that takes none, and every single
        injection was rejected.
        """
        try:
            commands = get_manual_commands()
            for code_str, cmd_data in commands.items():
                code = int(code_str, 16)
                request = cmd_data.get("request", {})
                response = cmd_data.get("response", {})
                self.command_specs[code] = CommandSpec(
                    code=code,
                    name=cmd_data.get("name", f"Command_{code:02X}"),
                    description=cmd_data.get("description", ""),
                    data_length=_payload_length(request.get("dlc", 0)),
                    response_length=_payload_length(response.get("dlc", 0)),
                    category=cmd_data.get("category", "unknown"),
                    parameters=request.get("parameters", []),
                )
        except (OSError, ValueError, KeyError, AttributeError) as exc:
            logger.warning(
                "Could not load the manual command specification (%s); falling "
                "back to a small hard-coded table whose names are approximate.",
                exc,
            )
            self._setup_basic_command_specs()

    def _setup_basic_command_specs(self):
        """
        Setup basic command specifications as fallback.

        Names and lengths come from `constants` and the manual. The version
        this replaces called 0x80 "Enable Motor" (it is encoder calibration;
        enable is 0xF3), called 0x33 "Read Position" (it counts pulses
        received), and had 0xFD and 0xFE the wrong way round.
        """
        basic_commands = [
            (const.CMD_READ_ENCODER_CARRY, "Read Encoder", "Read encoder carry and value", 0, 6, "status"),
            (const.CMD_READ_MOTOR_SPEED_RPM, "Read Speed", "Read current motor speed in RPM", 0, 2, "status"),
            (const.CMD_READ_PULSES_RECEIVED, "Read Pulses Received", "Read the number of pulses received", 0, 4, "status"),
            (const.CMD_QUERY_MOTOR_STATUS, "Query Motor Status", "Get motor enable/status", 0, 1, "status"),
            (const.CMD_RUN_SPEED_MODE, "Speed Mode", "Run continuously at a speed and direction", 3, 1, "motion"),
            (const.CMD_RUN_POSITION_MODE_RELATIVE_PULSES, "Position Mode 1", "Move a relative number of pulses", 6, 1, "motion"),
            (const.CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES, "Position Mode 2", "Move to an absolute pulse count", 6, 1, "motion"),
            (const.CMD_EMERGENCY_STOP, "Emergency Stop", "Stop motor immediately", 0, 1, "motion"),
            (const.CMD_ENABLE_MOTOR, "Enable Motor", "Enable or disable the motor", 1, 1, "control"),
            (const.CMD_CALIBRATE_ENCODER, "Calibrate Encoder", "Run encoder calibration", 0, 1, "control"),
            (const.CMD_SET_WORK_MODE, "Set Work Mode", "Set operating mode", 1, 1, "config"),
            (const.CMD_SET_WORKING_CURRENT, "Set Working Current", "Set motor working current in mA", 2, 1, "config"),
            (const.CMD_SET_SUBDIVISION, "Set Subdivision", "Set step subdivision", 1, 1, "config"),
        ]

        for code, name, desc, data_len, resp_len, category in basic_commands:
            self.command_specs[code] = CommandSpec(
                code=code,
                name=name,
                description=desc,
                data_length=data_len,
                response_length=resp_len,
                category=category,
                parameters=[]
            )

    def _setup_command_templates(self):
        """
        Setup pre-defined command templates for common operations.

        Every entry here is a frame that must survive `validate_command` and
        then mean what its name says on a real motor, so the codes and payloads
        come from `constants` and the manual rather than from round numbers.
        Eight of the eleven were wrong: `enable` sent 0x80 (calibrate), the two
        `move_*` templates sent 0xFD (relative pulses) while describing absolute
        positions and carried four bytes where the frame takes six, the speed
        templates put the direction bit in the wrong byte, and the current
        templates passed a percentage to a command that takes milliamps.
        """
        # The simulator's default motor: 200 full steps at 1/16 microstepping.
        pulses_per_rev = 200 * 16

        def _abs_pulses(speed: int, accel: int, pulses: int) -> List[int]:
            """Payload for 0xFE: speed(uint16 BE), accel(uint8), pulses(int24 BE)."""
            return [
                (speed >> 8) & 0xFF, speed & 0xFF, accel & 0xFF,
                (pulses >> 16) & 0xFF, (pulses >> 8) & 0xFF, pulses & 0xFF,
            ]

        def _speed_mode(clockwise: bool, speed: int, accel: int) -> List[int]:
            """Payload for 0xF6: bit7 of byte 0 is direction, 1 = clockwise."""
            return [
                (0x80 if clockwise else 0x00) | ((speed >> 8) & 0x0F),
                speed & 0xFF,
                accel & 0xFF,
            ]

        self.command_templates = {
            "enable": {
                "name": "Enable Motor",
                "code": const.CMD_ENABLE_MOTOR,
                "data": [0x01],
                "description": "Enable the selected motor"
            },
            "disable": {
                "name": "Disable Motor",
                "code": const.CMD_ENABLE_MOTOR,
                "data": [0x00],
                "description": "Disable the selected motor"
            },
            "stop": {
                "name": "Emergency Stop",
                "code": const.CMD_EMERGENCY_STOP,
                "data": [],
                "description": "Stop motor immediately"
            },
            "read_position": {
                "name": "Read Encoder Position",
                "code": const.CMD_READ_ENCODER_CARRY,
                "data": [],
                "description": "Read current encoder position"
            },
            "read_speed": {
                "name": "Read Current Speed",
                "code": const.CMD_READ_MOTOR_SPEED_RPM,
                "data": [],
                "description": "Read current motor speed"
            },
            "move_cw_slow": {
                "name": "Move Clockwise (Slow)",
                "code": const.CMD_RUN_SPEED_MODE,
                "data": _speed_mode(clockwise=True, speed=50, accel=2),
                "description": "Run clockwise at speed parameter 50"
            },
            "move_ccw_slow": {
                "name": "Move Counter-Clockwise (Slow)",
                "code": const.CMD_RUN_SPEED_MODE,
                "data": _speed_mode(clockwise=False, speed=50, accel=2),
                "description": "Run counter-clockwise at speed parameter 50"
            },
            "move_home": {
                "name": "Move to Home Position",
                "code": const.CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES,
                "data": _abs_pulses(speed=100, accel=2, pulses=0),
                "description": "Move to absolute pulse 0"
            },
            "move_90deg": {
                "name": "Move to 90 Degrees",
                "code": const.CMD_RUN_POSITION_MODE_ABSOLUTE_PULSES,
                "data": _abs_pulses(speed=100, accel=2, pulses=pulses_per_rev // 4),
                "description": "Move to the absolute pulse count a quarter turn from zero"
            },
            "set_high_current": {
                "name": "Set High Current",
                "code": const.CMD_SET_WORKING_CURRENT,
                "data": [0x06, 0x40],  # 1600 mA, big-endian
                "description": "Set the working current to 1600 mA"
            },
            "set_low_current": {
                "name": "Set Low Current",
                "code": const.CMD_SET_WORKING_CURRENT,
                "data": [0x03, 0x20],  # 800 mA, big-endian
                "description": "Set the working current to 800 mA"
            }
        }

    def get_command_spec(self, command_code: int) -> Optional[CommandSpec]:
        """Get command specification for a given command code"""
        return self.command_specs.get(command_code)

    def get_available_templates(self) -> Dict[str, Dict[str, Any]]:
        """Get all available command templates"""
        return self.command_templates.copy()

    def validate_command(self, command_code: int, data_bytes: List[int]) -> Tuple[bool, str]:
        """
        Validate a command before injection.
        
        Args:
            command_code: The command code to validate
            data_bytes: The data bytes for the command
            
        Returns:
            Tuple of (is_valid, error_message)
        """
        # Check if command code is known
        spec = self.get_command_spec(command_code)
        if not spec:
            return False, f"Unknown command code: 0x{command_code:02X}"

        # Check data length
        if spec.data_length > 0 and len(data_bytes) != spec.data_length:
            return False, f"Expected {spec.data_length} data bytes, got {len(data_bytes)}"

        # Check data byte values
        for i, byte_val in enumerate(data_bytes):
            if not (0 <= byte_val <= 255):
                return False, f"Invalid byte value at position {i}: {byte_val} (must be 0-255)"

        return True, "Command is valid"

    def calculate_crc(self, motor_id: int, command_code: int, data_bytes: List[int]) -> int:
        """Calculate CRC for a command"""
        crc = (motor_id + command_code + sum(data_bytes)) & 0xFF
        return crc

    async def inject_command(
        self,
        motor_id: int,
        command_code: int,
        data_bytes: List[int],
        expect_response: bool = True
    ) -> InjectedCommand:
        """
        Inject a command into the specified motor.
        
        Args:
            motor_id: Target motor CAN ID
            command_code: Command code to execute
            data_bytes: Data bytes for the command
            expect_response: Whether to wait for a response
            
        Returns:
            InjectedCommand record with execution details
        """
        start_time = time.time()

        # Validate command
        is_valid, error_msg = self.validate_command(command_code, data_bytes)
        if not is_valid:
            return InjectedCommand(
                timestamp=start_time,
                motor_id=motor_id,
                command_code=command_code,
                command_name=f"Unknown_0x{command_code:02X}",
                data_bytes=bytes(data_bytes),
                response_data=None,
                execution_time_ms=0.0,
                success=False,
                error_message=error_msg
            )

        # Get command spec
        spec = self.get_command_spec(command_code)
        command_name = spec.name if spec else f"Command_0x{command_code:02X}"

        # Check if motor exists
        motor = self.virtual_can_bus.simulated_motors.get(motor_id)
        if not motor:
            return InjectedCommand(
                timestamp=start_time,
                motor_id=motor_id,
                command_code=command_code,
                command_name=command_name,
                data_bytes=bytes(data_bytes),
                response_data=None,
                execution_time_ms=0.0,
                success=False,
                error_message=f"Motor {motor_id} not found"
            )

        try:
            # `process_command` is synchronous and hands back the immediate
            # reply as a `(can_id, payload)` tuple; anything it answers later -
            # a move completing, say - arrives through the callback instead.
            # This used to `await` the call (a tuple is not awaitable) and pass
            # a one-argument callback, so no injection had ever reached a motor.
            async_response: Optional[bytes] = None
            response_received = asyncio.Event()

            async def response_callback(_resp_can_id: int, resp_payload: bytes):
                nonlocal async_response
                async_response = resp_payload
                response_received.set()

            # The motor keeps whichever callback it was last given, so an
            # injection would otherwise divert a connected client's completion
            # frames to us for good. Put the bus's callback back afterwards.
            previous_callback = motor._send_completion_callback

            response_tuple = motor.process_command(
                command_code,
                bytes(data_bytes),
                response_callback,
            )

            if response_tuple is not None:
                response_data = response_tuple[1]
            elif expect_response:
                # Nothing came back immediately: the motor is answering through
                # the callback, so give it a moment to do so.
                try:
                    await asyncio.wait_for(response_received.wait(), timeout=1.0)
                except asyncio.TimeoutError:
                    pass  # Continue without response
                response_data = async_response
            else:
                response_data = None

            motor._send_completion_callback = previous_callback

            execution_time = (time.time() - start_time) * 1000

            # A command that asked for a reply and got none did not reach the
            # motor in any useful sense; saying otherwise is how the injector
            # looked healthy while injecting nothing.
            answered = response_data is not None or not expect_response

            # Create command record
            command_record = InjectedCommand(
                timestamp=start_time,
                motor_id=motor_id,
                command_code=command_code,
                command_name=command_name,
                data_bytes=bytes(data_bytes),
                response_data=response_data,
                execution_time_ms=execution_time,
                success=answered,
                error_message=None if answered else "Motor sent no response",
            )

            # Add to history
            self.injected_commands.append(command_record)
            if len(self.injected_commands) > self.max_history:
                self.injected_commands.pop(0)

            return command_record

        except Exception as e:
            execution_time = (time.time() - start_time) * 1000

            return InjectedCommand(
                timestamp=start_time,
                motor_id=motor_id,
                command_code=command_code,
                command_name=command_name,
                data_bytes=bytes(data_bytes),
                response_data=None,
                execution_time_ms=execution_time,
                success=False,
                error_message=str(e)
            )

    async def inject_template_command(
        self,
        motor_id: int,
        template_name: str
    ) -> InjectedCommand:
        """
        Inject a pre-defined template command.
        
        Args:
            motor_id: Target motor CAN ID
            template_name: Name of the template to use
            
        Returns:
            InjectedCommand record with execution details
        """
        template = self.command_templates.get(template_name)
        if not template:
            return InjectedCommand(
                timestamp=time.time(),
                motor_id=motor_id,
                command_code=0,
                command_name="Invalid Template",
                data_bytes=b"",
                response_data=None,
                execution_time_ms=0.0,
                success=False,
                error_message=f"Template '{template_name}' not found"
            )

        return await self.inject_command(
            motor_id=motor_id,
            command_code=template['code'],
            data_bytes=template['data']
        )

    def get_command_history(self, limit: int = 50) -> List[InjectedCommand]:
        """Get recent command injection history"""
        return self.injected_commands[-limit:] if self.injected_commands else []

    def clear_command_history(self):
        """Clear the command injection history"""
        self.injected_commands.clear()

    def get_command_statistics(self) -> Dict[str, Any]:
        """Get statistics about injected commands"""
        if not self.injected_commands:
            return {
                "total_commands": 0,
                "successful_commands": 0,
                "failed_commands": 0,
                "average_execution_time_ms": 0.0,
                "most_used_commands": [],
                "motor_usage": {}
            }

        total = len(self.injected_commands)
        successful = sum(1 for cmd in self.injected_commands if cmd.success)
        failed = total - successful

        # Calculate average execution time
        total_time = sum(cmd.execution_time_ms for cmd in self.injected_commands)
        avg_time = total_time / total if total > 0 else 0.0

        # Count command usage
        command_counts = {}
        motor_counts = {}

        for cmd in self.injected_commands:
            command_counts[cmd.command_name] = command_counts.get(cmd.command_name, 0) + 1
            motor_counts[cmd.motor_id] = motor_counts.get(cmd.motor_id, 0) + 1

        # Sort by usage
        most_used = sorted(command_counts.items(), key=lambda x: x[1], reverse=True)[:5]

        return {
            "total_commands": total,
            "successful_commands": successful,
            "failed_commands": failed,
            "success_rate": (successful / total * 100) if total > 0 else 0.0,
            "average_execution_time_ms": avg_time,
            "most_used_commands": most_used,
            "motor_usage": motor_counts
        }

    async def run_test_scenario(
        self,
        motor_id: int,
        scenario_name: str = "basic_movement"
    ) -> List[InjectedCommand]:
        """
        Run a pre-defined test scenario.
        
        Args:
            motor_id: Target motor CAN ID
            scenario_name: Name of the scenario to run
            
        Returns:
            List of executed commands
        """
        scenarios = {
            "basic_movement": [
                "enable",
                "read_position",
                "move_cw_slow",
                "read_speed",
                "stop",
                "disable"
            ],
            "position_test": [
                "enable",
                "move_home",
                "read_position",
                "move_90deg",
                "read_position",
                "move_home",
                "disable"
            ],
            "current_test": [
                "enable",
                "set_low_current",
                "move_cw_slow",
                "set_high_current",
                "stop",
                "disable"
            ],
            "status_check": [
                "read_position",
                "read_speed",
                "enable",
                "disable"
            ]
        }

        template_sequence = scenarios.get(scenario_name, [])
        if not template_sequence:
            return []

        results = []
        for template_name in template_sequence:
            result = await self.inject_template_command(motor_id, template_name)
            results.append(result)
            # Small delay between commands
            await asyncio.sleep(0.1)

        return results
