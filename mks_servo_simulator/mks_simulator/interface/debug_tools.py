"""
Debug tools for the MKS servo simulator.

Provides command injection, testing utilities, and debugging features.
"""

import asyncio
import json
import time
from typing import Dict, List, Optional, Tuple, Any, TYPE_CHECKING
from dataclasses import dataclass
from pathlib import Path

if TYPE_CHECKING:
    from ..virtual_can_bus import VirtualCANBus
    from ..motor_model import SimulatedMotor
    from .llm_debug_interface import LLMDebugInterface


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
        """Load command specifications from the manual database"""
        try:
            # Look for the manual commands JSON file
            possible_paths = [
                Path("tests/fixtures/manual_commands_v106.json"),
                Path("../tests/fixtures/manual_commands_v106.json"),
                Path("../../tests/fixtures/manual_commands_v106.json")
            ]
            
            commands_file = None
            for path in possible_paths:
                if path.exists():
                    commands_file = path
                    break
            
            if commands_file:
                with open(commands_file, 'r') as f:
                    data = json.load(f)
                
                # Parse command specifications
                for cmd_data in data.get('commands', []):
                    code = cmd_data.get('code')
                    if code is not None:
                        self.command_specs[code] = CommandSpec(
                            code=code,
                            name=cmd_data.get('name', f'Command_{code:02X}'),
                            description=cmd_data.get('description', ''),
                            data_length=cmd_data.get('data_length', 0),
                            response_length=cmd_data.get('response_length', 0),
                            category=cmd_data.get('category', 'unknown'),
                            parameters=cmd_data.get('parameters', [])
                        )
        except Exception as e:
            # Fallback to basic command specs
            self._setup_basic_command_specs()
    
    def _setup_basic_command_specs(self):
        """Setup basic command specifications as fallback"""
        basic_commands = [
            (0x30, "Read Encoder", "Read current encoder position", 0, 4, "status"),
            (0x32, "Read Speed", "Read current motor speed", 0, 2, "status"),
            (0x33, "Read Position", "Read current position in degrees", 0, 4, "status"),
            (0xF1, "Query Motor Status", "Get motor enable/status", 0, 1, "status"),
            (0xF6, "Speed Mode", "Set motor speed and direction", 4, 0, "motion"),
            (0xFD, "Position Mode 1", "Move to absolute position", 4, 0, "motion"),
            (0xFE, "Position Mode 2", "Move relative position", 4, 0, "motion"),
            (0xF7, "Emergency Stop", "Stop motor immediately", 0, 0, "motion"),
            (0x80, "Enable Motor", "Enable/disable motor", 1, 0, "control"),
            (0x82, "Set Work Mode", "Set operating mode", 1, 0, "config"),
            (0x83, "Set Current", "Set motor current", 2, 0, "config"),
            (0x84, "Set Subdivision", "Set step subdivision", 1, 0, "config"),
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
        """Setup pre-defined command templates for common operations"""
        self.command_templates = {
            "enable": {
                "name": "Enable Motor",
                "code": 0x80,
                "data": [0x01],
                "description": "Enable the selected motor"
            },
            "disable": {
                "name": "Disable Motor",
                "code": 0x80,
                "data": [0x00],
                "description": "Disable the selected motor"
            },
            "stop": {
                "name": "Emergency Stop",
                "code": 0xF7,
                "data": [],
                "description": "Stop motor immediately"
            },
            "read_position": {
                "name": "Read Encoder Position",
                "code": 0x30,
                "data": [],
                "description": "Read current encoder position"
            },
            "read_speed": {
                "name": "Read Current Speed",
                "code": 0x32,
                "data": [],
                "description": "Read current motor speed"
            },
            "move_cw_slow": {
                "name": "Move Clockwise (Slow)",
                "code": 0xF6,
                "data": [0x00, 0x00, 0x32, 0x00],  # 50 RPM clockwise
                "description": "Move clockwise at 50 RPM"
            },
            "move_ccw_slow": {
                "name": "Move Counter-Clockwise (Slow)",
                "code": 0xF6,
                "data": [0x01, 0x00, 0x32, 0x00],  # 50 RPM counter-clockwise
                "description": "Move counter-clockwise at 50 RPM"
            },
            "move_home": {
                "name": "Move to Home Position",
                "code": 0xFD,
                "data": [0x00, 0x00, 0x00, 0x00],  # Position 0
                "description": "Move to home position (0 degrees)"
            },
            "move_90deg": {
                "name": "Move to 90 Degrees",
                "code": 0xFD,
                "data": [0x00, 0x40, 0x00, 0x00],  # 90 degrees (16384 steps / 4)
                "description": "Move to 90 degree position"
            },
            "set_high_current": {
                "name": "Set High Current",
                "code": 0x83,
                "data": [0x64, 0x00],  # 100% current
                "description": "Set motor current to 100%"
            },
            "set_low_current": {
                "name": "Set Low Current",
                "code": 0x83,
                "data": [0x32, 0x00],  # 50% current
                "description": "Set motor current to 50%"
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
            # Create a mock callback for response handling
            response_data = None
            response_received = asyncio.Event()
            
            def response_callback(response_bytes: bytes):
                nonlocal response_data
                response_data = response_bytes
                response_received.set()
            
            # Execute the command
            await motor.process_command(
                command_code=command_code,
                data_from_payload=bytes(data_bytes),
                send_completion_callback=response_callback if expect_response else None
            )
            
            # Wait for response if expected
            if expect_response and spec and spec.response_length > 0:
                try:
                    await asyncio.wait_for(response_received.wait(), timeout=1.0)
                except asyncio.TimeoutError:
                    pass  # Continue without response
            
            execution_time = (time.time() - start_time) * 1000
            
            # Create command record
            command_record = InjectedCommand(
                timestamp=start_time,
                motor_id=motor_id,
                command_code=command_code,
                command_name=command_name,
                data_bytes=bytes(data_bytes),
                response_data=response_data,
                execution_time_ms=execution_time,
                success=True,
                error_message=None
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