"""
LLM-friendly debugging interface for MKS servo simulator.

This module provides structured debugging information optimized for
Large Language Models like Claude Code to effectively debug and
analyze mks-servo-can based applications.
"""

import time
import json
from typing import Dict, Any, List, Optional, TYPE_CHECKING
from dataclasses import dataclass, asdict
from collections import deque

if TYPE_CHECKING:
    from ..motor_model import MotorModel
    from ..virtual_can_bus import VirtualCANBus

# Load manual command specifications for reference
import sys
from pathlib import Path
try:
    # Try to load command specifications
    test_dir = Path(__file__).parent.parent.parent.parent.parent
    fixtures_path = test_dir / "tests" / "fixtures" / "manual_commands_v106.json"
    if fixtures_path.exists():
        with open(fixtures_path) as f:
            MANUAL_SPEC = json.load(f)
        MANUAL_COMMANDS = MANUAL_SPEC["commands"]
    else:
        MANUAL_COMMANDS = {}
except Exception:
    MANUAL_COMMANDS = {}


@dataclass
class CommandRecord:
    """Record of a CAN command execution"""
    timestamp: float
    motor_id: int
    command_code: int
    command_name: str
    parameters: Dict[str, Any]
    response_time_ms: float
    success: bool
    error_message: Optional[str] = None


@dataclass
class ErrorRecord:
    """Record of an error condition"""
    timestamp: float
    motor_id: int
    error_type: str
    description: str
    context: Dict[str, Any]


class LLMDebugInterface:
    """
    Provides structured debugging information for LLMs.
    
    This interface creates structured, JSON-serializable data that allows
    LLMs to effectively debug mks-servo-can applications by providing:
    - Real-time system status
    - Command execution history
    - Error tracking and analysis
    - State validation capabilities
    """
    
    def __init__(self, motors: Dict[int, 'MotorModel'], can_bus: 'VirtualCANBus'):
        """
        Initialize LLM debug interface.
        
        Args:
            motors: Dictionary of motor ID to MotorModel instances
            can_bus: VirtualCANBus instance managing communication
        """
        self.motors = motors
        self.can_bus = can_bus
        self.command_history: deque = deque(maxlen=1000)
        self.recent_errors: deque = deque(maxlen=100)
        self.start_time = time.time()
        
        # Statistics tracking
        self._message_count = 0
        self._last_message_time = time.time()
        self._message_times = deque(maxlen=50)  # For rate calculation
    
    def get_system_status(self) -> Dict[str, Any]:
        """
        Return complete system state in structured format.
        
        Returns:
            Dictionary containing comprehensive system status suitable
            for LLM analysis and debugging.
        """
        current_time = time.time()
        
        return {
            "timestamp": current_time,
            "uptime_seconds": current_time - self.start_time,
            "simulator_status": "running",
            "motors": {
                str(motor_id): self._get_motor_status(motor)
                for motor_id, motor in self.motors.items()
            },
            "communication": {
                "total_messages": self._message_count,
                "messages_per_second": self._calculate_message_rate(),
                "average_latency_ms": self._calculate_average_latency(),
                "last_10_commands": [
                    {
                        "timestamp": cmd.timestamp,
                        "motor_id": cmd.motor_id,
                        "command": f"0x{cmd.command_code:02X}",
                        "name": cmd.command_name,
                        "success": cmd.success,
                        "response_time_ms": cmd.response_time_ms
                    }
                    for cmd in list(self.command_history)[-10:]
                ]
            },
            "errors": [
                asdict(error) for error in list(self.recent_errors)[-10:]
            ],
            "available_commands": len(MANUAL_COMMANDS)
        }
    
    def _get_motor_status(self, motor: 'MotorModel') -> Dict[str, Any]:
        """Get detailed status for a specific motor"""
        return {
            "can_id": motor.can_id,
            "enabled": getattr(motor, 'enabled', True),
            "current_position": getattr(motor, 'encoder_position', 0),
            "current_angle_degrees": getattr(motor, 'current_angle', 0.0),
            "target_position": getattr(motor, 'target_position', 0),
            "current_speed_rpm": getattr(motor, 'current_speed', 0.0),
            "target_speed_rpm": getattr(motor, 'target_speed', 0.0),
            "last_command": {
                "type": getattr(motor, 'last_command_type', None),
                "timestamp": getattr(motor, 'last_command_time', 0),
                "parameters": getattr(motor, 'last_command_params', {})
            },
            "status_flags": {
                "homing": getattr(motor, 'is_homing', False),
                "moving": getattr(motor, 'is_moving', False),
                "error": getattr(motor, 'has_error', False),
                "stalled": getattr(motor, 'is_stalled', False)
            },
            "performance": {
                "load_percentage": getattr(motor, 'load_percentage', 0),
                "temperature_c": getattr(motor, 'temperature', 25),
                "total_commands": getattr(motor, 'total_commands', 0)
            }
        }
    
    def get_motor_status(self, motor_id: int) -> Optional[Dict[str, Any]]:
        """
        Get detailed status for specific motor.
        
        Args:
            motor_id: CAN ID of the motor
            
        Returns:
            Motor status dictionary or None if motor not found
        """
        if motor_id in self.motors:
            return self._get_motor_status(self.motors[motor_id])
        return None
    
    def get_command_history(self, motor_id: Optional[int] = None, limit: int = 50) -> List[Dict]:
        """
        Return recent command history for analysis.
        
        Args:
            motor_id: Optional motor ID filter
            limit: Maximum number of commands to return
            
        Returns:
            List of command records suitable for LLM analysis
        """
        commands = list(self.command_history)
        if motor_id is not None:
            commands = [cmd for cmd in commands if cmd.motor_id == motor_id]
        
        return [
            {
                "timestamp": cmd.timestamp,
                "motor_id": cmd.motor_id,
                "command_code": f"0x{cmd.command_code:02X}",
                "command_name": cmd.command_name,
                "parameters": cmd.parameters,
                "response_time_ms": cmd.response_time_ms,
                "success": cmd.success,
                "error_message": cmd.error_message
            }
            for cmd in commands[-limit:]
        ]
    
    def validate_expected_state(self, expected_state: Dict) -> Dict[str, Any]:
        """
        Validate current state against expected state for testing.
        
        Args:
            expected_state: Dictionary describing expected system state
            
        Returns:
            Validation results with pass/fail status and detailed feedback
        """
        current = self.get_system_status()
        
        validation_results = {
            "passed": True,
            "failures": [],
            "warnings": [],
            "summary": "",
            "timestamp": time.time()
        }
        
        tolerance = expected_state.get("tolerance", 0.1)
        
        # Compare expected vs actual motor states
        for motor_id, expected_motor in expected_state.get("motors", {}).items():
            motor_id_str = str(motor_id)
            
            if motor_id_str not in current["motors"]:
                validation_results["failures"].append(f"Motor {motor_id} not found in simulator")
                continue
                
            actual_motor = current["motors"][motor_id_str]
            
            for field, expected_value in expected_motor.items():
                if field not in actual_motor:
                    validation_results["warnings"].append(f"Motor {motor_id} field '{field}' not found")
                    continue
                
                actual_value = actual_motor[field]
                
                # Handle different value types
                if isinstance(expected_value, (int, float)):
                    if abs(actual_value - expected_value) > tolerance:
                        validation_results["failures"].append(
                            f"Motor {motor_id} {field}: expected {expected_value}, got {actual_value} (tolerance: {tolerance})"
                        )
                elif isinstance(expected_value, bool):
                    if actual_value != expected_value:
                        validation_results["failures"].append(
                            f"Motor {motor_id} {field}: expected {expected_value}, got {actual_value}"
                        )
                elif isinstance(expected_value, str):
                    if actual_value != expected_value:
                        validation_results["failures"].append(
                            f"Motor {motor_id} {field}: expected '{expected_value}', got '{actual_value}'"
                        )
        
        # Validate communication expectations
        if "communication" in expected_state:
            comm_expected = expected_state["communication"]
            comm_actual = current["communication"]
            
            for field, expected_value in comm_expected.items():
                if field in comm_actual:
                    actual_value = comm_actual[field]
                    if isinstance(expected_value, (int, float)) and isinstance(actual_value, (int, float)):
                        if abs(actual_value - expected_value) > tolerance:
                            validation_results["failures"].append(
                                f"Communication {field}: expected {expected_value}, got {actual_value}"
                            )
        
        validation_results["passed"] = len(validation_results["failures"]) == 0
        validation_results["summary"] = (
            f"{'PASS' if validation_results['passed'] else 'FAIL'}: "
            f"{len(validation_results['failures'])} failures, {len(validation_results['warnings'])} warnings"
        )
        
        return validation_results
    
    def record_command(self, motor_id: int, command_code: int, command_name: str, 
                      parameters: Dict, response_time: float, success: bool, 
                      error_message: Optional[str] = None):
        """
        Record a command execution for history tracking.
        
        Args:
            motor_id: CAN ID of target motor
            command_code: Command code (e.g., 0x30)
            command_name: Human-readable command name
            parameters: Command parameters
            response_time: Response time in seconds
            success: Whether command succeeded
            error_message: Optional error description
        """
        record = CommandRecord(
            timestamp=time.time(),
            motor_id=motor_id,
            command_code=command_code,
            command_name=command_name,
            parameters=parameters,
            response_time_ms=response_time * 1000,
            success=success,
            error_message=error_message
        )
        self.command_history.append(record)
        
        # Update statistics
        self._message_count += 1
        self._message_times.append(time.time())
    
    def record_error(self, motor_id: int, error_type: str, description: str, context: Dict):
        """
        Record an error for debugging analysis.
        
        Args:
            motor_id: Motor ID where error occurred
            error_type: Type of error (e.g., "communication", "protocol", "state")
            description: Human-readable error description
            context: Additional context information
        """
        error = ErrorRecord(
            timestamp=time.time(),
            motor_id=motor_id,
            error_type=error_type,
            description=description,
            context=context
        )
        self.recent_errors.append(error)
    
    def get_available_commands(self) -> Dict[str, Any]:
        """
        Return list of all available commands for LLM reference.
        
        Returns:
            Dictionary with command information suitable for LLM understanding
        """
        if not MANUAL_COMMANDS:
            return {
                "commands": [],
                "categories": {},
                "note": "Manual commands specification not loaded"
            }
        
        return {
            "commands": [
                {
                    "code": code,
                    "name": spec["name"],
                    "description": spec["description"],
                    "category": spec.get("category", "unknown"),
                    "parameters": spec.get("request", {}).get("parameters", [])
                }
                for code, spec in MANUAL_COMMANDS.items()
            ],
            "categories": {
                "status": "Commands for reading motor status and encoder values",
                "motion": "Commands for controlling motor movement",
                "control": "Commands for motor control operations",
                "configuration": "Commands for motor setup and calibration"
            },
            "total_commands": len(MANUAL_COMMANDS)
        }
    
    def get_debug_summary(self) -> str:
        """
        Get a concise debug summary suitable for LLM context.
        
        Returns:
            String summary of current system state
        """
        status = self.get_system_status()
        
        motor_summaries = []
        for motor_id, motor_status in status["motors"].items():
            pos = motor_status["current_position"]
            enabled = motor_status["enabled"]
            moving = motor_status["status_flags"]["moving"]
            motor_summaries.append(f"Motor {motor_id}: pos={pos}, {'enabled' if enabled else 'disabled'}, {'moving' if moving else 'stopped'}")
        
        recent_commands = len(self.command_history)
        recent_errors = len(self.recent_errors)
        uptime = status["uptime_seconds"]
        
        return f"Simulator uptime: {uptime:.1f}s, Motors: {len(status['motors'])}, Commands: {recent_commands}, Errors: {recent_errors}, {', '.join(motor_summaries)}"
    
    def _calculate_message_rate(self) -> float:
        """Calculate current message rate"""
        if len(self._message_times) < 2:
            return 0.0
        
        time_span = self._message_times[-1] - self._message_times[0]
        return len(self._message_times) / time_span if time_span > 0 else 0.0
    
    def _calculate_average_latency(self) -> float:
        """Calculate average command response latency"""
        if not self.command_history:
            return 0.0
        
        recent_commands = list(self.command_history)[-20:]  # Last 20 commands
        latencies = [cmd.response_time_ms for cmd in recent_commands if cmd.success]
        
        return sum(latencies) / len(latencies) if latencies else 0.0
    
    def export_status_json(self, filepath: Optional[str] = None) -> str:
        """
        Export current status as JSON string or file.
        
        Args:
            filepath: Optional file path to save JSON
            
        Returns:
            JSON string representation of current status
        """
        status = self.get_system_status()
        json_str = json.dumps(status, indent=2, default=str)
        
        if filepath:
            with open(filepath, 'w') as f:
                f.write(json_str)
        
        return json_str