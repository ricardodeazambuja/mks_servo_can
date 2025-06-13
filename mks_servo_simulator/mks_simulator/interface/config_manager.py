"""
Configuration Management System for MKS Servo Simulator.

This module provides:
- Live parameter adjustment capabilities
- Configuration profiles (save/load different simulator setups)
- Motor templates for quick motor configuration
- Real-time configuration updates without simulator restart
"""

import json
import os
import logging
from pathlib import Path
from typing import Dict, Any, List, Optional, Union
from dataclasses import dataclass, asdict, field
from datetime import datetime
import asyncio

logger = logging.getLogger(__name__)


@dataclass
class MotorConfig:
    """Configuration for a single motor."""
    can_id: int
    motor_type: str = "GENERIC"
    steps_per_rev: int = 16384
    max_current: int = 1000
    max_speed: int = 3000
    initial_position: float = 0.0
    enable_on_start: bool = False
    position_limits: Optional[Dict[str, float]] = None
    
    def __post_init__(self):
        if self.position_limits is None:
            self.position_limits = {"min": -360.0, "max": 360.0}


@dataclass
class SimulatorConfig:
    """Main simulator configuration."""
    # Network settings
    host: str = "localhost"
    port: int = 6789
    debug_api_port: int = 8765
    
    # Simulation settings
    latency_ms: float = 2.0
    log_level: str = "INFO"
    
    # Interface settings
    refresh_rate: int = 200
    no_color: bool = False
    
    # Feature flags
    json_output: bool = False
    debug_api: bool = False
    dashboard: bool = False
    
    # Motor configurations
    motors: List[MotorConfig] = field(default_factory=list)
    
    # Metadata
    name: str = "default"
    description: str = "Default simulator configuration"
    created_at: str = field(default_factory=lambda: datetime.now().isoformat())
    modified_at: str = field(default_factory=lambda: datetime.now().isoformat())


class ConfigurationManager:
    """Manages simulator configuration with live updates and profiles."""
    
    def __init__(self, config_dir: Optional[str] = None):
        """Initialize configuration manager.
        
        Args:
            config_dir: Directory to store configuration files. Defaults to ~/.mks_simulator_config
        """
        if config_dir is None:
            config_dir = os.path.expanduser("~/.mks_simulator_config")
        
        self.config_dir = Path(config_dir)
        self.config_dir.mkdir(parents=True, exist_ok=True)
        
        self.profiles_dir = self.config_dir / "profiles"
        self.templates_dir = self.config_dir / "templates"
        self.profiles_dir.mkdir(exist_ok=True)
        self.templates_dir.mkdir(exist_ok=True)
        
        self.current_config: Optional[SimulatorConfig] = None
        self.config_file = self.config_dir / "current_config.json"
        
        # Live update callbacks
        self._update_callbacks: List[callable] = []
        
        logger.info(f"Configuration manager initialized with config dir: {self.config_dir}")
    
    def add_update_callback(self, callback: callable):
        """Add callback to be called when configuration is updated live."""
        self._update_callbacks.append(callback)
    
    def remove_update_callback(self, callback: callable):
        """Remove update callback."""
        if callback in self._update_callbacks:
            self._update_callbacks.remove(callback)
    
    async def _notify_update_callbacks(self, config_change: Dict[str, Any]):
        """Notify all callbacks of configuration changes."""
        for callback in self._update_callbacks:
            try:
                if asyncio.iscoroutinefunction(callback):
                    await callback(config_change)
                else:
                    callback(config_change)
            except Exception as e:
                logger.error(f"Error in config update callback: {e}")
    
    def create_default_config(self, num_motors: int = 1, start_can_id: int = 1) -> SimulatorConfig:
        """Create a default configuration with specified number of motors."""
        motors = []
        for i in range(num_motors):
            motor = MotorConfig(
                can_id=start_can_id + i,
                motor_type="GENERIC",
                steps_per_rev=16384
            )
            motors.append(motor)
        
        config = SimulatorConfig(motors=motors)
        return config
    
    def load_config(self, config_name: str = "current") -> Optional[SimulatorConfig]:
        """Load configuration from file.
        
        Args:
            config_name: Name of configuration to load. "current" loads the current active config.
        
        Returns:
            SimulatorConfig if found, None otherwise
        """
        if config_name == "current":
            config_file = self.config_file
        else:
            config_file = self.profiles_dir / f"{config_name}.json"
        
        if not config_file.exists():
            logger.warning(f"Configuration file not found: {config_file}")
            return None
        
        try:
            with open(config_file, 'r') as f:
                config_data = json.load(f)
            
            # Convert motor configurations
            motors = []
            for motor_data in config_data.get('motors', []):
                motor = MotorConfig(**motor_data)
                motors.append(motor)
            
            # Remove motors from config_data and create config
            config_data['motors'] = motors
            config = SimulatorConfig(**config_data)
            
            logger.info(f"Loaded configuration: {config_name}")
            return config
            
        except Exception as e:
            logger.error(f"Error loading configuration {config_name}: {e}")
            return None
    
    def save_config(self, config: SimulatorConfig, config_name: str = "current") -> bool:
        """Save configuration to file.
        
        Args:
            config: Configuration to save
            config_name: Name to save configuration as. "current" saves as active config.
        
        Returns:
            True if saved successfully, False otherwise
        """
        if config_name == "current":
            config_file = self.config_file
        else:
            config_file = self.profiles_dir / f"{config_name}.json"
        
        try:
            # Update modification time
            config.modified_at = datetime.now().isoformat()
            
            # Convert to dict for JSON serialization
            config_dict = asdict(config)
            
            with open(config_file, 'w') as f:
                json.dump(config_dict, f, indent=2)
            
            logger.info(f"Saved configuration: {config_name}")
            return True
            
        except Exception as e:
            logger.error(f"Error saving configuration {config_name}: {e}")
            return False
    
    def list_profiles(self) -> List[str]:
        """List all available configuration profiles."""
        profiles = []
        for profile_file in self.profiles_dir.glob("*.json"):
            profiles.append(profile_file.stem)
        return sorted(profiles)
    
    def delete_profile(self, profile_name: str) -> bool:
        """Delete a configuration profile.
        
        Args:
            profile_name: Name of profile to delete
        
        Returns:
            True if deleted successfully, False otherwise
        """
        if profile_name == "current":
            logger.error("Cannot delete current configuration")
            return False
        
        profile_file = self.profiles_dir / f"{profile_name}.json"
        if not profile_file.exists():
            logger.warning(f"Profile not found: {profile_name}")
            return False
        
        try:
            profile_file.unlink()
            logger.info(f"Deleted profile: {profile_name}")
            return True
        except Exception as e:
            logger.error(f"Error deleting profile {profile_name}: {e}")
            return False
    
    async def update_parameter(self, parameter_path: str, value: Any) -> bool:
        """Update a configuration parameter live.
        
        Args:
            parameter_path: Dot-separated path to parameter (e.g., "latency_ms", "motors.0.max_current")
            value: New value for parameter
        
        Returns:
            True if updated successfully, False otherwise
        """
        if not self.current_config:
            logger.error("No current configuration loaded")
            return False
        
        try:
            # Parse parameter path
            path_parts = parameter_path.split('.')
            
            # Navigate to the target parameter
            target = self.current_config
            for i, part in enumerate(path_parts[:-1]):
                if part.isdigit():
                    # Array index
                    target = target[int(part)]
                elif hasattr(target, part):
                    target = getattr(target, part)
                else:
                    logger.error(f"Invalid parameter path: {parameter_path}")
                    return False
            
            # Set the final parameter
            final_key = path_parts[-1]
            if hasattr(target, final_key):
                setattr(target, final_key, value)
            else:
                logger.error(f"Invalid parameter: {final_key}")
                return False
            
            # Save updated configuration
            self.save_config(self.current_config)
            
            # Notify callbacks
            change_info = {
                "parameter": parameter_path,
                "old_value": getattr(target, final_key),
                "new_value": value,
                "timestamp": datetime.now().isoformat()
            }
            await self._notify_update_callbacks(change_info)
            
            logger.info(f"Updated parameter {parameter_path} = {value}")
            return True
            
        except Exception as e:
            logger.error(f"Error updating parameter {parameter_path}: {e}")
            return False
    
    def get_motor_templates(self) -> Dict[str, MotorConfig]:
        """Get available motor templates."""
        templates = {}
        
        # Built-in templates
        servo42d = MotorConfig(
            can_id=1,
            motor_type="SERVO42D",
            steps_per_rev=16384,
            max_current=800,
            max_speed=2000
        )
        servo42d.description = "MKS SERVO42D stepper motor"
        templates["servo42d"] = servo42d
        
        servo57d = MotorConfig(
            can_id=1,
            motor_type="SERVO57D",
            steps_per_rev=16384,
            max_current=1500,
            max_speed=3000
        )
        servo57d.description = "MKS SERVO57D stepper motor"
        templates["servo57d"] = servo57d
        
        high_precision = MotorConfig(
            can_id=1,
            motor_type="GENERIC",
            steps_per_rev=51200,  # High resolution
            max_current=1200,
            max_speed=1000  # Lower speed for precision
        )
        high_precision.description = "High precision motor configuration"
        templates["high_precision"] = high_precision
        
        high_speed = MotorConfig(
            can_id=1,
            motor_type="GENERIC",
            steps_per_rev=16384,
            max_current=2000,
            max_speed=5000
        )
        high_speed.description = "High speed motor configuration"
        templates["high_speed"] = high_speed
        
        # Load custom templates from files
        for template_file in self.templates_dir.glob("*.json"):
            try:
                with open(template_file, 'r') as f:
                    template_data = json.load(f)
                template = MotorConfig(**template_data)
                templates[template_file.stem] = template
            except Exception as e:
                logger.error(f"Error loading template {template_file}: {e}")
        
        return templates
    
    def save_motor_template(self, name: str, motor_config: MotorConfig) -> bool:
        """Save a motor configuration as a template.
        
        Args:
            name: Template name
            motor_config: Motor configuration to save as template
        
        Returns:
            True if saved successfully, False otherwise
        """
        template_file = self.templates_dir / f"{name}.json"
        
        try:
            template_dict = asdict(motor_config)
            # Remove CAN ID from template (will be set when applied)
            template_dict.pop('can_id', None)
            
            with open(template_file, 'w') as f:
                json.dump(template_dict, f, indent=2)
            
            logger.info(f"Saved motor template: {name}")
            return True
            
        except Exception as e:
            logger.error(f"Error saving motor template {name}: {e}")
            return False
    
    def apply_motor_template(self, motor_index: int, template_name: str) -> bool:
        """Apply a motor template to a specific motor.
        
        Args:
            motor_index: Index of motor to apply template to
            template_name: Name of template to apply
        
        Returns:
            True if applied successfully, False otherwise
        """
        if not self.current_config:
            logger.error("No current configuration loaded")
            return False
        
        if motor_index >= len(self.current_config.motors):
            logger.error(f"Motor index {motor_index} out of range")
            return False
        
        templates = self.get_motor_templates()
        if template_name not in templates:
            logger.error(f"Template not found: {template_name}")
            return False
        
        try:
            template = templates[template_name]
            current_motor = self.current_config.motors[motor_index]
            
            # Keep the current CAN ID
            original_can_id = current_motor.can_id
            
            # Apply template
            self.current_config.motors[motor_index] = template
            self.current_config.motors[motor_index].can_id = original_can_id
            
            # Save configuration
            self.save_config(self.current_config)
            
            logger.info(f"Applied template '{template_name}' to motor {motor_index}")
            return True
            
        except Exception as e:
            logger.error(f"Error applying template {template_name} to motor {motor_index}: {e}")
            return False
    
    def get_config_summary(self) -> Dict[str, Any]:
        """Get a summary of the current configuration."""
        if not self.current_config:
            return {"status": "No configuration loaded"}
        
        return {
            "name": self.current_config.name,
            "description": self.current_config.description,
            "num_motors": len(self.current_config.motors),
            "host": self.current_config.host,
            "port": self.current_config.port,
            "latency_ms": self.current_config.latency_ms,
            "features": {
                "dashboard": self.current_config.dashboard,
                "debug_api": self.current_config.debug_api,
                "json_output": self.current_config.json_output
            },
            "motors": [
                {
                    "can_id": motor.can_id,
                    "type": motor.motor_type,
                    "steps_per_rev": motor.steps_per_rev
                }
                for motor in self.current_config.motors
            ]
        }


class LiveConfigurationInterface:
    """Interface for live configuration updates during simulation."""
    
    def __init__(self, config_manager: ConfigurationManager, virtual_can_bus=None):
        """Initialize live configuration interface.
        
        Args:
            config_manager: Configuration manager instance
            virtual_can_bus: Optional reference to virtual CAN bus for live updates
        """
        self.config_manager = config_manager
        self.virtual_can_bus = virtual_can_bus
        
        # Register for configuration updates
        config_manager.add_update_callback(self._handle_config_update)
        
        logger.info("Live configuration interface initialized")
    
    async def _handle_config_update(self, change_info: Dict[str, Any]):
        """Handle configuration updates and apply them to running simulation."""
        parameter = change_info["parameter"]
        new_value = change_info["new_value"]
        
        logger.info(f"Applying live configuration update: {parameter} = {new_value}")
        
        try:
            if parameter == "latency_ms" and self.virtual_can_bus:
                # Update CAN bus latency
                self.virtual_can_bus.set_latency(new_value)
                logger.info(f"Updated CAN bus latency to {new_value}ms")
            
            elif parameter.startswith("motors.") and "max_current" in parameter:
                # Update motor current limits
                motor_index = int(parameter.split('.')[1])
                if self.virtual_can_bus and motor_index < len(self.virtual_can_bus.simulated_motors):
                    motor = self.virtual_can_bus.simulated_motors[motor_index]
                    motor.max_current = new_value
                    logger.info(f"Updated motor {motor_index} max current to {new_value}")
            
            elif parameter.startswith("motors.") and "max_speed" in parameter:
                # Update motor speed limits
                motor_index = int(parameter.split('.')[1])
                if self.virtual_can_bus and motor_index < len(self.virtual_can_bus.simulated_motors):
                    motor = self.virtual_can_bus.simulated_motors[motor_index]
                    motor.max_speed = new_value
                    logger.info(f"Updated motor {motor_index} max speed to {new_value}")
            
            # Add more parameter handlers as needed
            
        except Exception as e:
            logger.error(f"Error applying live configuration update: {e}")
    
    def get_adjustable_parameters(self) -> Dict[str, Dict[str, Any]]:
        """Get list of parameters that can be adjusted live."""
        return {
            "latency_ms": {
                "description": "CAN bus latency in milliseconds",
                "type": "float",
                "min": 0.1,
                "max": 100.0,
                "current": self.config_manager.current_config.latency_ms if self.config_manager.current_config else None
            },
            "refresh_rate": {
                "description": "Dashboard refresh rate in milliseconds",
                "type": "int",
                "min": 50,
                "max": 2000,
                "current": self.config_manager.current_config.refresh_rate if self.config_manager.current_config else None
            },
            "motors.*.max_current": {
                "description": "Maximum current for motor (mA)",
                "type": "int",
                "min": 100,
                "max": 3000,
                "pattern": "motors.{motor_index}.max_current"
            },
            "motors.*.max_speed": {
                "description": "Maximum speed for motor (steps/s)",
                "type": "int",
                "min": 100,
                "max": 10000,
                "pattern": "motors.{motor_index}.max_speed"
            }
        }
    
    async def update_parameter(self, parameter: str, value: Any) -> bool:
        """Update a parameter with validation."""
        adjustable = self.get_adjustable_parameters()
        
        # Check if parameter is adjustable
        if parameter not in adjustable and not any(param.replace('*', '\\d+') in parameter for param in adjustable):
            logger.error(f"Parameter '{parameter}' is not adjustable during runtime")
            return False
        
        # Apply the update
        return await self.config_manager.update_parameter(parameter, value)