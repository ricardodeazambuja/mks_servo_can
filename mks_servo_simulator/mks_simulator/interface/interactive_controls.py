"""
Interactive controls for the Rich dashboard.

Handles keyboard input and user interactions for the MKS servo simulator dashboard.
"""

import asyncio
import sys
import termios
import tty
from typing import Optional, Callable, Dict, Any, TYPE_CHECKING
from dataclasses import dataclass

if TYPE_CHECKING:
    from .rich_dashboard import RichDashboard
    from ..virtual_can_bus import VirtualCANBus
    from .config_manager import ConfigurationManager, LiveConfigurationInterface


@dataclass
class KeyBinding:
    """Represents a key binding and its associated action"""
    key: str
    description: str
    action: Callable
    category: str = "general"


class InteractiveController:
    """
    Handles keyboard input and interactive controls for the dashboard.
    
    Features:
    - Non-blocking keyboard input
    - Customizable key bindings
    - Help system
    - Motor selection and control
    """
    
    def __init__(
        self,
        dashboard: "RichDashboard",
        virtual_can_bus: "VirtualCANBus",
        config_manager: Optional["ConfigurationManager"] = None,
        live_config: Optional["LiveConfigurationInterface"] = None
    ):
        """
        Initialize the interactive controller.
        
        Args:
            dashboard: The Rich dashboard instance
            virtual_can_bus: The virtual CAN bus instance
            config_manager: Optional configuration manager for profile management
            live_config: Optional live configuration interface for runtime updates
        """
        self.dashboard = dashboard
        self.virtual_can_bus = virtual_can_bus
        self.config_manager = config_manager
        self.live_config = live_config
        self.running = False
        self.original_tty_settings = None
        
        # State management
        self.selected_motor_id: Optional[int] = None
        self.show_help = False
        self.command_mode = False
        self.command_buffer = ""
        
        # Setup key bindings
        self.key_bindings: Dict[str, KeyBinding] = {}
        self._setup_key_bindings()
    
    def _setup_key_bindings(self):
        """Configure all key bindings"""
        # Dashboard controls
        self.key_bindings[' '] = KeyBinding(
            key=' ',
            description="Pause/Resume dashboard updates",
            action=self._toggle_pause,
            category="dashboard"
        )
        
        self.key_bindings['r'] = KeyBinding(
            key='r',
            description="Restart simulator",
            action=self._restart_simulator,
            category="dashboard"
        )
        
        self.key_bindings['q'] = KeyBinding(
            key='q',
            description="Quit simulator",
            action=self._quit_simulator,
            category="dashboard"
        )
        
        self.key_bindings['h'] = KeyBinding(
            key='h',
            description="Show/Hide help",
            action=self._toggle_help,
            category="dashboard"
        )
        
        # Motor selection
        self.key_bindings['\x1b[A'] = KeyBinding(  # Up arrow
            key='↑',
            description="Select previous motor",
            action=self._select_previous_motor,
            category="motor"
        )
        
        self.key_bindings['\x1b[B'] = KeyBinding(  # Down arrow
            key='↓',
            description="Select next motor",
            action=self._select_next_motor,
            category="motor"
        )
        
        self.key_bindings['\r'] = KeyBinding(  # Enter
            key='Enter',
            description="Show detailed motor info",
            action=self._show_motor_details,
            category="motor"
        )
        
        # Motor controls
        self.key_bindings['e'] = KeyBinding(
            key='e',
            description="Enable/Disable selected motor",
            action=self._toggle_motor_enable,
            category="motor"
        )
        
        self.key_bindings['s'] = KeyBinding(
            key='s',
            description="Stop selected motor",
            action=self._stop_motor,
            category="motor"
        )
        
        self.key_bindings['z'] = KeyBinding(
            key='z',
            description="Zero selected motor position",
            action=self._zero_motor_position,
            category="motor"
        )
        
        # Command mode
        self.key_bindings['c'] = KeyBinding(
            key='c',
            description="Enter command mode",
            action=self._enter_command_mode,
            category="advanced"
        )
        
        # Refresh rate control
        self.key_bindings['+'] = KeyBinding(
            key='+',
            description="Increase refresh rate",
            action=self._increase_refresh_rate,
            category="dashboard"
        )
        
        self.key_bindings['-'] = KeyBinding(
            key='-',
            description="Decrease refresh rate",
            action=self._decrease_refresh_rate,
            category="dashboard"
        )
        
        # Configuration management (only if config manager is available)
        if self.config_manager:
            self.key_bindings['p'] = KeyBinding(
                key='p',
                description="Configuration profiles menu",
                action=self._show_profiles_menu,
                category="config"
            )
            
            self.key_bindings['l'] = KeyBinding(
                key='l',
                description="Live parameter adjustment",
                action=self._live_parameter_menu,
                category="config"
            )
            
            self.key_bindings['m'] = KeyBinding(
                key='m',
                description="Motor templates menu",
                action=self._motor_templates_menu,
                category="config"
            )
            
            self.key_bindings['k'] = KeyBinding(
                key='k',
                description="Save current configuration",
                action=self._save_config_prompt,
                category="config"
            )
    
    def _setup_terminal(self):
        """Configure terminal for non-blocking input"""
        if sys.stdin.isatty():
            self.original_tty_settings = termios.tcgetattr(sys.stdin.fileno())
            tty.setraw(sys.stdin.fileno())
    
    def _restore_terminal(self):
        """Restore original terminal settings"""
        if self.original_tty_settings and sys.stdin.isatty():
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self.original_tty_settings)
    
    async def _get_key_async(self) -> Optional[str]:
        """Get a key press asynchronously without blocking"""
        if not sys.stdin.isatty():
            return None
        
        # Check if input is available
        import select
        if select.select([sys.stdin], [], [], 0) == ([], [], []):
            return None
        
        try:
            key = sys.stdin.read(1)
            
            # Handle escape sequences (arrow keys, etc.)
            if key == '\x1b':
                key += sys.stdin.read(2)
            
            return key
        except:
            return None
    
    def _get_motor_ids(self) -> list:
        """Get sorted list of motor IDs"""
        return sorted(self.virtual_can_bus.simulated_motors.keys())
    
    def _get_selected_motor(self):
        """Get the currently selected motor"""
        if self.selected_motor_id is None:
            return None
        return self.virtual_can_bus.simulated_motors.get(self.selected_motor_id)
    
    # Key action handlers
    def _toggle_pause(self):
        """Toggle dashboard pause state"""
        self.dashboard.toggle_pause()
    
    def _restart_simulator(self):
        """Restart the simulator"""
        self.dashboard.add_event("Restart requested (not implemented)")
        # TODO: Implement simulator restart
    
    def _quit_simulator(self):
        """Quit the simulator"""
        self.dashboard.add_event("Quit requested")
        self.running = False
        # Raise KeyboardInterrupt to trigger graceful shutdown
        import os
        os.kill(os.getpid(), 2)  # Send SIGINT
    
    def _toggle_help(self):
        """Toggle help display"""
        self.show_help = not self.show_help
        self.dashboard.add_event(f"Help {'shown' if self.show_help else 'hidden'}")
    
    def _select_previous_motor(self):
        """Select the previous motor in the list"""
        motor_ids = self._get_motor_ids()
        if not motor_ids:
            return
        
        if self.selected_motor_id is None:
            self.selected_motor_id = motor_ids[-1]
        else:
            try:
                current_index = motor_ids.index(self.selected_motor_id)
                self.selected_motor_id = motor_ids[(current_index - 1) % len(motor_ids)]
            except ValueError:
                self.selected_motor_id = motor_ids[0]
        
        self.dashboard.state.selected_motor_id = self.selected_motor_id
        self.dashboard.add_event(f"Selected motor {self.selected_motor_id}")
    
    def _select_next_motor(self):
        """Select the next motor in the list"""
        motor_ids = self._get_motor_ids()
        if not motor_ids:
            return
        
        if self.selected_motor_id is None:
            self.selected_motor_id = motor_ids[0]
        else:
            try:
                current_index = motor_ids.index(self.selected_motor_id)
                self.selected_motor_id = motor_ids[(current_index + 1) % len(motor_ids)]
            except ValueError:
                self.selected_motor_id = motor_ids[0]
        
        self.dashboard.state.selected_motor_id = self.selected_motor_id
        self.dashboard.add_event(f"Selected motor {self.selected_motor_id}")
    
    def _show_motor_details(self):
        """Show detailed information for selected motor"""
        if self.selected_motor_id is None:
            self.dashboard.add_event("No motor selected")
            return
        
        motor = self._get_selected_motor()
        if motor is None:
            self.dashboard.add_event(f"Motor {self.selected_motor_id} not found")
            return
        
        self.dashboard.state.show_detailed_view = not self.dashboard.state.show_detailed_view
        state = "shown" if self.dashboard.state.show_detailed_view else "hidden"
        self.dashboard.add_event(f"Motor {self.selected_motor_id} details {state}")
    
    def _toggle_motor_enable(self):
        """Toggle enable state of selected motor"""
        if self.selected_motor_id is None:
            self.dashboard.add_event("No motor selected")
            return
        
        motor = self._get_selected_motor()
        if motor is None:
            self.dashboard.add_event(f"Motor {self.selected_motor_id} not found")
            return
        
        # Toggle motor enable state
        motor.is_enabled = not motor.is_enabled
        state = "enabled" if motor.is_enabled else "disabled"
        self.dashboard.add_event(f"Motor {self.selected_motor_id} {state}")
    
    def _stop_motor(self):
        """Stop the selected motor"""
        if self.selected_motor_id is None:
            self.dashboard.add_event("No motor selected")
            return
        
        motor = self._get_selected_motor()
        if motor is None:
            self.dashboard.add_event(f"Motor {self.selected_motor_id} not found")
            return
        
        # Stop motor movement
        motor.current_rpm = 0
        motor.target_position_steps = motor.position_steps
        motor.motor_status_code = 0  # Stopped
        self.dashboard.add_event(f"Motor {self.selected_motor_id} stopped")
    
    def _zero_motor_position(self):
        """Zero the position of selected motor"""
        if self.selected_motor_id is None:
            self.dashboard.add_event("No motor selected")
            return
        
        motor = self._get_selected_motor()
        if motor is None:
            self.dashboard.add_event(f"Motor {self.selected_motor_id} not found")
            return
        
        # Zero the motor position
        motor.position_steps = 0
        motor.target_position_steps = 0
        self.dashboard.add_event(f"Motor {self.selected_motor_id} position zeroed")
    
    def _enter_command_mode(self):
        """Enter command input mode"""
        self.command_mode = True
        self.command_buffer = ""
        self.dashboard.add_event("Command mode activated (type 'exit' to leave)")
    
    def _increase_refresh_rate(self):
        """Increase dashboard refresh rate (make faster)"""
        current_rate = self.dashboard.state.refresh_rate_ms
        new_rate = max(50, current_rate - 50)  # Minimum 50ms
        self.dashboard.set_refresh_rate(new_rate)
    
    def _decrease_refresh_rate(self):
        """Decrease dashboard refresh rate (make slower)"""
        current_rate = self.dashboard.state.refresh_rate_ms
        new_rate = min(2000, current_rate + 50)  # Maximum 2000ms
        self.dashboard.set_refresh_rate(new_rate)
    
    def get_help_text(self) -> str:
        """Generate help text for display"""
        help_lines = ["Interactive Controls Help:", ""]
        
        categories = {"dashboard": "Dashboard Controls", "motor": "Motor Controls", "config": "Configuration Management", "advanced": "Advanced Commands"}
        
        for category, title in categories.items():
            help_lines.append(f"{title}:")
            for binding in self.key_bindings.values():
                if binding.category == category:
                    help_lines.append(f"  {binding.key}: {binding.description}")
            help_lines.append("")
        
        # Add command mode help
        help_lines.extend([
            "Command Mode Instructions:",
            "  inject <code> [data...] - Inject raw command (hex)",
            "  templates - List available command templates",
            "  scenario <name> - Run test scenario",
            "  stats - Show command injection statistics",
            "",
            "Injection Mode Instructions:",
            "  1. Press 'i' to enter injection mode",
            "  2. Enter command code in hex (e.g., F6)",
            "  3. Enter data bytes in hex (e.g., 01 00 64 00)",
            "  4. Command will be executed automatically",
            ""
        ])
        
        return "\n".join(help_lines)
    
    async def handle_input(self):
        """Main input handling loop"""
        while self.running:
            try:
                key = await self._get_key_async()
                if key is None:
                    await asyncio.sleep(0.01)  # Small delay to prevent busy waiting
                    continue
                
                # Handle command mode
                if self.command_mode:
                    if key == '\r':  # Enter
                        await self._process_command(self.command_buffer)
                        self.command_mode = False
                        self.command_buffer = ""
                    elif key == '\x1b':  # Escape
                        self.command_mode = False
                        self.command_buffer = ""
                        self.dashboard.add_event("Command mode cancelled")
                    elif key == '\x7f':  # Backspace
                        self.command_buffer = self.command_buffer[:-1]
                    elif key.isprintable():
                        self.command_buffer += key
                    continue
                
                # Handle regular key bindings
                if key in self.key_bindings:
                    try:
                        self.key_bindings[key].action()
                    except Exception as e:
                        self.dashboard.add_event(f"Error executing command: {e}")
                else:
                    # Log unknown key for debugging
                    if key.isprintable():
                        self.dashboard.add_event(f"Unknown key: '{key}'")
                    else:
                        self.dashboard.add_event(f"Unknown key code: {repr(key)}")
                
            except Exception as e:
                self.dashboard.add_event(f"Input error: {e}")
                await asyncio.sleep(0.1)
    
    async def _process_command(self, command: str):
        """Process a command entered in command mode"""
        command = command.strip().lower()
        
        if command == "exit":
            self.dashboard.add_event("Exited command mode")
            return
        
        # Simple command processor
        parts = command.split()
        if not parts:
            return
        
        cmd = parts[0]
        args = parts[1:] if len(parts) > 1 else []
        
        if cmd == "help":
            self.dashboard.add_event("Available commands: help, status, select, enable, disable, stop, zero, move")
        elif cmd == "status":
            motor_count = len(self.virtual_can_bus.simulated_motors)
            selected = self.selected_motor_id or "None"
            self.dashboard.add_event(f"Status: {motor_count} motors, selected: {selected}")
        elif cmd == "select" and args:
            try:
                motor_id = int(args[0])
                if motor_id in self.virtual_can_bus.simulated_motors:
                    self.selected_motor_id = motor_id
                    self.dashboard.state.selected_motor_id = motor_id
                    self.dashboard.add_event(f"Selected motor {motor_id}")
                else:
                    self.dashboard.add_event(f"Motor {motor_id} not found")
            except ValueError:
                self.dashboard.add_event("Invalid motor ID")
        elif cmd in ["enable", "disable", "stop", "zero"]:
            if self.selected_motor_id is None:
                self.dashboard.add_event("No motor selected")
            else:
                if cmd == "enable":
                    self._toggle_motor_enable()
                elif cmd == "disable":
                    motor = self._get_selected_motor()
                    if motor:
                        motor.is_enabled = False
                        self.dashboard.add_event(f"Motor {self.selected_motor_id} disabled")
                elif cmd == "stop":
                    self._stop_motor()
                elif cmd == "zero":
                    self._zero_motor_position()
        elif cmd == "param" and len(parts) >= 3:
            # Live parameter adjustment: param <name> <value>
            param_name = parts[1]
            param_value = parts[2]
            await self._adjust_parameter(param_name, param_value)
        elif cmd == "template" and len(parts) >= 3:
            # Apply motor template: template <motor_id> <template_name>
            try:
                motor_id = int(parts[1])
                template_name = parts[2]
                await self._apply_template(motor_id, template_name)
            except ValueError:
                self.dashboard.add_event("Invalid motor ID for template command")
        elif cmd == "profile" and len(parts) >= 2:
            # Load configuration profile: profile <name>
            profile_name = parts[1]
            await self._load_profile(profile_name)
        elif cmd == "save" and len(parts) >= 2:
            # Save configuration profile: save <name>
            profile_name = parts[1]
            await self._save_profile(profile_name)
        else:
            self.dashboard.add_event(f"Unknown command: {command}")
    
    def _show_profiles_menu(self):
        """Show configuration profiles menu"""
        if not self.config_manager:
            self.dashboard.add_event("Configuration manager not available")
            return
        
        profiles = self.config_manager.list_profiles()
        if not profiles:
            self.dashboard.add_event("No configuration profiles found")
            return
        
        menu_text = "Configuration Profiles:\n"
        for i, profile in enumerate(profiles, 1):
            menu_text += f"  {i}. {profile}\n"
        menu_text += "Type profile number to load, or press any key to cancel"
        
        self.dashboard.add_event(menu_text)
        # TODO: Implement profile selection interface
    
    def _live_parameter_menu(self):
        """Show live parameter adjustment menu"""
        if not self.live_config:
            self.dashboard.add_event("Live configuration not available")
            return
        
        adjustable = self.live_config.get_adjustable_parameters()
        menu_text = "Live Parameter Adjustment:\n"
        for param, info in adjustable.items():
            current = info.get('current', 'N/A')
            menu_text += f"  {param}: {info['description']} (current: {current})\n"
        menu_text += "Use command mode (c) to adjust: param <name> <value>"
        
        self.dashboard.add_event(menu_text)
    
    def _motor_templates_menu(self):
        """Show motor templates menu"""
        if not self.config_manager:
            self.dashboard.add_event("Configuration manager not available")
            return
        
        templates = self.config_manager.get_motor_templates()
        menu_text = "Motor Templates:\n"
        for name, template in templates.items():
            menu_text += f"  {name}: {template.motor_type} - {getattr(template, 'description', 'No description')}\n"
        menu_text += "Use command mode (c) to apply: template <motor_id> <template_name>"
        
        self.dashboard.add_event(menu_text)
    
    def _save_config_prompt(self):
        """Prompt to save current configuration"""
        if not self.config_manager:
            self.dashboard.add_event("Configuration manager not available")
            return
        
        # Save current configuration
        if self.config_manager.current_config:
            success = self.config_manager.save_config(self.config_manager.current_config)
            if success:
                self.dashboard.add_event("Configuration saved successfully")
            else:
                self.dashboard.add_event("Failed to save configuration")
        else:
            self.dashboard.add_event("No current configuration to save")
    
    async def _adjust_parameter(self, param_name: str, param_value: str):
        """Adjust a live parameter"""
        if not self.live_config:
            self.dashboard.add_event("Live configuration not available")
            return
        
        try:
            # Convert value to appropriate type
            if '.' in param_value:
                value = float(param_value)
            else:
                value = int(param_value)
        except ValueError:
            # Try as string
            value = param_value
        
        success = await self.live_config.update_parameter(param_name, value)
        if success:
            self.dashboard.add_event(f"Updated {param_name} = {value}")
        else:
            self.dashboard.add_event(f"Failed to update {param_name}")
    
    async def _apply_template(self, motor_id: int, template_name: str):
        """Apply a motor template"""
        if not self.config_manager:
            self.dashboard.add_event("Configuration manager not available")
            return
        
        success = self.config_manager.apply_motor_template(motor_id - 1, template_name)  # Convert to 0-based index
        if success:
            self.dashboard.add_event(f"Applied template '{template_name}' to motor {motor_id}")
        else:
            self.dashboard.add_event(f"Failed to apply template '{template_name}' to motor {motor_id}")
    
    async def _load_profile(self, profile_name: str):
        """Load a configuration profile"""
        if not self.config_manager:
            self.dashboard.add_event("Configuration manager not available")
            return
        
        config = self.config_manager.load_config(profile_name)
        if config:
            self.config_manager.current_config = config
            self.dashboard.add_event(f"Loaded profile '{profile_name}'")
        else:
            self.dashboard.add_event(f"Failed to load profile '{profile_name}'")
    
    async def _save_profile(self, profile_name: str):
        """Save current configuration as a profile"""
        if not self.config_manager:
            self.dashboard.add_event("Configuration manager not available")
            return
        
        if not self.config_manager.current_config:
            self.dashboard.add_event("No current configuration to save")
            return
        
        success = self.config_manager.save_config(self.config_manager.current_config, profile_name)
        if success:
            self.dashboard.add_event(f"Saved profile '{profile_name}'")
        else:
            self.dashboard.add_event(f"Failed to save profile '{profile_name}'")
    
    async def start(self):
        """Start the interactive controller"""
        self.running = True
        self._setup_terminal()
        
        # Initialize with first motor selected if available
        motor_ids = self._get_motor_ids()
        if motor_ids:
            self.selected_motor_id = motor_ids[0]
            self.dashboard.state.selected_motor_id = self.selected_motor_id
        
        self.dashboard.add_event("Interactive controls started (press 'h' for help)")
        
        try:
            await self.handle_input()
        finally:
            self._restore_terminal()
    
    def stop(self):
        """Stop the interactive controller"""
        self.running = False
        self._restore_terminal()