"""
Rich console dashboard for MKS servo simulator.

Provides a modern text-based interface showing real-time motor status,
communication, system information, and event logs.
"""

import asyncio
import time
from typing import Dict, List, Optional, TYPE_CHECKING, Any
from dataclasses import dataclass

from rich.console import Console
from rich.layout import Layout
from rich.panel import Panel
from rich.table import Table
from rich.text import Text
from rich.live import Live
from rich.progress import Progress, BarColumn, TextColumn
from rich.align import Align
from rich.columns import Columns

if TYPE_CHECKING:
    from ..virtual_can_bus import VirtualCANBus
    from ..motor_model import SimulatedMotor
    from .llm_debug_interface import LLMDebugInterface


@dataclass
class DashboardState:
    """Current state of the dashboard display"""
    start_time: float
    refresh_rate_ms: int
    paused: bool = False
    selected_motor_id: Optional[int] = None
    show_detailed_view: bool = False


class RichDashboard:
    """
    Rich console dashboard for real-time motor status display.
    
    Features:
    - Real-time motor status (position, speed, encoder values)
    - Communication panel (commands, response times)
    - System information (uptime, connections)
    - Scrolling event log
    - Interactive controls
    """
    
    def __init__(
        self,
        virtual_can_bus: "VirtualCANBus",
        debug_interface: Optional["LLMDebugInterface"] = None,
        refresh_rate_ms: int = 200,
        no_color: bool = False
    ):
        """
        Initialize the Rich dashboard.
        
        Args:
            virtual_can_bus: The virtual CAN bus instance
            debug_interface: Optional debug interface for additional data
            refresh_rate_ms: Dashboard refresh rate in milliseconds
            no_color: Disable color output for compatibility
        """
        self.virtual_can_bus = virtual_can_bus
        self.debug_interface = debug_interface
        self.console = Console(force_terminal=not no_color)
        self.state = DashboardState(
            start_time=time.time(),
            refresh_rate_ms=refresh_rate_ms
        )
        
        # Layout configuration
        self.layout = Layout()
        self._setup_layout()
        
        # Event log (circular buffer)
        self.event_log: List[str] = []
        self.max_log_entries = 50
        
        # Last update tracking
        self._last_command_times: Dict[int, float] = {}
        self._command_rates: Dict[int, float] = {}
        
    def _setup_layout(self):
        """Configure the dashboard layout structure"""
        self.layout.split_column(
            Layout(name="header", size=3),
            Layout(name="main", ratio=1),
            Layout(name="footer", size=3)
        )
        
        # Main area split into left and right panels
        self.layout["main"].split_row(
            Layout(name="left_panel", ratio=2),
            Layout(name="right_panel", ratio=1)
        )
        
        # Left panel: motor status and communication
        self.layout["left_panel"].split_column(
            Layout(name="motor_status", ratio=2),
            Layout(name="communication", ratio=1)
        )
        
        # Right panel: system info and logs
        self.layout["right_panel"].split_column(
            Layout(name="system_info", size=8),
            Layout(name="event_log", ratio=1)
        )
    
    def _create_header(self) -> Panel:
        """Create the dashboard header"""
        uptime = time.time() - self.state.start_time
        uptime_str = f"{int(uptime//3600):02d}:{int((uptime%3600)//60):02d}:{int(uptime%60):02d}"
        
        title_text = Text("MKS Servo Simulator Dashboard", style="bold blue")
        status_text = Text(f"Uptime: {uptime_str}", style="dim")
        
        if self.state.paused:
            status_text.append(" | PAUSED", style="bold red")
        
        header_content = Columns([
            Align.left(title_text),
            Align.right(status_text)
        ])
        
        return Panel(header_content, title="Status", border_style="blue")
    
    def _create_motor_status_panel(self) -> Panel:
        """Create the motor status panel showing all motors"""
        if not self.virtual_can_bus.simulated_motors:
            return Panel(
                Align.center(Text("No motors connected", style="dim")),
                title="Motor Status",
                border_style="yellow"
            )
        
        # Create table for motor overview
        table = Table(show_header=True, header_style="bold magenta")
        table.add_column("ID", style="cyan", width=4)
        table.add_column("Status", width=10)
        table.add_column("Position", width=12)
        table.add_column("Speed", width=10)
        table.add_column("Target", width=12)
        table.add_column("Enabled", width=8)
        
        for motor_id, motor in sorted(self.virtual_can_bus.simulated_motors.items()):
            # Status color coding
            status_style = "green" if motor.is_enabled else "red"
            status_text = self._get_motor_status_text(motor)
            
            # Position in degrees
            pos_degrees = (motor.position_steps / motor.steps_per_rev_encoder) * 360
            pos_text = f"{pos_degrees:.1f}°"
            
            # Speed in RPM
            speed_text = f"{motor.current_rpm:.1f} RPM"
            
            # Target position
            if motor.target_position_steps is not None:
                target_degrees = (motor.target_position_steps / motor.steps_per_rev_encoder) * 360
                target_text = f"{target_degrees:.1f}°"
            else:
                target_text = "None"
            
            # Enabled status
            enabled_text = "Yes" if motor.is_enabled else "No"
            
            table.add_row(
                str(motor_id),
                Text(status_text, style=status_style),
                pos_text,
                speed_text,
                target_text,
                Text(enabled_text, style=status_style)
            )
        
        return Panel(table, title="Motor Status", border_style="green")
    
    def _create_communication_panel(self) -> Panel:
        """Create the communication panel showing command stats"""
        table = Table(show_header=True, header_style="bold cyan")
        table.add_column("Motor ID", width=8)
        table.add_column("Last Command", width=12)
        table.add_column("Response Time", width=12)
        table.add_column("Cmd/sec", width=8)
        table.add_column("Errors", width=8)
        
        # Get communication stats from debug interface if available
        if self.debug_interface:
            for motor_id in sorted(self.virtual_can_bus.simulated_motors.keys()):
                motor_stats = self.debug_interface.get_motor_status(motor_id)
                if motor_stats:
                    # Calculate command rate
                    current_time = time.time()
                    if motor_id in self._last_command_times:
                        time_diff = current_time - self._last_command_times[motor_id]
                        if time_diff > 0:
                            self._command_rates[motor_id] = 1.0 / time_diff
                    
                    last_cmd = "N/A"
                    response_time = "N/A"
                    cmd_rate = self._command_rates.get(motor_id, 0.0)
                    
                    # Get recent command history
                    recent_commands = [cmd for cmd in self.debug_interface.command_history 
                                     if cmd.motor_id == motor_id]
                    if recent_commands:
                        latest_cmd = recent_commands[-1]
                        last_cmd = latest_cmd.command_name[:10]
                        response_time = f"{latest_cmd.response_time_ms:.1f}ms"
                        self._last_command_times[motor_id] = latest_cmd.timestamp
                    
                    table.add_row(
                        str(motor_id),
                        last_cmd,
                        response_time,
                        f"{cmd_rate:.1f}",
                        "0"  # Error count placeholder
                    )
        else:
            # Basic fallback without debug interface
            for motor_id in sorted(self.virtual_can_bus.simulated_motors.keys()):
                table.add_row(str(motor_id), "N/A", "N/A", "0.0", "0")
        
        return Panel(table, title="Communication", border_style="cyan")
    
    def _create_system_info_panel(self) -> Panel:
        """Create the system information panel"""
        info_table = Table(show_header=False, box=None, pad_edge=False)
        info_table.add_column("Key", style="bold")
        info_table.add_column("Value")
        
        # System information
        uptime = time.time() - self.state.start_time
        motor_count = len(self.virtual_can_bus.simulated_motors)
        client_count = len(self.virtual_can_bus.clients)
        
        info_table.add_row("Motors:", str(motor_count))
        info_table.add_row("Clients:", str(client_count))
        info_table.add_row("Refresh:", f"{self.state.refresh_rate_ms}ms")
        info_table.add_row("Uptime:", f"{uptime:.1f}s")
        
        if self.debug_interface:
            info_table.add_row("Debug API:", "Enabled")
            info_table.add_row("Commands:", str(len(self.debug_interface.command_history)))
        
        return Panel(info_table, title="System", border_style="magenta")
    
    def _create_event_log_panel(self) -> Panel:
        """Create the scrolling event log panel"""
        log_text = Text()
        
        # Show recent log entries
        recent_entries = self.event_log[-20:] if len(self.event_log) > 20 else self.event_log
        
        for entry in recent_entries:
            log_text.append(entry + "\n", style="dim")
        
        if not recent_entries:
            log_text.append("No events yet...", style="dim italic")
        
        return Panel(log_text, title="Event Log", border_style="white")
    
    def _create_footer(self) -> Panel:
        """Create the dashboard footer with controls"""
        controls = Text()
        controls.append("Controls: ", style="bold")
        controls.append("[SPACE] Pause/Resume  ", style="cyan")
        controls.append("[R] Restart  ", style="cyan")
        controls.append("[Q] Quit  ", style="cyan")
        controls.append("[H] Help", style="cyan")
        
        return Panel(Align.center(controls), border_style="blue")
    
    def _get_motor_status_text(self, motor: "SimulatedMotor") -> str:
        """Get human-readable status text for a motor"""
        if not motor.is_enabled:
            return "Disabled"
        
        # Map status codes to text
        status_map = {
            0: "Stopped",
            1: "Running",
            2: "Stopping",
            3: "Homing",
            4: "Error"
        }
        
        return status_map.get(motor.motor_status_code, f"Status {motor.motor_status_code}")
    
    def add_event(self, message: str):
        """Add an event to the log"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        self.event_log.append(log_entry)
        
        # Keep log size manageable
        if len(self.event_log) > self.max_log_entries:
            self.event_log.pop(0)
    
    def update_display(self):
        """Update all dashboard panels"""
        if self.state.paused:
            return
        
        # Update layout panels
        self.layout["header"].update(self._create_header())
        self.layout["motor_status"].update(self._create_motor_status_panel())
        self.layout["communication"].update(self._create_communication_panel())
        self.layout["system_info"].update(self._create_system_info_panel())
        self.layout["event_log"].update(self._create_event_log_panel())
        self.layout["footer"].update(self._create_footer())
    
    async def run(self):
        """Run the dashboard with live updates"""
        self.add_event("Dashboard started")
        
        with Live(self.layout, console=self.console, refresh_per_second=1000/self.state.refresh_rate_ms) as live:
            try:
                while True:
                    self.update_display()
                    await asyncio.sleep(self.state.refresh_rate_ms / 1000.0)
            except KeyboardInterrupt:
                self.add_event("Dashboard stopped by user")
                raise
            except Exception as e:
                self.add_event(f"Dashboard error: {e}")
                raise
    
    def toggle_pause(self):
        """Toggle pause state"""
        self.state.paused = not self.state.paused
        status = "paused" if self.state.paused else "resumed"
        self.add_event(f"Dashboard {status}")
    
    def set_refresh_rate(self, rate_ms: int):
        """Set the dashboard refresh rate"""
        self.state.refresh_rate_ms = max(50, min(2000, rate_ms))  # Clamp between 50ms and 2s
        self.add_event(f"Refresh rate set to {self.state.refresh_rate_ms}ms")