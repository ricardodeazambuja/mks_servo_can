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
        self.performance_monitor = getattr(virtual_can_bus, 'performance_monitor', None)
        # Simple console - let Rich auto-detect (works fine without Live)
        self.console = Console(force_terminal=not no_color)
        self.state = DashboardState(
            start_time=time.time(),
            refresh_rate_ms=refresh_rate_ms
        )
        
        # No layout needed - direct printing approach
        
        # Event log (circular buffer)
        self.event_log: List[str] = []
        self.max_log_entries = 50
        
        # Last update tracking
        self._last_command_times: Dict[int, float] = {}
        self._command_rates: Dict[int, float] = {}
        
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
        
        # Show detailed view if requested
        if self.state.show_detailed_view and self.state.selected_motor_id:
            return self._create_detailed_motor_view()
        
        # Create table for motor overview with compact widths
        table = Table(show_header=True, header_style="bold magenta", box=None)
        table.add_column("ID", style="cyan", width=3)
        table.add_column("Status", width=8)
        table.add_column("Position", width=9)
        table.add_column("Speed", width=8)
        table.add_column("Target", width=9)
        table.add_column("Enabled", width=6)
        
        for motor_id, motor in sorted(self.virtual_can_bus.simulated_motors.items()):
            # Highlight selected motor
            row_style = "bold" if motor_id == self.state.selected_motor_id else None
            
            # Status color coding
            status_style = "green" if motor.is_enabled else "red"
            if row_style:
                status_style = f"{status_style} {row_style}"
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
            
            # Add selection indicator
            id_text = f"► {motor_id}" if motor_id == self.state.selected_motor_id else str(motor_id)
            
            table.add_row(
                Text(id_text, style=f"cyan {row_style or ''}".strip()),
                Text(status_text, style=status_style),
                Text(pos_text, style=row_style),
                Text(speed_text, style=row_style),
                Text(target_text, style=row_style),
                Text(enabled_text, style=f"{status_style}")
            )
        
        title = "Motor Status"
        if self.state.selected_motor_id:
            title += f" (Selected: {self.state.selected_motor_id})"
        
        return Panel(table, title=title, border_style="green")
    
    def _create_communication_panel(self) -> Panel:
        """Create the communication panel showing command stats"""
        table = Table(show_header=True, header_style="bold cyan", box=None)
        table.add_column("Motor ID", width=6)
        table.add_column("Last Command", width=10)
        table.add_column("Response Time", width=10)
        table.add_column("Cmd/sec", width=6)
        table.add_column("Errors", width=6)
        
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
        # Show help if requested
        if hasattr(self, 'interactive_controller') and self.interactive_controller and hasattr(self.interactive_controller, 'show_help') and self.interactive_controller.show_help:
            help_text = Text(self.interactive_controller.get_help_text(), style="dim")
            return Panel(help_text, title="Help (press 'h' to hide)", border_style="yellow")
        
        # Show injection mode if active
        if hasattr(self, 'interactive_controller') and self.interactive_controller and hasattr(self.interactive_controller, 'injection_mode') and self.interactive_controller.injection_mode:
            prompt_text = Text()
            controller = self.interactive_controller
            
            if controller.injection_step == 0:
                prompt_text.append("Command Code (hex): ", style="bold yellow")
                prompt_text.append(controller.command_buffer, style="white")
                prompt_text.append("█", style="yellow")
            elif controller.injection_step == 1:
                prompt_text.append("Data Bytes (hex, space-separated): ", style="bold yellow")
                prompt_text.append(controller.command_buffer, style="white")
                prompt_text.append("█", style="yellow")
            
            return Panel(prompt_text, title=f"Command Injection - Motor {controller.injection_data['motor_id']} (ESC to cancel)", border_style="yellow")
        
        # Show command mode if active
        if hasattr(self, 'interactive_controller') and self.interactive_controller and hasattr(self.interactive_controller, 'command_mode') and self.interactive_controller.command_mode:
            prompt_text = Text()
            prompt_text.append("Command: ", style="bold yellow")
            prompt_text.append(self.interactive_controller.command_buffer, style="white")
            prompt_text.append("█", style="yellow")  # Cursor
            return Panel(prompt_text, title="Command Mode (ESC to cancel)", border_style="yellow")
        
        # Regular controls
        controls = Text()
        controls.append("Controls: ", style="bold")
        controls.append("[SPACE] Pause  ", style="cyan")
        controls.append("[↑↓] Select  ", style="cyan")
        controls.append("[E] Enable  ", style="cyan")
        controls.append("[I] Inject  ", style="cyan")
        controls.append("[T] Templates  ", style="cyan")
        controls.append("[C] Command  ", style="cyan")
        controls.append("[H] Help  ", style="cyan")
        controls.append("[Q] Quit", style="cyan")
        
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
    
    def _create_performance_panel(self) -> Panel:
        """Create the performance monitoring panel"""
        if not self.performance_monitor:
            return Panel(
                Align.center(Text("Performance monitoring disabled", style="dim")),
                title="Performance",
                border_style="yellow"
            )
        
        current_metrics = self.performance_monitor.get_current_metrics()
        if not current_metrics:
            return Panel(
                Align.center(Text("No performance data yet...", style="dim")),
                title="Performance",
                border_style="cyan"
            )
        
        # Create performance metrics table
        perf_table = Table(show_header=False, box=None, pad_edge=False)
        perf_table.add_column("Metric", style="bold cyan", width=12)
        perf_table.add_column("Value", style="white")
        
        # Latency metrics
        perf_table.add_row("Avg Latency:", f"{current_metrics.avg_response_time:.1f}ms")
        perf_table.add_row("P95 Latency:", f"{current_metrics.p95_response_time:.1f}ms")
        perf_table.add_row("P99 Latency:", f"{current_metrics.p99_response_time:.1f}ms")
        perf_table.add_row("", "")  # Separator
        
        # Throughput metrics
        perf_table.add_row("Commands/sec:", f"{current_metrics.commands_per_second:.1f}")
        perf_table.add_row("Total Cmds:", str(current_metrics.total_commands))
        perf_table.add_row("Error Rate:", f"{current_metrics.error_rate*100:.1f}%")
        perf_table.add_row("", "")  # Separator
        
        # Memory metrics
        perf_table.add_row("Memory:", f"{current_metrics.memory_usage_mb:.1f}MB")
        perf_table.add_row("Memory %:", f"{current_metrics.memory_percent:.1f}%")
        perf_table.add_row("", "")  # Separator
        
        # Connection metrics
        perf_table.add_row("Connections:", str(current_metrics.active_connections))
        perf_table.add_row("Timeouts:", str(current_metrics.connection_timeouts))
        
        # Check for alerts
        alerts = self.performance_monitor.alerts
        border_style = "red" if alerts else "cyan"
        title = "Performance" + (f" ({len(alerts)} alerts)" if alerts else "")
        
        return Panel(perf_table, title=title, border_style=border_style)
    
    def add_event(self, message: str):
        """Add an event to the log"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        self.event_log.append(log_entry)
        
        # Keep log size manageable
        if len(self.event_log) > self.max_log_entries:
            self.event_log.pop(0)
    
    
    async def run(self):
        """Run the dashboard with simple ANSI positioning"""
        self.add_event("Dashboard started")
        
        # Hide cursor and clear screen once
        print("\033[?25l", end="")  # Hide cursor
        print("\033[2J", end="")    # Clear screen
        
        try:
            while True:
                if not self.state.paused:
                    # Move cursor to top-left
                    print("\033[H", end="")
                    
                    # Render each panel with Rich but print normally
                    with self.console.capture() as capture:
                        self.console.print(self._create_header())
                    print(capture.get(), end="")
                    
                    with self.console.capture() as capture:
                        from rich.columns import Columns
                        row1 = Columns([
                            self._create_motor_status_panel(), 
                            self._create_system_info_panel()
                        ], equal=False, expand=True)
                        self.console.print(row1)
                    print(capture.get(), end="")
                    
                    with self.console.capture() as capture:
                        row2 = Columns([
                            self._create_communication_panel(),
                            self._create_performance_panel()
                        ], equal=False, expand=True)
                        self.console.print(row2)
                    print(capture.get(), end="")
                    
                    with self.console.capture() as capture:
                        self.console.print(self._create_event_log_panel())
                    print(capture.get(), end="")
                    
                    with self.console.capture() as capture:
                        self.console.print(self._create_footer())
                    print(capture.get(), end="")
                
                await asyncio.sleep(self.state.refresh_rate_ms / 1000.0)
        except KeyboardInterrupt:
            self.add_event("Dashboard stopped by user")
            raise
        except Exception as e:
            self.add_event(f"Dashboard error: {e}")
            raise
        finally:
            # Restore cursor
            print("\033[?25h", end="")
    
    def toggle_pause(self):
        """Toggle pause state"""
        self.state.paused = not self.state.paused
        status = "paused" if self.state.paused else "resumed"
        self.add_event(f"Dashboard {status}")
    
    def set_refresh_rate(self, rate_ms: int):
        """Set the dashboard refresh rate"""
        self.state.refresh_rate_ms = max(50, min(2000, rate_ms))  # Clamp between 50ms and 2s
        self.add_event(f"Refresh rate set to {self.state.refresh_rate_ms}ms")
    
    def _create_detailed_motor_view(self) -> Panel:
        """Create detailed view for selected motor"""
        if not self.state.selected_motor_id:
            return Panel(Text("No motor selected", style="dim"), title="Motor Details", border_style="red")
        
        motor = self.virtual_can_bus.simulated_motors.get(self.state.selected_motor_id)
        if not motor:
            return Panel(Text("Motor not found", style="dim"), title="Motor Details", border_style="red")
        
        # Create detailed information table
        detail_table = Table(show_header=False, box=None, pad_edge=False)
        detail_table.add_column("Property", style="bold cyan", width=20)
        detail_table.add_column("Value", style="white")
        
        # Motor identification
        detail_table.add_row("Motor ID:", str(motor.can_id))
        detail_table.add_row("Motor Type:", motor.motor_type)
        detail_table.add_row("Enabled:", "Yes" if motor.is_enabled else "No")
        detail_table.add_row("Status Code:", f"{motor.motor_status_code} ({self._get_motor_status_text(motor)})")
        detail_table.add_row("", "")  # Separator
        
        # Position information
        pos_degrees = (motor.position_steps / motor.steps_per_rev_encoder) * 360
        detail_table.add_row("Position (steps):", str(motor.position_steps))
        detail_table.add_row("Position (degrees):", f"{pos_degrees:.2f}°")
        detail_table.add_row("Position (revolutions):", f"{pos_degrees/360:.2f}")
        
        if motor.target_position_steps is not None:
            target_degrees = (motor.target_position_steps / motor.steps_per_rev_encoder) * 360
            detail_table.add_row("Target (steps):", str(motor.target_position_steps))
            detail_table.add_row("Target (degrees):", f"{target_degrees:.2f}°")
            detail_table.add_row("Remaining (steps):", str(abs(motor.target_position_steps - motor.position_steps)))
        
        detail_table.add_row("", "")  # Separator
        
        # Speed and movement
        detail_table.add_row("Current Speed (RPM):", f"{motor.current_rpm:.2f}")
        detail_table.add_row("Max Speed (RPM):", f"{motor.max_rpm}")
        detail_table.add_row("Steps per Rev:", str(motor.steps_per_rev_encoder))
        detail_table.add_row("", "")  # Separator
        
        # Configuration
        detail_table.add_row("Acceleration:", f"{motor.acceleration_rpm_per_s} RPM/s")
        detail_table.add_row("Deceleration:", f"{motor.deceleration_rpm_per_s} RPM/s")
        detail_table.add_row("Position Limits:", f"{motor.min_position_steps} to {motor.max_position_steps}")
        
        return Panel(detail_table, title=f"Motor {self.state.selected_motor_id} Details (press Enter to close)", border_style="magenta")
    
    def set_interactive_controller(self, controller):
        """Set the interactive controller reference"""
        self.interactive_controller = controller