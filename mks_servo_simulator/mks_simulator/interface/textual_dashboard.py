"""
Textual-based dashboard for MKS servo simulator.
Enhanced version with auto-refresh and improved layout.
"""

import asyncio
import threading
import time
from typing import TYPE_CHECKING

from textual.app import App, ComposeResult
from textual.containers import Container, Horizontal, Vertical
from textual.widgets import Header, Footer, DataTable, Static, Label
from textual.timer import Timer

from mks_servo_can import constants as const

if TYPE_CHECKING:
    from ..virtual_can_bus import VirtualCANBus


class MotorStatusWidget(Static):
    """Widget displaying motor status in a table"""
    
    def __init__(self, virtual_can_bus=None):
        super().__init__()
        self.virtual_can_bus = virtual_can_bus
    
    def compose(self) -> ComposeResult:
        yield DataTable()
    
    def on_mount(self) -> None:
        """Initialize the motor status table"""
        table = self.query_one(DataTable)
        table.add_columns("ID", "Status", "Position", "Speed", "Target", "Enabled")
        self.refresh_data()
    
    def refresh_data(self) -> None:
        """Refresh table with real motor data"""
        try:
            table = self.query_one(DataTable)
            table.clear()
            
            if self.virtual_can_bus and hasattr(self.virtual_can_bus, 'simulated_motors') and self.virtual_can_bus.simulated_motors:
                # Real data from virtual CAN bus
                for motor_id, motor in sorted(self.virtual_can_bus.simulated_motors.items()):
                    try:
                        # Get motor status text
                        status_map = {
                            const.MOTOR_STATUS_QUERY_FAIL: "Error/Query Fail",
                            const.MOTOR_STATUS_STOPPED: "Stopped",
                            const.MOTOR_STATUS_SPEED_UP: "Accelerating",
                            const.MOTOR_STATUS_SPEED_DOWN: "Decelerating", # Could also be "Stopping"
                            const.MOTOR_STATUS_FULL_SPEED: "Running",      # Was "Full Speed"
                            const.MOTOR_STATUS_HOMING: "Homing",
                            const.MOTOR_STATUS_CALIBRATING: "Calibrating"
                        }
                        status = status_map.get(getattr(motor, 'motor_status_code', const.MOTOR_STATUS_QUERY_FAIL), "Unknown Status")
                        
                        # Position in degrees with safe defaults
                        position_steps = getattr(motor, 'position_steps', 0)
                        steps_per_rev = getattr(motor, 'steps_per_rev_encoder', 1)
                        pos_degrees = (position_steps / steps_per_rev) * 360
                        position = f"{pos_degrees:.1f}°"
                        
                        # Speed in RPM
                        current_rpm = getattr(motor, 'current_rpm', 0.0)
                        speed = f"{current_rpm:.1f} RPM"
                        
                        # Target position
                        target_position_steps = getattr(motor, 'target_position_steps', None)
                        if target_position_steps is not None:
                            target_degrees = (target_position_steps / steps_per_rev) * 360
                            target = f"{target_degrees:.1f}°"
                        else:
                            target = "None"
                        
                        # Enabled status
                        is_enabled = getattr(motor, 'is_enabled', False)
                        enabled = "Yes" if is_enabled else "No"
                        
                        table.add_row(str(motor_id), status, position, speed, target, enabled)
                    except Exception as e:
                        # Handle individual motor errors gracefully
                        table.add_row(str(motor_id), "Error", "N/A", "N/A", "N/A", "N/A")
            else:
                # Fallback to demo data if no bus or connection issues
                demo_motors = [
                    ("1", "Disabled", "0.0°", "0.0 RPM", "None", "No"),
                    ("2", "Disabled", "45.0°", "0.0 RPM", "90.0°", "No"),
                    ("3", "Disabled", "180.0°", "0.0 RPM", "None", "No"),
                ]
                for motor_data in demo_motors:
                    table.add_row(*motor_data)
        except Exception as e:
            # Ultimate fallback - show error in table
            try:
                table = self.query_one(DataTable)
                table.clear()
                table.add_row("Error", str(e)[:20], "N/A", "N/A", "N/A", "N/A")
            except:
                pass


class SystemInfoWidget(Static):
    """Widget displaying system information"""
    
    def __init__(self, virtual_can_bus=None):
        super().__init__()
        self.virtual_can_bus = virtual_can_bus
        self.start_time = None
    
    def compose(self) -> ComposeResult:
        yield Static("", id="system-info")
    
    def on_mount(self) -> None:
        """Initialize with current data"""
        import time
        self.start_time = time.time()
        self.refresh_data()
    
    def refresh_data(self) -> None:
        """Refresh system information"""
        try:
            import time
            
            if self.virtual_can_bus:
                try:
                    motor_count = len(self.virtual_can_bus.simulated_motors)
                    client_count = len(self.virtual_can_bus.clients)
                    # Check if any motors are enabled
                    enabled_motors = sum(1 for motor in self.virtual_can_bus.simulated_motors.values() 
                                       if getattr(motor, 'is_enabled', False))
                    connection_status = "Connected"
                except Exception as e:
                    motor_count = 0
                    client_count = 0
                    enabled_motors = 0
                    connection_status = f"Error: {str(e)[:20]}"
            else:
                motor_count = 0
                client_count = 0
                enabled_motors = 0
                connection_status = "No CAN Bus"
            
            uptime = time.time() - self.start_time if self.start_time else 0.0
            uptime_str = f"{int(uptime//3600):02d}:{int((uptime%3600)//60):02d}:{int(uptime%60):02d}"
            
            info_text = (f"Status: {connection_status}\n"
                        f"Motors: {motor_count} ({enabled_motors} enabled)\n"
                        f"Clients: {client_count}\n"
                        f"Uptime: {uptime_str}")
            
            self.update(info_text)
        except Exception as e:
            # Graceful fallback on any error
            try:
                self.update(f"System Info Error:\n{str(e)[:30]}")
            except:
                pass


class TextualDashboard(App):
    """
    Textual-based dashboard application.
    Phase 1: Static layout with fake data.
    """
    
    CSS = """
    Screen {
        layout: grid;
        grid-size: 2 2;
        grid-gutter: 1 2;
        padding: 1;
    }
    
    #motor-status {
        column-span: 2;
        height: 12;
    }
    
    #system-info {
        height: 8;
    }
    
    #status-panel {
        height: 8;
    }
    
    DataTable {
        scrollbar-size: 1 1;
        scrollbar-size-horizontal: 1;
        scrollbar-size-vertical: 1;
    }
    """
    
    BINDINGS = [
        ("q", "quit", "Quit"),
        ("r", "refresh", "Refresh"),
        ("p", "toggle_pause", "Pause/Resume"),
        ("escape", "quit", "Quit"),
    ]
    
    def __init__(self, virtual_can_bus: "VirtualCANBus" = None, enable_auto_refresh: bool = True, refresh_interval: float = 2.0):
        super().__init__()
        self.virtual_can_bus = virtual_can_bus
        self.enable_auto_refresh = enable_auto_refresh
        self.refresh_interval = refresh_interval
        self.refresh_timer: Timer = None
        self.start_time = time.time()
        self.is_paused = False
    
    def compose(self) -> ComposeResult:
        """Create the dashboard layout"""
        yield Header(show_clock=True)
        with Horizontal():
            yield MotorStatusWidget(self.virtual_can_bus)
            yield SystemInfoWidget(self.virtual_can_bus)
        yield Footer()
    
    def on_mount(self) -> None:
        """Called when the app starts"""
        if self.enable_auto_refresh:
            self.start_auto_refresh()
    
    def start_auto_refresh(self) -> None:
        """Start the auto-refresh timer"""
        if self.refresh_timer:
            self.refresh_timer.stop()
        self.refresh_timer = self.set_interval(self.refresh_interval, self.action_refresh)
    
    def stop_auto_refresh(self) -> None:
        """Stop the auto-refresh timer"""
        if self.refresh_timer:
            self.refresh_timer.stop()
            self.refresh_timer = None
    
    def action_refresh(self) -> None:
        """Manual refresh action - updates all widgets with real data"""
        try:
            # Refresh motor status
            motor_widget = self.query_one(MotorStatusWidget)
            motor_widget.refresh_data()
            
            # Refresh system info  
            system_widget = self.query_one(SystemInfoWidget)
            system_widget.refresh_data()
            
            # Update status panel
            self.update_status_panel()
            
        except Exception as e:
            self.notify(f"Refresh error: {e}", severity="error")
    
    def action_toggle_pause(self) -> None:
        """Toggle pause/resume of auto-refresh"""
        self.is_paused = not self.is_paused
        if self.is_paused:
            self.stop_auto_refresh()
            self.notify("Auto-refresh paused")
        else:
            self.start_auto_refresh()
            self.notify("Auto-refresh resumed")
        self.update_status_panel()
    
    def update_status_panel(self) -> None:
        """Update the status panel with current information"""
        try:
            status_widget = self.query_one(Static)
            uptime = time.time() - self.start_time
            status_text = f"Uptime: {int(uptime//60)}:{int(uptime%60):02d}\n"
            if self.is_paused:
                status_text += "Auto-refresh: PAUSED\n"
            else:
                status_text += f"Auto-refresh: {self.refresh_interval}s\n"
            status_text += "Press 'q' to quit, 'r' to refresh, 'p' to pause"
            status_widget.update(status_text)
        except Exception:
            pass  # Graceful fallback
    
    def action_quit(self) -> None:
        """Quit the application"""
        self.stop_auto_refresh()
        self.exit()


def run_textual_dashboard(virtual_can_bus: "VirtualCANBus" = None, enable_auto_refresh: bool = True, refresh_interval: float = 2.0):
    """Run the textual dashboard with enhanced features"""
    app = TextualDashboard(virtual_can_bus, enable_auto_refresh=enable_auto_refresh, refresh_interval=refresh_interval)
    app.run()


if __name__ == "__main__":
    # Standalone test
    app = TextualDashboard()
    app.run()