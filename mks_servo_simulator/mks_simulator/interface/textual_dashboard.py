"""
Textual-based dashboard for MKS servo simulator.

Legacy. The supported human-facing surface is the browser dashboard served at
`/dashboard` under `--debug-api`; the machine-facing one is `--json-output`.
This is kept for terminal-only use and renders `MotorSnapshot` like every other
surface, so it cannot drift away from what the motors are actually doing.
"""

import itertools
import time
from typing import TYPE_CHECKING, List, Optional, Tuple

from rich.markup import escape
from rich.text import Text
from textual.app import App, ComposeResult
from textual.timer import Timer
from textual.widgets import DataTable, Footer, Header, Static

if TYPE_CHECKING:
    from ..motor_model import MotorSnapshot, SimulatedMotor  # Added import
    from ..virtual_can_bus import VirtualCANBus
    from .llm_debug_interface import LLMDebugInterface  # Add this


def motor_row(snapshot: "MotorSnapshot") -> Tuple[str, ...]:
    """
    Renders one motor's snapshot as a table row.

    Kept out of the widget so it can be tested against a real motor without
    standing up a terminal application.

    Args:
        snapshot: The motor's state, from `SimulatedMotor.status_snapshot()`.

    Returns:
        The row's cells, in the order the table declares its columns.
    """
    target = (
        f"{snapshot.target_position_degrees:.1f} deg"
        if snapshot.target_position_degrees is not None
        else "None"
    )
    return (
        str(snapshot.can_id),
        snapshot.status_text,
        f"{snapshot.position_degrees:.1f} deg",
        f"{snapshot.current_rpm:.1f} RPM",
        target,
        f"{snapshot.speed_deg_per_s:.1f} deg/s",
        snapshot.work_mode_name,
        str(snapshot.microsteps),
        "Yes" if snapshot.enabled else "No",
    )


def motor_details(snapshot: "MotorSnapshot") -> List[str]:
    """
    Renders one motor's snapshot as the detail pane's lines.

    Args:
        snapshot: The motor's state, from `SimulatedMotor.status_snapshot()`.

    Returns:
        The lines to display, with Rich markup.
    """
    position_error = (
        f"{snapshot.position_error_steps} steps"
        if snapshot.position_error_steps is not None
        else "n/a"
    )
    target_steps = (
        snapshot.target_position_steps
        if snapshot.target_position_steps is not None
        else "None"
    )
    return [
        f"[bold]Motor ID {snapshot.can_id}[/bold] ({snapshot.motor_type})",
        f"  Enabled: {'Yes' if snapshot.enabled else 'No'}",
        f"  Work Mode: {snapshot.work_mode_name}",
        f"  Microsteps: {snapshot.microsteps}",
        f"  Position: {snapshot.position_steps:.1f} steps "
        f"({snapshot.position_degrees:.1f} deg)",
        f"  Target Pos: {target_steps} steps",
        f"  Position error: {position_error}",
        f"  Speed (RPM): {snapshot.current_rpm:.1f}",
        f"  Speed: {snapshot.speed_deg_per_s:.1f} deg/s",
        f"  Target RPM: {snapshot.target_rpm:.1f}",
        f"  Calibrated: {'Yes' if snapshot.calibrated else 'No'}",
        f"  Homed: {'Yes' if snapshot.homed else 'No'}",
        f"  Responses enabled: {'Yes' if snapshot.responses_enabled else 'No'}",
    ]


class MotorStatusWidget(Static):
    """Widget displaying motor status in a table"""

    def __init__(self, virtual_can_bus=None, id: Optional[str] = None):
        super().__init__(id=id)
        self.virtual_can_bus = virtual_can_bus

    def compose(self) -> ComposeResult:
        yield DataTable()

    def on_mount(self) -> None:
        """Initialize the motor status table"""
        table = self.query_one(DataTable)
        table.add_columns("ID", "Status", "Position (units)", "Speed", "Target (units)", "Velocity (u/s)", "WorkMode", "MStep", "Enabled")

        # --- BEGINNING OF CHANGE ---
        table.cursor_type = "none"  # Disable cursor navigation for the table
        # --- END OF CHANGE ---

        self.refresh_data()

    def refresh_data(self) -> None:
        """Refresh table with real motor data"""
        try:
            table = self.query_one(DataTable)
            table.clear()

            if self.virtual_can_bus and hasattr(self.virtual_can_bus, 'simulated_motors') and self.virtual_can_bus.simulated_motors:
                # Every reporting surface renders MotorSnapshot and nothing else.
                # Reading motor attributes directly here is what let three other
                # surfaces drift into showing zeros for a moving motor.
                for motor_id, motor in sorted(self.virtual_can_bus.simulated_motors.items()):
                    try:
                        table.add_row(*motor_row(motor.status_snapshot()))
                    except Exception:
                        # Handle individual motor errors gracefully
                        table.add_row(str(motor_id), "Error", "N/A", "N/A", "N/A", "N/A", "N/A", "N/A", "N/A")
            else:
                # Fallback to demo data if no bus or connection issues
                demo_motors = [
                    ("1", "Disabled", "0.0 deg", "0.0 RPM", "None", "0.0 deg/s", "N/A", "N/A", "No"),
                    ("2", "Disabled", "45.0 deg", "0.0 RPM", "90.0 deg", "0.0 deg/s", "N/A", "N/A", "No"),
                    ("3", "Disabled", "180.0 deg", "0.0 RPM", "None", "0.0 deg/s", "N/A", "N/A", "No"),
                ]
                for motor_data in demo_motors:
                    table.add_row(*motor_data)
        except Exception as e:
            # Ultimate fallback - show error in table
            try:
                table = self.query_one(DataTable)
                table.clear()
                table.add_row("Error", str(e)[:20], "N/A", "N/A", "N/A", "N/A", "N/A", "N/A", "N/A")
            except Exception:
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
                    enabled_motors = sum(
                        1
                        for motor in self.virtual_can_bus.simulated_motors.values()
                        if motor.status_snapshot().enabled
                    )
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
            except Exception:
                pass


class DetailedMotorViewWidget(Static):
    """Widget to display detailed information for a selected motor."""

    def __init__(self, id: Optional[str] = None): # Removed *args, **kwargs for specific signature
        super().__init__(id=id)
        self.selected_motor: Optional[SimulatedMotor] = None

    def on_mount(self) -> None:
        """Called when the widget is mounted."""
        self.update_details(None)

    def update_details(self, motor: Optional['SimulatedMotor']) -> None:
        """Update the displayed motor details."""
        self.selected_motor = motor
        if motor:
            self.update("\n".join(motor_details(motor.status_snapshot())))
        else:
            self.update("Select a motor (Up/Down)")


class CommandLogWidget(Static):
    """Widget to display a log of the most recent commands."""

    def __init__(self, debug_interface: Optional['LLMDebugInterface'] = None, id: Optional[str] = None): # Removed *args, **kwargs
        super().__init__(id=id)
        self.debug_interface = debug_interface
        self.max_log_entries = 10

    def on_mount(self) -> None:
        """Called when the widget is mounted."""
        self.update_log()

    def update_log(self) -> None:
        """Update the command log display."""
        if not self.debug_interface or not self.debug_interface.command_history:
            self.update("No commands yet.")
            return

        history = self.debug_interface.command_history
        # Get the last N entries, most recent first for display usually, but log is chronological
        # --- MODIFIED LINE BELOW ---
        log_entries_to_display = list(itertools.islice(history, max(0, len(history) - self.max_log_entries), len(history)))

        formatted_log_lines = []
        for record in log_entries_to_display:
            try:
                ts_val = getattr(record, 'timestamp', None)
                ts_str = f"[{ts_val:.2f}]" if ts_val is not None else "[N/A]"

                motor_id_val = getattr(record, 'motor_id', None)
                motor_id_display_str = f"M{motor_id_val}" if motor_id_val is not None else "SYS"

                cmd_code_val = getattr(record, 'command_code', None)
                cmd_name_val = getattr(record, 'command_name', None)

                if cmd_name_val:
                    cmd_display_str = str(cmd_name_val)
                elif cmd_code_val is not None:
                    cmd_display_str = f"0x{cmd_code_val:02X}"
                else:
                    cmd_display_str = "CMD_N/A"

                success_val = getattr(record, 'success', None)
                success_display_str = "Ok" if success_val is True else ("Fail" if success_val is False else "N/A")

                resp_time_ms_val = getattr(record, 'response_time_ms', None)
                resp_time_display_str = f"{resp_time_ms_val:.1f}ms" if resp_time_ms_val is not None else "-"

                # Ensure all parts are strings before joining, to be extra safe
                line_parts = [
                    str(ts_str), " ",
                    str(motor_id_display_str), ": ",
                    str(cmd_display_str), " - ",
                    str(success_display_str), " (",
                    str(resp_time_display_str), ")"
                ]
                line = "".join(line_parts)

                max_line_len = 80 # Assuming this is defined or reasonable
                formatted_log_lines.append(line[:max_line_len] + "..." if len(line) > max_line_len else line)

            except TypeError as te:
                # Log the specific TypeError and the problematic record's attributes
                problematic_record_details = f"Problematic record: timestamp={getattr(record, 'timestamp', 'ErrorAccessing')}, "
                problematic_record_details += f"motor_id={getattr(record, 'motor_id', 'ErrorAccessing')}, "
                problematic_record_details += f"cmd_code={getattr(record, 'command_code', 'ErrorAccessing')}, "
                problematic_record_details += f"cmd_name={getattr(record, 'command_name', 'ErrorAccessing')}, "
                problematic_record_details += f"success={getattr(record, 'success', 'ErrorAccessing')}, "
                problematic_record_details += f"resp_time_ms={getattr(record, 'response_time_ms', 'ErrorAccessing')}"

                # At the beginning of the 'except TypeError as te:' block, ensure 'escape' is available
                # from rich.markup import escape # This import should be at the top of the file.

                escaped_te = escape(str(te))
                escaped_details = escape(problematic_record_details)
                error_line_for_ui = f"[CmdLog TypeError: {escaped_te}. Details: {escaped_details}]"
                formatted_log_lines.append(error_line_for_ui)

                # self.app.log.error(f"CommandLogWidget TypeError: {te} on record: {record!r}") # Original logging line, can be kept or removed.

            except Exception as e_format:
                # Catch other general exceptions during formatting
                formatted_log_lines.append(f"[Error formatting record: {type(e_format).__name__} - {e_format}. Record: {record!r}]")

        # --- BEGINNING OF CHANGE ---
        # --- BEGINNING OF CHANGE ---
        # The list comprehension [escape(line) for line in formatted_log_lines] should be removed.
        # formatted_log_lines should be the list of raw strings as they were before escaping.

        plain_log_content = "\n".join(formatted_log_lines)
        # Ensure Text is imported: from rich.text import Text (should be at top of file)
        text_object = Text(plain_log_content)
        self.update(text_object)
        # --- END OF CHANGE ---


class TextualDashboard(App):
    """
    Textual-based dashboard application.
    Phase 1: Static layout with fake data.
    """

    CSS = """
    Screen {
        layout: vertical;
        overflow-y: auto; /* Allow screen to scroll if content is too tall */
        padding: 1;
    }
    
    #motor-status-widget { /* Ensure ID matches compose */
        height: 12;
        margin-bottom: 1;
    }

    #detailed-motor-view {
        height: 12; /* Adjusted height */
        border: round white;
        padding: 1;
        margin-top: 1;
    }

    #command-log-widget {
        height: 10;
        border: round white;
        padding: 1;
        overflow-y: auto; /* Make it scrollable */
        margin-top: 1;
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
        ("up", "select_previous_motor", "Prev Mtr"),
        ("down", "select_next_motor", "Next Mtr"),
    ]

    def __init__(self, virtual_can_bus: "VirtualCANBus" = None, enable_auto_refresh: bool = True, refresh_interval: float = 2.0):
        super().__init__()

        # --- BEGINNING OF CHANGE ---
        self.log_file = "dashboard.log"  # Directs Textual's internal logging to this file
        self.log_verbosity = 2           # Set verbosity (1=normal, 2=verbose including debug)
        # --- END OF CHANGE ---

        self.virtual_can_bus = virtual_can_bus
        self.debug_interface = virtual_can_bus.debug_interface if virtual_can_bus else None
        self.enable_auto_refresh = enable_auto_refresh
        self.refresh_interval = refresh_interval
        self.refresh_timer: Timer = None # Timer should be Textual's Timer, not just Optional[Timer]
        self.start_time = time.time()
        self.is_paused = False
        self.selected_motor_id: Optional[int] = None

    def compose(self) -> ComposeResult:
        """Create the dashboard layout"""
        yield Header(show_clock=True)
        yield MotorStatusWidget(self.virtual_can_bus, id="motor-status-widget")
        yield DetailedMotorViewWidget(id="detailed-motor-view")
        yield CommandLogWidget(self.debug_interface, id="command-log-widget") # Add this
        yield Footer()

    def on_mount(self) -> None:
        """Called when the app starts"""
        # Only set default if no motor is selected yet
        if self.selected_motor_id is None:
            if self.virtual_can_bus and self.virtual_can_bus.simulated_motors:
                motor_ids = sorted(self.virtual_can_bus.simulated_motors.keys())
                if motor_ids:
                    self.selected_motor_id = motor_ids[0]
                    # self.notify(f"Default motor selected: ID {self.selected_motor_id}") # Optional for debugging

        if self.enable_auto_refresh:
            self.start_auto_refresh()
        self.action_refresh() # Initial refresh
        self.screen.focus() # Added this line

    def action_select_previous_motor(self) -> None:
        """Selects the previous motor in the list."""
        if not self.virtual_can_bus or not self.virtual_can_bus.simulated_motors:
            self.notify("No motors available.", severity="warning")
            return

        motor_ids = sorted(self.virtual_can_bus.simulated_motors.keys())
        if not motor_ids:
            self.notify("No motors available.", severity="warning")
            return

        if self.selected_motor_id is None:
            self.selected_motor_id = motor_ids[-1]
        else:
            try:
                current_index = motor_ids.index(self.selected_motor_id)
                self.selected_motor_id = motor_ids[current_index - 1] # Wraps around due to negative index
            except ValueError:
                self.selected_motor_id = motor_ids[-1]

        selected_motor = self.virtual_can_bus.simulated_motors.get(self.selected_motor_id)
        self.query_one(DetailedMotorViewWidget).update_details(selected_motor)
        self.notify(f"Selected Motor ID: {self.selected_motor_id}")

    def action_select_next_motor(self) -> None:
        """Selects the next motor in the list."""
        if not self.virtual_can_bus or not self.virtual_can_bus.simulated_motors:
            self.notify("No motors available.", severity="warning")
            return

        motor_ids = sorted(self.virtual_can_bus.simulated_motors.keys())
        if not motor_ids:
            self.notify("No motors available.", severity="warning")
            return

        if self.selected_motor_id is None:
            self.selected_motor_id = motor_ids[0]
        else:
            try:
                current_index = motor_ids.index(self.selected_motor_id)
                if current_index + 1 < len(motor_ids):
                    self.selected_motor_id = motor_ids[current_index + 1]
                else:
                    self.selected_motor_id = motor_ids[0] # Wrap around
            except ValueError:
                self.selected_motor_id = motor_ids[0]

        selected_motor = self.virtual_can_bus.simulated_motors.get(self.selected_motor_id)
        self.query_one(DetailedMotorViewWidget).update_details(selected_motor)
        self.notify(f"Selected Motor ID: {self.selected_motor_id}")

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
        # self.app.log.info(f"[ACTION_REFRESH] Start. selected_motor_id: {self.selected_motor_id}") # Original logging

        # Try to refresh MotorStatusWidget
        try:
            motor_status_widget = self.query_one(MotorStatusWidget)
            motor_status_widget.refresh_data()
        except Exception as e:
            self.notify(f'Refresh error (MotorStatus): {type(e).__name__}', severity='error', timeout=10)
            # self.app.log.error(f"[ACTION_REFRESH] Error refreshing MotorStatusWidget: {e}") # Original logging


        # self.app.log.info(f"[ACTION_REFRESH] Before DetailedMotorViewWidget update. selected_motor_id: {self.selected_motor_id}") # Original logging
        # Try to refresh DetailedMotorViewWidget
        try:
            detailed_view_widget = self.query_one(DetailedMotorViewWidget)
            selected_motor_object = None
            if self.virtual_can_bus and self.selected_motor_id is not None:
                selected_motor_object = self.virtual_can_bus.simulated_motors.get(self.selected_motor_id)
            detailed_view_widget.update_details(selected_motor_object)
        except Exception as e:
            self.notify(f'Refresh error (DetailView): {type(e).__name__}', severity='error', timeout=10)
            # self.app.log.error(f"[ACTION_REFRESH] Error refreshing DetailedMotorViewWidget: {e}") # Original logging

        # self.app.log.info(f"[ACTION_REFRESH] After DetailedMotorViewWidget update. selected_motor_id: {self.selected_motor_id}") # Original logging

        # Try to refresh CommandLogWidget
        try:
            command_log_widget = self.query_one(CommandLogWidget)
            command_log_widget.update_log()
        except Exception as e:
            self.notify(f'Refresh error (CmdLog): {type(e).__name__}', severity='error', timeout=10)
            # self.app.log.error(f"[ACTION_REFRESH] Error refreshing CommandLogWidget: {e}") # Original logging

        # self.app.log.info(f"[ACTION_REFRESH] End. selected_motor_id: {self.selected_motor_id}") # Original logging

    def action_toggle_pause(self) -> None:
        """Toggle pause/resume of auto-refresh"""
        self.is_paused = not self.is_paused
        if self.is_paused:
            self.stop_auto_refresh()
            self.notify("Auto-refresh paused")
        else:
            self.start_auto_refresh()
            self.notify("Auto-refresh resumed")
        # self.update_status_panel() # Status panel display might be removed or changed

    # update_status_panel might be removed or simplified if SystemInfoWidget is removed from layout
    def update_status_panel(self) -> None:
        """Update the status panel with current information (simplified for now)."""
        # This method might need significant rework if SystemInfoWidget is not present
        # or if its display logic changes. For now, it's a placeholder.
        pass # Placeholder

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
