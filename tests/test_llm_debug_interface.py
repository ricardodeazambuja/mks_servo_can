"""
Tests for the agent-facing debug interface, against real simulated motors.

This file used to build its motors with `MagicMock(spec=SimulatedMotor)` and
then assign the attributes the interface expected - `name`, `enabled`,
`encoder_position`, `current_angle` and a dozen more. `spec` constrains reads
from a mock only until something writes to it, so each assignment quietly
taught the mock an attribute the real class had never had. The tests passed
against a motor that did not exist, while `/status` returned HTTP 500 and
`--json-output` crashed on startup for everyone else.

So every motor here is a real `SimulatedMotor`. If the interface reads a field
the motor does not have, these tests fail - which is the only arrangement in
which they are worth running.
"""
import asyncio
import time
import unittest
from unittest.mock import patch

from mks_servo_simulator.mks_simulator.interface.llm_debug_interface import (
    ErrorRecord,
    LLMDebugInterface,
)
from mks_servo_simulator.mks_simulator.motor_model import SimulatedMotor
from mks_servo_simulator.mks_simulator.virtual_can_bus import VirtualCANBus


class TestLLMDebugInterface(unittest.TestCase):
    def setUp(self):
        self.loop = asyncio.new_event_loop()
        self.bus = VirtualCANBus(self.loop)

        # Two real motors in deliberately different states, so a field wired to
        # a constant cannot satisfy assertions about both.
        self.motor_1 = SimulatedMotor(can_id=1, loop=self.loop)
        self.motor_1.is_enabled = True
        self.motor_1.position_steps = 4096.0        # a quarter turn
        self.motor_1.current_rpm = 0.0

        self.motor_2 = SimulatedMotor(can_id=2, loop=self.loop)
        self.motor_2.is_enabled = False
        self.motor_2.position_steps = 2048.0        # an eighth of a turn
        self.motor_2.current_rpm = 30.0
        self.motor_2.target_position_steps = 8192.0

        self.bus.add_motor(self.motor_1)
        self.bus.add_motor(self.motor_2)

        self.interface = LLMDebugInterface(
            motors=self.bus.simulated_motors, can_bus=self.bus
        )
        self.bus.debug_interface = self.interface
        self.interface.start_time = time.time() - 10  # 10 seconds of uptime

        self.interface.command_history.clear()
        self.interface.recent_errors.clear()

    def tearDown(self):
        self.loop.close()

    def test_get_system_status(self):
        status = self.interface.get_system_status()

        self.assertIsInstance(status, dict)
        self.assertIn("timestamp", status)
        self.assertGreaterEqual(status["uptime_seconds"], 10)
        self.assertEqual(status["simulator_status"], "running")
        self.assertEqual(status["motor_count"], 2)

        self.assertIn("motors", status)
        self.assertEqual(len(status["motors"]), 2)
        self.assertEqual(status["motors"]["1"]["can_id"], 1)
        self.assertEqual(status["motors"]["2"]["can_id"], 2)

        self.assertIn("total_messages", status["communication"])
        self.assertEqual(len(status["errors"]), 0)

    def test_status_reports_the_motors_actual_state(self):
        """
        The reported values must be the motors', not plausible defaults.

        This is the specific regression guard. Every assertion below failed
        silently under the old implementation - it reported position 0, speed 0
        and enabled True for both motors regardless of what they were doing.
        """
        motors = self.interface.get_system_status()["motors"]

        self.assertEqual(motors["1"]["position_steps"], 4096.0)
        self.assertAlmostEqual(motors["1"]["position_degrees"], 90.0)
        self.assertTrue(motors["1"]["enabled"])
        self.assertFalse(motors["1"]["moving"])
        self.assertIsNone(motors["1"]["target_position_steps"])

        self.assertEqual(motors["2"]["position_steps"], 2048.0)
        self.assertAlmostEqual(motors["2"]["position_degrees"], 45.0)
        self.assertFalse(motors["2"]["enabled"])
        self.assertTrue(motors["2"]["moving"])
        self.assertEqual(motors["2"]["current_rpm"], 30.0)
        self.assertEqual(motors["2"]["target_position_steps"], 8192.0)
        self.assertEqual(motors["2"]["position_error_steps"], 6144.0)

    def test_status_follows_a_motor_as_it_changes(self):
        """A second read after moving the motor must show the new state."""
        before = self.interface.get_motor_status(1)
        self.motor_1.position_steps = 8192.0
        self.motor_1.current_rpm = 120.0
        after = self.interface.get_motor_status(1)

        self.assertNotEqual(before["position_steps"], after["position_steps"])
        self.assertEqual(after["position_steps"], 8192.0)
        self.assertAlmostEqual(after["position_degrees"], 180.0)
        self.assertTrue(after["moving"])

    def test_get_motor_status_found(self):
        status = self.interface.get_motor_status(1)
        self.assertIsNotNone(status)
        self.assertEqual(status["can_id"], 1)
        self.assertEqual(status["position_steps"], 4096.0)

    def test_get_motor_status_not_found(self):
        self.assertIsNone(self.interface.get_motor_status(99))

    def test_record_and_get_command_history(self):
        self.interface.record_command(
            motor_id=1,
            command_code=0x10,
            command_name="TestCommand1",
            parameters={"param": "value1"},
            response_time=0.01,
            success=True,
            error_message=None,
        )
        time.sleep(0.001)  # keep timestamps distinct on coarse clocks
        self.interface.record_command(
            motor_id=2,
            command_code=0x20,
            command_name="TestCommand2",
            parameters={"param": "value2"},
            response_time=0.02,
            success=False,
            error_message="Command Failed",
        )

        history_all = self.interface.get_command_history()
        self.assertEqual(len(history_all), 2)
        self.assertEqual(history_all[0]["motor_id"], 1)
        self.assertEqual(history_all[0]["command_code"], "0x10")
        self.assertTrue(history_all[0]["success"])
        self.assertEqual(history_all[1]["command_code"], "0x20")
        self.assertFalse(history_all[1]["success"])

        self.assertEqual(len(self.interface.get_command_history(motor_id=1)), 1)

        history_limit1 = self.interface.get_command_history(limit=1)
        self.assertEqual(len(history_limit1), 1)
        self.assertEqual(history_limit1[0]["motor_id"], 2)

    def test_validate_expected_state_pass(self):
        expected_state = {
            "motors": [
                {"id": 1, "position_degrees": 90.0, "tolerance": 0.1},
                {"id": 2, "current_rpm": 30.0, "tolerance": 0.1},
            ]
        }
        result = self.interface.validate_expected_state(expected_state)
        self.assertTrue(result["passed"], result["failures"])
        self.assertEqual(len(result["failures"]), 0)

    def test_validate_expected_state_fail(self):
        expected_state = {
            "motors": [
                {"id": 1, "position_degrees": 105.0, "tolerance": 1.0},  # actually 90
                {"id": 2, "current_rpm": 30.0, "tolerance": 1.0},        # correct
            ]
        }
        result = self.interface.validate_expected_state(expected_state)
        self.assertFalse(result["passed"])
        self.assertEqual(len(result["failures"]), 1)
        self.assertIn("position_degrees", result["failures"][0])

    def test_validate_warns_about_unknown_fields(self):
        """
        Asking about a field that does not exist must warn, not silently pass.

        Under the old schema this was how a validation could look green while
        checking nothing at all.
        """
        result = self.interface.validate_expected_state(
            {"motors": [{"id": 1, "no_such_field": 1.0}]}
        )
        self.assertTrue(any("no_such_field" in w for w in result["warnings"]))

    def test_get_available_commands_loaded(self):
        mock_manual_commands_data = {
            "0x30": {
                "name": "Test Command",
                "description": "A test command",
                "request": {"parameters": [{"name": "speed", "type": "int"}]},
            },
            "0x31": {"name": "Another Command", "description": "Another one", "request": {}},
        }
        with patch.dict(
            "mks_servo_simulator.mks_simulator.interface.llm_debug_interface.MANUAL_COMMANDS",
            mock_manual_commands_data,
            clear=True,
        ):
            commands_info = self.interface.get_available_commands()

        self.assertIn("commands", commands_info)
        self.assertEqual(commands_info["total_commands"], 2)
        command_0x30 = next(
            cmd for cmd in commands_info["commands"] if cmd["code"] == "0x30"
        )
        self.assertEqual(command_0x30["name"], "Test Command")

    def test_get_available_commands_not_loaded(self):
        with patch.dict(
            "mks_servo_simulator.mks_simulator.interface.llm_debug_interface.MANUAL_COMMANDS",
            {},
            clear=True,
        ):
            commands_info = self.interface.get_available_commands()

        self.assertEqual(
            commands_info.get("note"), "Manual commands specification not loaded"
        )
        self.assertEqual(commands_info.get("total_commands", 0), 0)

    def test_manual_command_spec_is_actually_available(self):
        """
        The shipped spec must load.

        The path to it was off by one directory, so `available_commands`
        reported 0 and the command reference an agent consults was empty. That
        failed silently; this asserts it does not.
        """
        from mks_servo_simulator.mks_simulator.interface import llm_debug_interface

        self.assertGreater(
            len(llm_debug_interface.MANUAL_COMMANDS),
            0,
            "manual command specification failed to load - check "
            "_MANUAL_SPEC_PATH in llm_debug_interface.py",
        )

    def test_record_error(self):
        self.interface.record_error(
            1, "firmware_error", "Motor firmware crashed", {"register": "0xABC"}
        )

        self.assertEqual(len(self.interface.recent_errors), 1)
        error_record = self.interface.recent_errors[0]
        self.assertIsInstance(error_record, ErrorRecord)
        self.assertEqual(error_record.motor_id, 1)
        self.assertEqual(error_record.error_type, "firmware_error")
        self.assertEqual(error_record.description, "Motor firmware crashed")
        self.assertEqual(error_record.context, {"register": "0xABC"})
        self.assertIsNotNone(error_record.timestamp)

    def test_motor_anomalies_reach_the_error_log(self):
        """
        A motor reporting an anomaly must show up in the interface's errors.

        `record_error` previously had no callers anywhere in the simulator, so
        the `errors` array was permanently empty and the dashboard's error
        panel was decorative. This wires bus -> interface and checks the path
        end to end.
        """
        self.motor_1.report_anomaly(
            "move_superseded", "old move abandoned", new_target_steps=1234
        )

        self.assertEqual(len(self.interface.recent_errors), 1)
        record = self.interface.recent_errors[0]
        self.assertEqual(record.motor_id, 1)
        self.assertEqual(record.error_type, "move_superseded")
        self.assertEqual(record.context["new_target_steps"], 1234)

    def test_get_debug_summary(self):
        self.interface.record_error(1, "test_error", "This is a test error", {})
        summary = self.interface.get_debug_summary()

        self.assertIsInstance(summary, str)
        self.assertIn("Simulator uptime:", summary)
        self.assertIn("Motors: 2", summary)
        self.assertIn("Motor 1: 90.00deg enabled, stopped", summary)
        self.assertIn("Motor 2: 45.00deg disabled, moving", summary)
        self.assertIn("target 180.00deg", summary)
        self.assertIn("Errors: 1", summary)
        self.assertIn("Commands: 0", summary)


if __name__ == "__main__":
    unittest.main()
