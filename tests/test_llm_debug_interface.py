import unittest
import time
import sys
from unittest.mock import MagicMock, patch, PropertyMock

# Mock modules before importing LLMDebugInterface
# This is a common pattern when dealing with modules that might have complex
# dependencies or side effects on import, especially if those modules are
# not directly under test or are part of a larger system not fully available
# in the unit test environment.

# Import actual classes for spec
from mks_servo_simulator.mks_simulator.motor_model import SimulatedMotor # Corrected class name
from mks_servo_simulator.mks_simulator.virtual_can_bus import VirtualCANBus
# sys.modules['mks_servo_simulator.mks_simulator.helpers'] = MagicMock() # If helpers is also an issue - keeping for now if it was needed

# Now, import the class to be tested
from mks_servo_simulator.mks_simulator.interface.llm_debug_interface import LLMDebugInterface, CommandRecord, ErrorRecord, MANUAL_COMMANDS


class TestLLMDebugInterface(unittest.TestCase):
    def setUp(self):
        # Create mock motor instances using actual classes for spec
        self.mock_motor_1 = MagicMock(spec=SimulatedMotor) # Corrected spec
        self.mock_motor_1.can_id = 1
        self.mock_motor_1.name = "Motor1"
        self.mock_motor_1.enabled = True
        self.mock_motor_1.encoder_position = 1000
        self.mock_motor_1.current_angle = 100.0
        self.mock_motor_1.target_position = 1000
        self.mock_motor_1.current_speed = 0
        self.mock_motor_1.target_speed = 0
        self.mock_motor_1.is_homing = False
        self.mock_motor_1.is_moving = False
        self.mock_motor_1.get_status_dict = MagicMock(return_value={
            "can_id": 1, "name": "Motor1", "enabled": True, "encoder_position": 1000,
            "current_angle": 100.0, "target_position": 1000, "current_speed": 0,
            "target_speed": 0, "is_homing": False, "is_moving": False, "current_ma": 50, "error_code": 0
        })


        self.mock_motor_2 = MagicMock(spec=SimulatedMotor) # Corrected spec
        self.mock_motor_2.can_id = 2
        self.mock_motor_2.name = "Motor2"
        self.mock_motor_2.enabled = False
        self.mock_motor_2.encoder_position = 500
        self.mock_motor_2.current_angle = 50.0
        self.mock_motor_2.target_position = 500
        self.mock_motor_2.current_speed = 10
        self.mock_motor_2.target_speed = 10
        self.mock_motor_2.is_homing = True
        self.mock_motor_2.is_moving = True
        self.mock_motor_2.get_status_dict = MagicMock(return_value={
            "can_id": 2, "name": "Motor2", "enabled": False, "encoder_position": 500,
            "current_angle": 50.0, "target_position": 500, "current_speed": 10,
            "target_speed": 10, "is_homing": True, "is_moving": True, "current_ma": 10, "error_code": 1
        })

        self.mock_motors = {1: self.mock_motor_1, 2: self.mock_motor_2}

        self.mock_can_bus = MagicMock(spec=VirtualCANBus)
        self.mock_can_bus.get_bus_load_percentage = MagicMock(return_value=15.5)
        self.mock_can_bus.get_total_messages_sent = MagicMock(return_value=1234)
        self.mock_can_bus.get_total_messages_received = MagicMock(return_value=5678)
        self.mock_can_bus.is_running = PropertyMock(return_value=True)

        self.interface = LLMDebugInterface(motors=self.mock_motors, can_bus=self.mock_can_bus)
        # Ensure start_time is set for uptime calculation
        self.interface.start_time = time.time() - 10 # Simulating 10 seconds of uptime

        self.interface.command_history.clear()
        self.interface.recent_errors.clear()

    def test_get_system_status(self):
        status = self.interface.get_system_status()

        self.assertIsInstance(status, dict)
        self.assertIn("timestamp", status)
        self.assertGreaterEqual(status["uptime_seconds"], 10)
        self.assertEqual(status["simulator_status"], "running")
        self.assertEqual(status["motor_count"], 2)

        self.assertIn("motors", status)
        self.assertEqual(len(status["motors"]), 2)
        self.assertEqual(status["motors"][0]["can_id"], 1)
        self.assertEqual(status["motors"][0]["encoder_position"], 1000)
        self.assertEqual(status["motors"][1]["can_id"], 2)
        self.assertEqual(status["motors"][1]["is_homing"], True)

        self.assertIn("communication", status)
        self.assertEqual(status["communication"]["can_bus_status"], "running")
        self.assertEqual(status["communication"]["can_bus_load_percentage"], 15.5)

        self.assertIn("errors", status)
        self.assertEqual(status["errors"]["error_count"], 0) # Initially no errors recorded by interface itself
        self.assertEqual(status["errors"]["recent_errors"], [])


    def test_get_motor_status_found(self):
        status = self.interface.get_motor_status(1)
        self.assertIsNotNone(status)
        self.assertEqual(status["can_id"], 1)
        self.assertEqual(status["name"], "Motor1")
        self.assertEqual(status["encoder_position"], 1000)
        self.mock_motor_1.get_status_dict.assert_called_once()

    def test_get_motor_status_not_found(self):
        status = self.interface.get_motor_status(99)
        self.assertIsNone(status)

    def test_record_and_get_command_history(self):
        self.interface.record_command(1, 0x10, {"param": "value1"}, True, "Command OK", {"res": "data1"})
        time.sleep(0.001) # Ensure unique timestamps if system clock resolution is low
        self.interface.record_command(2, 0x20, {"param": "value2"}, False, "Command Failed", {"res": "data2"})

        history_all = self.interface.get_command_history()
        self.assertEqual(len(history_all), 2)
        self.assertIsInstance(history_all[0], CommandRecord)
        self.assertEqual(history_all[0].motor_id, 1)
        self.assertEqual(history_all[0].command_code_hex, "0x10")
        self.assertTrue(history_all[0].success)
        self.assertEqual(history_all[1].motor_id, 2)
        self.assertEqual(history_all[1].command_code_hex, "0x20")
        self.assertFalse(history_all[1].success)

        history_motor1 = self.interface.get_command_history(motor_id=1)
        self.assertEqual(len(history_motor1), 1)
        self.assertEqual(history_motor1[0].motor_id, 1)

        history_limit1 = self.interface.get_command_history(limit=1)
        self.assertEqual(len(history_limit1), 1)
        # Should be the latest command
        self.assertEqual(history_limit1[0].motor_id, 2)


    def test_validate_expected_state_pass(self):
        expected_state = {
            "motors": [
                {"id": 1, "current_angle": 100.0, "tolerance": 1.0},
                {"id": 2, "current_speed": 10, "tolerance": 2},
            ]
        }
        # Ensure motors return the expected values via get_status_dict
        self.mock_motor_1.get_status_dict.return_value["current_angle"] = 100.0
        self.mock_motor_2.get_status_dict.return_value["current_speed"] = 10

        result = self.interface.validate_expected_state(expected_state)
        self.assertTrue(result["overall_match"])
        self.assertEqual(len(result["details"]), 2)
        self.assertTrue(all(detail["match"] for detail in result["details"]))


    def test_validate_expected_state_fail(self):
        expected_state = {
            "motors": [
                {"id": 1, "current_angle": 105.0, "tolerance": 1.0}, # Mismatch here
                {"id": 2, "current_speed": 10, "tolerance": 1},
            ]
        }
        self.mock_motor_1.get_status_dict.return_value["current_angle"] = 100.0 # Actual is 100
        self.mock_motor_2.get_status_dict.return_value["current_speed"] = 10

        result = self.interface.validate_expected_state(expected_state)
        self.assertFalse(result["overall_match"])
        self.assertEqual(len(result["details"]), 2)
        self.assertFalse(result["details"][0]["match"])
        self.assertEqual(result["details"][0]["motor_id"], 1)
        self.assertEqual(result["details"][0]["parameter"], "current_angle")
        self.assertIn("Expected 105.0 (+/- 1.0), Actual 100.0", result["details"][0]["message"])
        self.assertTrue(result["details"][1]["match"])


    def test_get_available_commands_loaded(self):
        mock_manual_commands_data = {
            "0x30": {"name": "Test Command", "description": "A test command", "request": {"parameters": [{"name": "speed", "type": "int"}]}},
            "0x31": {"name": "Another Command", "description": "Another one", "request": {}}
        }
        # MANUAL_COMMANDS is directly imported, so we patch it in the module where it's defined and used.
        with patch.dict('mks_servo_simulator.mks_simulator.interface.llm_debug_interface.MANUAL_COMMANDS', mock_manual_commands_data, clear=True):
            commands_info = self.interface.get_available_commands()

        self.assertIn("commands", commands_info)
        self.assertEqual(commands_info["total_commands"], 2)
        self.assertIn("0x30", commands_info["commands"])
        self.assertEqual(commands_info["commands"]["0x30"]["name"], "Test Command")

    def test_get_available_commands_not_loaded(self):
        with patch.dict('mks_servo_simulator.mks_simulator.interface.llm_debug_interface.MANUAL_COMMANDS', {}, clear=True):
            commands_info = self.interface.get_available_commands()

        self.assertEqual(commands_info.get("note"), "Manual commands specification not loaded or empty.")
        self.assertEqual(commands_info.get("total_commands", 0), 0)


    def test_record_error(self):
        self.interface.record_error(1, "firmware_error", "Motor firmware crashed", {"register": "0xABC"})

        self.assertEqual(len(self.interface.recent_errors), 1)
        error_record = self.interface.recent_errors[0]
        self.assertIsInstance(error_record, ErrorRecord)
        self.assertEqual(error_record.motor_id, 1)
        self.assertEqual(error_record.error_type, "firmware_error")
        self.assertEqual(error_record.description, "Motor firmware crashed")
        self.assertEqual(error_record.context_data, {"register": "0xABC"})
        self.assertIsNotNone(error_record.timestamp)

    def test_get_debug_summary(self):
        # Add an error to test its inclusion in summary
        self.interface.record_error(1, "test_error", "This is a test error")
        summary = self.interface.get_debug_summary()

        self.assertIsInstance(summary, str)
        self.assertIn("Uptime:", summary)
        self.assertIn("Simulator Status: running", summary)
        self.assertIn("Motor Count: 2", summary)
        self.assertIn("Motor 1 (Motor1):", summary)
        self.assertIn("Enabled: True", summary) # From mock_motor_1
        self.assertIn("Encoder: 1000", summary)
        self.assertIn("Motor 2 (Motor2):", summary)
        self.assertIn("Enabled: False", summary) # From mock_motor_2
        self.assertIn("Is Homing: True", summary)
        self.assertIn("CAN Bus Status: running", summary)
        self.assertIn("Recent Errors (Last", summary)
        self.assertIn("Motor 1: test_error - This is a test error", summary) # Check error in summary
        self.assertIn("Command History (Last", summary)


if __name__ == '__main__':
    unittest.main()
