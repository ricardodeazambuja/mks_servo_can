import unittest
from unittest.mock import MagicMock, patch
from typing import Optional, List # Added to resolve NameError

# Assuming FASTAPI_AVAILABLE is True for these tests
# If it were False, these tests would likely need to be skipped or handled differently.
FASTAPI_AVAILABLE = True

if FASTAPI_AVAILABLE:
    import json # Added for JSON parsing
    from fastapi import FastAPI, HTTPException
    from fastapi.testclient import TestClient
    # Assuming these are the correct paths. Adjust if necessary.
    # Removed CommandResult, CommandInjectorPayload, TemplateCommandPayload, ParameterUpdatePayload
    # as they are not defined in http_debug_server.py and not directly used by tests.
    from mks_servo_simulator.mks_simulator.interface.http_debug_server import DebugHTTPServer, JSONOutputHandler
    from mks_servo_simulator.mks_simulator.interface.llm_debug_interface import LLMDebugInterface
    from mks_servo_simulator.mks_simulator.interface.config_manager import ConfigurationManager, LiveConfigurationInterface # Corrected import
    from mks_servo_simulator.mks_simulator.interface.debug_tools import CommandInjector # LiveConfigurationInterface removed from here

    # Mock dependencies
    MotorModel = MagicMock()
    VirtualCANBus = MagicMock()

    # Simple mock for CommandResult if the actual is complex or not easily available
    # For the purpose of these tests, this mock should align with InjectedCommand from debug_tools.py
    class MockCommandResult:
        def __init__(self, success: bool, command_name: str = "mock_command",
                     execution_time_ms: float = 1.0,
                     response_data: Optional[list] = None,
                     error_message: Optional[str] = None):
            self.success = success
            self.command_name = command_name
            self.execution_time_ms = execution_time_ms
            self.response_data = response_data if response_data is not None else []
            self.error_message = error_message


    class TestDebugHTTPServer(unittest.TestCase):
        def setUp(self):
            self.mock_motor_model_instance = MotorModel() # Renamed for clarity
            self.mock_can_bus = VirtualCANBus()

            # LLMDebugInterface expects a dictionary of motors
            self.mock_motors = {1: self.mock_motor_model_instance}

            # Mock LLMDebugInterface and its command_injector
            self.llm_debug_interface = LLMDebugInterface(
                motors=self.mock_motors, # Corrected: pass a dictionary of motors
                can_bus=self.mock_can_bus
            )
            # Mock the command_injector attribute directly on the instance for LLMDebugInterface
            self.llm_debug_interface.command_injector = MagicMock(spec=CommandInjector)
            # Also mock get_available_commands on llm_debug_interface
            self.llm_debug_interface.get_available_commands = MagicMock()

            # Mock ConfigurationManager and LiveConfigurationInterface
            self.mock_config_manager = MagicMock(spec=ConfigurationManager)
            self.mock_live_config_interface = MagicMock(spec=LiveConfigurationInterface)

            self.debug_server = DebugHTTPServer(
                debug_interface=self.llm_debug_interface,
                config_manager=self.mock_config_manager,
                live_config_interface=self.mock_live_config_interface
            )
            self.client = TestClient(self.debug_server.app)

        def test_root_endpoint(self):
            response = self.client.get("/")
            self.assertEqual(response.status_code, 200)
            data = response.json()
            self.assertIn("name", data)
            self.assertIn("version", data)
            self.assertIn("description", data)
            self.assertIn("endpoints", data)
            self.assertEqual(data["name"], "MKS Servo Simulator - Debug Interface")

        def test_health_endpoint(self):
            response = self.client.get("/health")
            self.assertEqual(response.status_code, 200)
            self.assertEqual(response.json(), {"status": "healthy"})

        @patch('mks_servo_simulator.mks_simulator.interface.llm_debug_interface.LLMDebugInterface.get_system_status')
        def test_get_status(self, mock_get_system_status):
            expected_status = {"motor_count": 1, "can_bus_status": "connected"}
            mock_get_system_status.return_value = expected_status

            response = self.client.get("/status")
            self.assertEqual(response.status_code, 200)
            self.assertEqual(response.json(), expected_status)
            mock_get_system_status.assert_called_once()

        @patch('mks_servo_simulator.mks_simulator.interface.llm_debug_interface.LLMDebugInterface.get_motor_status')
        def test_get_motor_status_found(self, mock_get_motor_status):
            motor_id = 1
            expected_motor_status = {"id": motor_id, "position": 100, "speed": 50}
            mock_get_motor_status.return_value = expected_motor_status

            response = self.client.get(f"/motors/{motor_id}")
            self.assertEqual(response.status_code, 200)
            self.assertEqual(response.json(), expected_motor_status)
            mock_get_motor_status.assert_called_once_with(motor_id)

        @patch('mks_servo_simulator.mks_simulator.interface.llm_debug_interface.LLMDebugInterface.get_motor_status')
        def test_get_motor_status_not_found(self, mock_get_motor_status):
            motor_id = 99  # An ID assumed not to exist
            mock_get_motor_status.return_value = None

            response = self.client.get(f"/motors/{motor_id}")
            self.assertEqual(response.status_code, 404)
            self.assertEqual(response.json(), {"detail": f"Motor with ID {motor_id} not found."})
            mock_get_motor_status.assert_called_once_with(motor_id)

        @patch('mks_servo_simulator.mks_simulator.interface.llm_debug_interface.LLMDebugInterface.validate_expected_state')
        def test_validate_state(self, mock_validate_expected_state):
            expected_state_payload = {
                "motors": [
                    {"id": 1, "expected_position": 100, "tolerance": 5},
                    {"id": 2, "expected_speed": 0, "tolerance": 1}
                ],
                "can_bus": {
                    "expected_messages_logged": 10
                }
            }
            validation_result = {
                "overall_match": True,
                "details": [
                    {"motor_id": 1, "position_match": True},
                    {"motor_id": 2, "speed_match": True},
                    {"can_bus_messages_match": True}
                ]
            }
            mock_validate_expected_state.return_value = validation_result

            response = self.client.post("/validate", json=expected_state_payload)
            self.assertEqual(response.status_code, 200)
            self.assertEqual(response.json(), validation_result)
            # Need to assert that the mock was called with a Pydantic model,
            # not a dict. This is a bit more involved to check directly without
            # importing the Pydantic model itself into the test.
            # For now, we'll just check that it was called.
            self.assertTrue(mock_validate_expected_state.called)
            # To be more precise, one might inspect args[0] of the call
            # called_args, _ = mock_validate_expected_state.call_args
            # self.assertIsInstance(called_args[0], ExpectedSystemState) # Requires importing ExpectedSystemState

    # --- Command Injection Tests ---

    def test_inject_raw_command_success(self):
        payload = {"motor_id": 1, "command_code": 10, "data_bytes": [0, 1, 2, 3], "expect_response": True}
        mock_return_obj = MockCommandResult(
            success=True,
            command_name="RawCmd_10", # Example name, actual might vary
            execution_time_ms=5.2,
            response_data=[1, 2, 3, 4],
            error_message=None
        )
        self.llm_debug_interface.command_injector.inject_command.return_value = mock_return_obj

        response = self.client.post("/inject", json=payload)
        self.assertEqual(response.status_code, 200)
        expected_json = {
            "success": True,
            "command_name": "RawCmd_10",
            "execution_time_ms": 5.2,
            "response_data": [1, 2, 3, 4],
            "error_message": None
        }
        self.assertEqual(response.json(), expected_json)
        self.llm_debug_interface.command_injector.inject_command.assert_called_once_with(
            payload["motor_id"], payload["command_code"], payload["data_bytes"], payload["expect_response"]
        )

    def test_inject_raw_command_fail(self):
        payload = {"motor_id": 1, "command_code": 10, "data_bytes": [0, 1, 2, 3], "expect_response": True}
        mock_return_obj = MockCommandResult(
            success=False,
            command_name="RawCmd_10_Fail",
            execution_time_ms=2.1,
            response_data=[],
            error_message="Device reported error"
        )
        self.llm_debug_interface.command_injector.inject_command.return_value = mock_return_obj

        response = self.client.post("/inject", json=payload)
        self.assertEqual(response.status_code, 200)
        expected_json = {
            "success": False,
            "command_name": "RawCmd_10_Fail",
            "execution_time_ms": 2.1,
            "response_data": [],
            "error_message": "Device reported error"
        }
        self.assertEqual(response.json(), expected_json)

    def test_inject_raw_command_validation_error(self):
        # Missing command_code, data_bytes, expect_response
        response = self.client.post("/inject", json={"motor_id": 1})
        self.assertEqual(response.status_code, 422) # FastAPI's validation error

    def test_inject_template_command_success(self):
        payload = {"motor_id": 1, "template_name": "reset_motor", "args": {}}
        mock_return_obj = MockCommandResult(
            success=True,
            command_name="reset_motor_template",
            execution_time_ms=10.0,
            response_data=[], # Assuming template has no direct byte response
            error_message=None
        )
        self.llm_debug_interface.command_injector.inject_template_command.return_value = mock_return_obj

        response = self.client.post("/inject_template", json=payload)
        self.assertEqual(response.status_code, 200)
        expected_json = {
            "success": True,
            "command_name": "reset_motor_template",
            "execution_time_ms": 10.0,
            "response_data": [],
            "error_message": None
        }
        self.assertEqual(response.json(), expected_json)
        self.llm_debug_interface.command_injector.inject_template_command.assert_called_once_with(
            payload["motor_id"], payload["template_name"], payload["args"]
        )

    def test_inject_template_command_validation_error(self):
        response = self.client.post("/inject_template", json={"motor_id": 1}) # Missing template_name and args
        self.assertEqual(response.status_code, 422)

    # --- Command Listing Tests ---

    def test_get_available_commands(self):
        expected_commands = {"0x01": "Read Status", "0x02": "Reset Error"}
        self.llm_debug_interface.get_available_commands.return_value = expected_commands

        response = self.client.get("/commands")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json(), expected_commands)
        self.llm_debug_interface.get_available_commands.assert_called_once()

    def test_get_command_templates(self):
        expected_templates = {"reset_all": {"description": "Resets all motors."}}
        self.llm_debug_interface.command_injector.get_available_templates.return_value = expected_templates

        response = self.client.get("/templates")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json(), expected_templates)
        self.llm_debug_interface.command_injector.get_available_templates.assert_called_once()

    # --- Configuration Management Tests ---

    def test_get_configuration(self):
        config_summary = {"param1": "value1", "param2": 123}
        self.mock_config_manager.get_config_summary.return_value = config_summary

        response = self.client.get("/config")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json(), config_summary)
        self.mock_config_manager.get_config_summary.assert_called_once()

    def test_list_profiles(self):
        profiles = ["default", "high_speed"]
        self.mock_config_manager.list_profiles.return_value = profiles

        response = self.client.get("/config/profiles")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json(), profiles)
        self.mock_config_manager.list_profiles.assert_called_once()

    def test_load_profile_success(self):
        profile_name = "my_profile"
        # Mock the behavior of load_config (which might involve setting current_config)
        # For simplicity, let's assume load_config updates current_config and returns it or a status.
        # The http_debug_server loads, then gets a summary.
        mock_loaded_config_summary = {"loaded_param": "value"}
        self.mock_config_manager.load_config.return_value = True # Indicates success
        self.mock_config_manager.get_config_summary.return_value = mock_loaded_config_summary

        response = self.client.post(f"/config/profiles/{profile_name}/load")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json(), {"success": True, "profile": profile_name, "config": mock_loaded_config_summary})
        self.mock_config_manager.load_config.assert_called_once_with(profile_name)
        self.mock_config_manager.get_config_summary.assert_called_once()


    def test_load_profile_not_found(self):
        profile_name = "non_existent_profile"
        self.mock_config_manager.load_config.return_value = False # Indicates failure

        response = self.client.post(f"/config/profiles/{profile_name}/load")
        # Based on current http_debug_server.py, it returns 200 with success: False
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json(), {"success": False, "profile": profile_name, "error": f"Profile '{profile_name}' not found or failed to load."})
        self.mock_config_manager.load_config.assert_called_once_with(profile_name)

    def test_save_profile_success(self):
        profile_name = "new_profile"
        self.mock_config_manager.current_config = MagicMock() # Simulate a current config exists
        self.mock_config_manager.save_config.return_value = True

        response = self.client.post(f"/config/profiles/{profile_name}/save")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json(), {"success": True, "profile": profile_name})
        self.mock_config_manager.save_config.assert_called_once_with(profile_name)

    def test_save_profile_no_current_config(self):
        profile_name = "another_profile"
        self.mock_config_manager.current_config = None # Simulate no current config

        response = self.client.post(f"/config/profiles/{profile_name}/save")
        # Based on current http_debug_server.py logic
        self.assertEqual(response.status_code, 400)
        self.assertEqual(response.json(), {"detail": "No current configuration loaded to save."})

    def test_update_parameter_success(self):
        parameter_name = "latency_ms"
        payload = {"value": 5.0}
        self.mock_live_config_interface.update_parameter.return_value = True # Server checks this boolean

        response = self.client.post(f"/config/parameters/{parameter_name}", json=payload)
        self.assertEqual(response.status_code, 200)
        # The server response is {"success": True, "message": f"Updated {parameter_name} = {value}"}
        self.assertEqual(response.json(), {"success": True, "message": f"Updated {parameter_name} = {payload['value']}"})
        self.mock_live_config_interface.update_parameter.assert_called_once_with(parameter_name, payload['value'])

    def test_update_parameter_fail(self):
        parameter_name = "non_existent_param"
        payload = {"value": "invalid_value"}
        self.mock_live_config_interface.update_parameter.return_value = False # Server checks this boolean

        response = self.client.post(f"/config/parameters/{parameter_name}", json=payload)
        self.assertEqual(response.status_code, 200) # API call is fine, but operation failed
        # The server response is {"success": False, "message": f"Failed to update {parameter_name}"}
        self.assertEqual(response.json(), {"success": False, "message": f"Failed to update {parameter_name}"})

    def test_update_parameter_validation_error(self):
        parameter_name = "some_param"
        response = self.client.post(f"/config/parameters/{parameter_name}", json={}) # Missing 'value'
        self.assertEqual(response.status_code, 422) # FastAPI validation


# Tests for JSONOutputHandler (can be in the same file or a new one)
class TestJSONOutputHandler(unittest.TestCase):
    def setUp(self):
        self.mock_debug_interface = MagicMock(spec=LLMDebugInterface)
        # JSONOutputHandler calls get_system_status to fetch a timestamp
        self.mock_debug_interface.get_system_status.return_value = {"timestamp": 12345.6789}
        self.json_handler = JSONOutputHandler(debug_interface=self.mock_debug_interface)

    @patch('builtins.print')
    def test_emit_event(self, mock_print):
        event_name = "test_event"
        event_data = {"key": "value", "details": [1, 2, 3]}

        self.json_handler.emit_event(event_name, event_data)

        mock_print.assert_called_once()
        printed_json_str = mock_print.call_args[0][0]
        parsed_output = json.loads(printed_json_str)

        self.assertEqual(parsed_output["event"], event_name)
        self.assertEqual(parsed_output["data"], event_data)
        self.assertEqual(parsed_output["timestamp"], self.mock_debug_interface.get_system_status()["timestamp"])

    @patch('builtins.print')
    def test_emit_startup(self, mock_print):
        config_data = {"config_param": "config_value", "motor_count": 2}
        self.json_handler.started = False # Ensure it's False before test

        self.json_handler.emit_startup(config_data)

        mock_print.assert_called_once()
        printed_json_str = mock_print.call_args[0][0]
        parsed_output = json.loads(printed_json_str)

        self.assertEqual(parsed_output["event"], "simulator_started")
        self.assertEqual(parsed_output["config"], config_data)
        self.assertEqual(parsed_output["timestamp"], self.mock_debug_interface.get_system_status()["timestamp"])
        self.assertTrue(self.json_handler.started)

        # Test that it doesn't print again if called twice
        mock_print.reset_mock()
        self.json_handler.emit_startup(config_data) # Call again
        mock_print.assert_not_called()


    @patch('builtins.print')
    def test_emit_status_update(self, mock_print):
        current_status = {"motors": [{"id": 1, "position": 100}], "can_bus_load": 0.5}
        # Override the default timestamp for this specific test if needed, or rely on setUp's mock
        self.mock_debug_interface.get_system_status.return_value = {**current_status, "timestamp": 12345.7890}

        self.json_handler.emit_status_update()

        mock_print.assert_called_once()
        printed_json_str = mock_print.call_args[0][0]
        parsed_output = json.loads(printed_json_str)

        self.assertEqual(parsed_output["event"], "status_update")
        # The data field should be the full system status including the timestamp from the mock
        expected_data = {**current_status, "timestamp": 12345.7890}
        self.assertEqual(parsed_output["data"], expected_data)
        self.assertEqual(parsed_output["timestamp"], expected_data["timestamp"]) # Ensure top-level timestamp is also there

    @patch('builtins.print')
    def test_emit_command_executed(self, mock_print):
        motor_id = 1
        command_code = 0xF6
        success_status = True
        command_name = "SET_POSITION"
        args = {"target_position": 1000}
        result_data = {"actual_position": 1000}

        self.json_handler.emit_command_executed(
            motor_id=motor_id,
            command_code=command_code,
            success=success_status,
            command_name=command_name,
            args=args,
            result_data=result_data
        )

        mock_print.assert_called_once()
        printed_json_str = mock_print.call_args[0][0]
        parsed_output = json.loads(printed_json_str)

        self.assertEqual(parsed_output["event"], "command_executed")
        self.assertEqual(parsed_output["data"]["motor_id"], motor_id)
        self.assertEqual(parsed_output["data"]["command_code"], f"0x{command_code:02X}") # As per implementation
        self.assertEqual(parsed_output["data"]["success"], success_status)
        self.assertEqual(parsed_output["data"]["command_name"], command_name)
        self.assertEqual(parsed_output["data"]["args"], args)
        self.assertEqual(parsed_output["data"]["result_data"], result_data)
        self.assertEqual(parsed_output["timestamp"], self.mock_debug_interface.get_system_status()["timestamp"])

    @patch('builtins.print')
    def test_emit_error(self, mock_print):
        motor_id = 2
        error_type = "communication_error"
        description = "Failed to communicate with motor."

        self.json_handler.emit_error(motor_id=motor_id, error_type=error_type, description=description)

        mock_print.assert_called_once()
        printed_json_str = mock_print.call_args[0][0]
        parsed_output = json.loads(printed_json_str)

        self.assertEqual(parsed_output["event"], "error")
        self.assertEqual(parsed_output["data"]["motor_id"], motor_id)
        self.assertEqual(parsed_output["data"]["error_type"], error_type)
        self.assertEqual(parsed_output["data"]["description"], description)
        self.assertEqual(parsed_output["timestamp"], self.mock_debug_interface.get_system_status()["timestamp"])


if __name__ == '__main__':
    # This is to ensure tests can be run directly from this file,
    # though typically a test runner like `python -m unittest discover` would be used.
    if FASTAPI_AVAILABLE:
        unittest.main()
    else:
        print("Skipping HTTP Debug Server tests as FastAPI is not available.")
