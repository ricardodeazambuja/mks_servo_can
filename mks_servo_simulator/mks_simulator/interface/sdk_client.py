import requests
import time
from typing import Dict, Any, Optional, List, Union

class SimulatorAPIError(Exception):
    """Custom exception for API errors."""
    def __init__(self, message: str, status_code: Optional[int] = None, response_data: Optional[Dict[str, Any]] = None):
        super().__init__(message)
        self.status_code = status_code
        self.response_data = response_data

    def __str__(self):
        return f"SimulatorAPIError: {self.args[0]} (Status Code: {self.status_code})"


class MKSSimulatorClient:
    """
    Python client for interacting with the MKS Servo Simulator's HTTP Debug API.
    """
    DEFAULT_TIMEOUT = 10 # seconds

    def __init__(self, base_url: str = "http://localhost:8765"):
        """
        Initializes the MKS Simulator Client.

        Args:
            base_url: The base URL of the simulator's HTTP API.
        """
        self.base_url = base_url.rstrip('/')
        self.session = requests.Session()

    def _request(self, method: str, endpoint: str, params: Optional[Dict[str, Any]] = None,
                 json_data: Optional[Dict[str, Any]] = None,
                 timeout: Optional[Union[float, tuple]] = DEFAULT_TIMEOUT) -> Dict[str, Any]:
        """
        Helper method to make HTTP requests to the API.

        Args:
            method: HTTP method (GET, POST, etc.).
            endpoint: API endpoint path (e.g., "/status").
            params: URL parameters for GET requests.
            json_data: JSON payload for POST/PUT requests.
            timeout: Request timeout in seconds.

        Returns:
            The JSON response from the API as a dictionary.

        Raises:
            SimulatorAPIError: If the API returns an error or network issues occur.
        """
        full_url = self.base_url + endpoint
        headers = {"Accept": "application/json"}
        if json_data:
            headers["Content-Type"] = "application/json"

        try:
            response = self.session.request(
                method,
                full_url,
                params=params,
                json=json_data,
                headers=headers,
                timeout=timeout
            )
            response.raise_for_status()  # Raises HTTPError for 4xx/5xx responses
            if response.status_code == 204: # No Content
                return {}
            return response.json()
        except requests.exceptions.HTTPError as e:
            response_data = None
            try:
                response_data = e.response.json() if e.response else None
            except requests.exceptions.JSONDecodeError:
                pass # Keep response_data as None if it's not valid JSON
            raise SimulatorAPIError(
                f"HTTP error occurred: {e}",
                status_code=e.response.status_code,
                response_data=response_data
            ) from e
        except requests.exceptions.RequestException as e:
            # For connection errors, timeouts, etc.
            raise SimulatorAPIError(f"Request failed: {e}") from e
        except requests.exceptions.JSONDecodeError as e:
            raise SimulatorAPIError(f"Failed to decode JSON response: {e}") from e


    # --- General System Endpoints ---
    def get_root_info(self) -> Dict[str, Any]:
        """Gets the root information of the API."""
        return self._request("GET", "/")

    def get_health(self) -> Dict[str, Any]:
        """Gets the health status of the simulator."""
        return self._request("GET", "/health")

    def get_status(self) -> Dict[str, Any]:
        """Retrieves the overall system status of the simulator."""
        return self._request("GET", "/status")

    def get_motor_status(self, motor_id: int) -> Optional[Dict[str, Any]]:
        """
        Retrieves the status of a specific motor.

        Args:
            motor_id: The ID of the motor.

        Returns:
            A dictionary containing the motor's status, or None if not found (404).
        """
        try:
            return self._request("GET", f"/motors/{motor_id}")
        except SimulatorAPIError as e:
            if e.status_code == 404:
                return None
            raise

    def get_command_history(self, motor_id: Optional[int] = None, limit: int = 50) -> Dict[str, Any]: # API returns dict, not List[CommandRecord]
        """
        Retrieves the command history.

        Args:
            motor_id: Optional motor ID to filter history.
            limit: Maximum number of records to return.

        Returns:
            A list of command records.
        """
        params = {}
        if motor_id is not None:
            params["motor_id"] = motor_id
        if limit is not None:
            params["limit"] = limit
        return self._request("GET", "/command_history", params=params)

    def validate_state(self, expected_state: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validates the current simulator state against an expected state.

        Args:
            expected_state: A dictionary describing the expected state.

        Returns:
            A dictionary containing validation results.
        """
        return self._request("POST", "/validate", json_data=expected_state)

    # --- Command Injection Endpoints ---
    def inject_command(self, motor_id: int, command_code: int, data_bytes: List[int] = [], expect_response: bool = True) -> Dict[str, Any]:
        """
        Injects a raw command to a motor. (Note: expect_response is part of payload based on http_debug_server)

        Args:
            motor_id: The ID of the motor.
            command_code: The command code (integer).
            data_bytes: A list of integers representing data bytes.
            expect_response: Whether the server should wait for and return a response from the command.

        Returns:
            A dictionary containing the result of the command injection.
        """
        payload = {
            "motor_id": motor_id,
            "command_code": command_code,
            "data_bytes": data_bytes,
            "expect_response": expect_response
        }
        return self._request("POST", "/inject", json_data=payload)

    def inject_template_command(self, motor_id: int, template_name: str, args: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Injects a command using a predefined template.

        Args:
            motor_id: The ID of the motor.
            template_name: The name of the command template.
            args: Optional arguments for the template.

        Returns:
            A dictionary containing the result of the command injection.
        """
        payload = {"motor_id": motor_id, "template_name": template_name, "args": args or {}}
        return self._request("POST", "/inject_template", json_data=payload)

    def get_available_commands(self) -> Dict[str, Any]:
        """Retrieves the list of available raw commands."""
        return self._request("GET", "/commands")

    def get_command_templates(self) -> Dict[str, Any]:
        """Retrieves the list of available command templates."""
        return self._request("GET", "/templates")

    def get_injection_stats(self) -> Dict[str, Any]:
        """Retrieves statistics about command injections."""
        return self._request("GET", "/injection_stats")

    # --- Scenario Endpoints ---
    def run_test_scenario(self, motor_id: int, scenario_name: str, params: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Runs a predefined test scenario on a motor.

        Args:
            motor_id: The ID of the motor.
            scenario_name: The name of the scenario to run.
            params: Optional parameters for the scenario.

        Returns:
            A dictionary containing the result of the scenario execution.
        """
        payload = {"motor_id": motor_id, "scenario_name": scenario_name, "params": params or {}}
        return self._request("POST", "/run_scenario", json_data=payload)

    # --- Configuration Endpoints ---
    def get_configuration(self) -> Dict[str, Any]:
        """Retrieves the current system configuration."""
        return self._request("GET", "/config")

    def list_config_profiles(self) -> List[str]: # API returns List[str] directly
        """Lists available configuration profiles."""
        # The http_debug_server returns a simple list for this endpoint.
        # Our _request expects a Dict, so we might need a special handling or adjust _request.
        # For now, let's assume _request can handle non-dict JSON if Content-Type is application/json
        # Or, more robustly, handle it here:
        full_url = self.base_url + "/config/profiles"
        try:
            response = self.session.get(full_url, timeout=self.DEFAULT_TIMEOUT)
            response.raise_for_status()
            return response.json() # This should be List[str]
        except requests.exceptions.HTTPError as e:
            raise SimulatorAPIError(f"HTTP error: {e}", status_code=e.response.status_code) from e
        except requests.exceptions.RequestException as e:
            raise SimulatorAPIError(f"Request failed: {e}") from e

    def load_config_profile(self, profile_name: str) -> Dict[str, Any]:
        """Loads a configuration profile by name."""
        return self._request("POST", f"/config/profiles/{profile_name}/load")

    def save_config_profile(self, profile_name: str) -> Dict[str, Any]:
        """Saves the current configuration to a profile by name."""
        return self._request("POST", f"/config/profiles/{profile_name}/save")

    def delete_config_profile(self, profile_name: str) -> Dict[str, Any]:
        """Deletes a configuration profile by name."""
        return self._request("DELETE", f"/config/profiles/{profile_name}")

    def get_motor_templates_config(self) -> Dict[str, Any]:
        """Retrieves motor configuration templates."""
        return self._request("GET", "/config/templates")

    def apply_motor_template(self, template_name: str, motor_id: int) -> Dict[str, Any]:
        """Applies a motor configuration template to a specific motor."""
        payload = {"template_name": template_name, "motor_id": motor_id}
        return self._request("POST", "/config/templates/apply", json_data=payload)

    def get_adjustable_parameters(self) -> Dict[str, Any]:
        """Retrieves live adjustable parameters and their metadata."""
        return self._request("GET", "/config/parameters")

    def update_live_parameter(self, parameter_name: str, value: Any) -> Dict[str, Any]:
        """
        Updates a live adjustable parameter.

        Args:
            parameter_name: The name of the parameter to update.
            value: The new value for the parameter.

        Returns:
            A dictionary confirming the update.
        """
        payload = {"value": value}
        return self._request("POST", f"/config/parameters/{parameter_name}", json_data=payload)

    # --- Performance Endpoints ---
    def get_performance_metrics(self) -> Dict[str, Any]:
        """Retrieves current performance metrics."""
        return self._request("GET", "/performance")

    def get_performance_history(self, minutes: int = 10) -> Dict[str, Any]:
        """
        Retrieves historical performance metrics.

        Args:
            minutes: The number of past minutes to retrieve history for.

        Returns:
            A dictionary containing historical performance data.
        """
        params = {"minutes": minutes}
        return self._request("GET", "/performance/history", params=params)

    def reset_performance_metrics(self) -> Dict[str, Any]:
        """Resets the collected performance metrics."""
        return self._request("POST", "/performance/reset")


if __name__ == '__main__':
    # Example Usage (requires the simulator to be running)
    client = MKSSimulatorClient(base_url="http://localhost:8765")

    try:
        print("--- Root Info ---")
        print(client.get_root_info())

        print("\n--- Health ---")
        print(client.get_health())

        print("\n--- System Status ---")
        status = client.get_status()
        print(status)

        if status.get("motors") and len(status["motors"]) > 0:
            motor_id_to_test = status["motors"][0]["can_id"]
            print(f"\n--- Motor {motor_id_to_test} Status ---")
            motor_status = client.get_motor_status(motor_id_to_test)
            print(motor_status)
        else:
            motor_id_to_test = 1 # Fallback if no motors in status
            print(f"\n--- Motor {motor_id_to_test} Status (fallback ID) ---")
            motor_status = client.get_motor_status(motor_id_to_test)
            if motor_status:
                print(motor_status)
            else:
                print(f"Motor {motor_id_to_test} not found or no motors available.")


        print("\n--- Command History (first 5) ---")
        history = client.get_command_history(limit=5)
        print(history)

        print("\n--- Available Commands ---")
        commands = client.get_available_commands()
        print(commands)

        print("\n--- Config Profiles ---")
        profiles = client.list_config_profiles()
        print(profiles)

        # Example of an operation that might fail if profile doesn't exist
        # print("\n--- Loading 'default' profile ---")
        # try:
        #     load_result = client.load_config_profile("default")
        #     print(load_result)
        # except SimulatorAPIError as e:
        #     print(f"Could not load profile: {e}")

    except SimulatorAPIError as e:
        print(f"An API error occurred: {e}")
        if e.response_data:
            print(f"Response data: {e.response_data}")
    except requests.exceptions.ConnectionError as e:
        print(f"Could not connect to the simulator at {client.base_url}. Ensure it's running.")
