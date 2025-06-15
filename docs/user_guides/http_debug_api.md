# HTTP Debug API for MKS Servo Simulator

The MKS Servo Simulator includes an HTTP Debug API server, built with FastAPI, to provide programmatic access to the simulator's state, command injection capabilities, and validation features. This API is particularly useful for automated testing, integration with other tools, and interaction with Large Language Models (LLMs).

## Enabling and Accessing the API

The HTTP Debug API server is started automatically when you run the `mks-servo-simulator`.
By default, it listens on:
- **Host**: `127.0.0.1` (localhost)
- **Port**: `8765`

You can access the interactive API documentation (Swagger UI) provided by FastAPI by navigating to `http://localhost:8765/docs` in your web browser while the simulator is running. An alternative ReDoc interface is available at `http://localhost:8765/redoc`.

## Key Endpoints

Here's an overview of some key endpoints. For full details, request/response models, and to try them out, please refer to the live `/docs` endpoint.

### General Information

*   **`GET /`**:
    *   **Description**: Provides basic information about the API and lists available top-level endpoint categories.
    *   **Response**: JSON object with API name, version, description, and a map of endpoints.

*   **`GET /health`**:
    *   **Description**: A simple health check for the simulator's HTTP API.
    *   **Response**: JSON object indicating status (e.g., `{"status": "healthy", "uptime": 120.5, ...}`).

### Simulator State and Status

*   **`GET /status`**:
    *   **Description**: Retrieves the complete current state of the simulator, including all motor statuses, communication statistics, and recent errors.
    *   **Response**: Comprehensive JSON object detailing the system state.

*   **`GET /motors/{motor_id}`**:
    *   **Description**: Gets the detailed status for a specific motor by its CAN ID.
    *   **Path Parameter**: `motor_id` (integer).
    *   **Response**: JSON object with the specified motor's current state. Returns 404 if motor not found.

*   **`GET /history`**:
    *   **Description**: Fetches recent command execution history. Can be filtered by `motor_id` and `limit`.
    *   **Query Parameters**: `motor_id` (optional int), `limit` (optional int, default 50).
    *   **Response**: JSON object containing the command history.

### Command Injection

*   **`POST /inject`**:
    *   **Description**: Injects a raw CAN command to a specified motor.
    *   **Request Body** (Pydantic model `RawCommandPayload`):
        ```json
        {
            "motor_id": 1,
            "command_code": 246, // Example: 0xF6 for speed mode
            "data_bytes": [1, 0, 100, 0], // Example: speed 100
            "expect_response": true
        }
        ```
    *   **Response**: JSON object with command execution result (success, name, time, response data, error).

*   **`POST /inject_template`**:
    *   **Description**: Injects a command based on a predefined template.
    *   **Request Body** (Pydantic model `TemplateCommandPayload`):
        ```json
        {
            "motor_id": 1,
            "template_name": "enable_motor",
            "args": {} // Optional arguments for the template
        }
        ```
    *   **Response**: JSON object with command execution result.

*   **`GET /templates`**:
    *   **Description**: Retrieves a list of available command templates that can be used with the `/inject_template` endpoint.
    *   **Response**: JSON object mapping template names to their specifications.

### State Validation

*   **`POST /validate`**:
    *   **Description**: Validates the current simulator state against an expected state provided in the request body.
    *   **Request Body** (Example):
        ```json
        {
            "motors": [
                {
                    "id": 1,
                    "current_angle_degrees": 90.0,
                    "tolerance": 1.5
                },
                {
                    "id": 2,
                    "status_flags.moving": false,
                    "tolerance": 0.0
                }
            ],
            "global_tolerance": 0.1 // Optional: fallback tolerance
        }
        ```
    *   **Response**: JSON object with validation results, including overall pass/fail status and details of any mismatches.

### Configuration Management (If available)

If the simulator is run with configuration management features enabled, the following endpoints may also be available:

*   `GET /config`: Get current full configuration.
*   `GET /config/profiles`: List available configuration profiles.
*   `POST /config/profiles/{profile_name}/load`: Load a named profile.
*   `POST /config/profiles/{profile_name}/save`: Save the current configuration to a named profile.

### Live Parameter Updates (If available)

*   `GET /config/parameters`: List parameters adjustable at runtime.
*   `POST /config/parameters/{parameter_name}`: Update a runtime parameter.
    *   **Request Body** (Pydantic model `ParameterUpdatePayload`):
        ```json
        {
            "value": 5.0
        }
        ```

## Example Scripts

Two example Python scripts are provided in the `examples/` directory to demonstrate how to interact with this API:

*   **`examples/http_debug_api_status_example.py`**: Shows how to fetch status information.
*   **`examples/http_debug_api_command_example.py`**: Demonstrates command injection and state validation.

These scripts typically use the `requests` library. If you don't have it installed, you can get it via pip:
`pip install requests`

## Notes

- The API is intended for debugging and testing purposes.
- Ensure the simulator is running before attempting to connect with an HTTP client.
- Refer to the live `/docs` on your running simulator instance for the most up-to-date and detailed API specification.
