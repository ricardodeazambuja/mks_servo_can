import requests
import json
import time

# Base URL for the MKS Servo Simulator HTTP Debug API
SIMULATOR_BASE_URL = "http://localhost:8765"
MOTOR_ID_TO_TEST = 1 # Default motor ID for this example

def inject_raw_command(motor_id: int, command_code: int, data_bytes: list, expect_response: bool = True):
    """Injects a raw command to a motor and prints the result."""
    payload = {
        "motor_id": motor_id,
        "command_code": command_code,
        "data_bytes": data_bytes,
        "expect_response": expect_response
    }
    response = None  # Initialize response to None
    try:
        print(f"\n--- Injecting Raw Command (Code: 0x{command_code:02X}) to Motor {motor_id} ---")
        print(f"Payload: {json.dumps(payload)}")
        response = requests.post(f"{SIMULATOR_BASE_URL}/inject", json=payload)
        response.raise_for_status()
        result = response.json()
        print("Injection Response:")
        print(json.dumps(result, indent=2))
        return result
    except requests.exceptions.RequestException as e:
        print(f"Error injecting raw command: {e}")
        if response is not None:
            print(f"Server response text: {response.text}")
        return None

def validate_motor_state(motor_id: int, expected_params: dict, tolerance: float = 1.0):
    """Validates the state of a specific motor against expected parameters."""
    # Construct the 'motors' list for the validation payload
    # Each item in the list should be a dictionary for one motor's expected state
    motor_validation_spec = {"id": motor_id, "tolerance": tolerance}
    motor_validation_spec.update(expected_params) # Add specific params to check

    payload = {
        "motors": [motor_validation_spec]
        # Global tolerance can also be set here if not per-motor or per-parameter
        # "tolerance": tolerance
    }
    response = None # Initialize response to None
    try:
        print(f"\n--- Validating State for Motor {motor_id} ---")
        print(f"Validation Payload: {json.dumps(payload)}")
        response = requests.post(f"{SIMULATOR_BASE_URL}/validate", json=payload)
        response.raise_for_status()
        result = response.json()
        print("Validation Result:")
        print(json.dumps(result, indent=2))
        return result
    except requests.exceptions.RequestException as e:
        print(f"Error validating state: {e}")
        if response is not None:
            print(f"Server response text: {response.text}")
        return None

if __name__ == "__main__":
    print(f"Running HTTP Debug API Command and Validation Example for Motor {MOTOR_ID_TO_TEST}...")
    print(f"Targeting simulator at: {SIMULATOR_BASE_URL}\n")

    # 1. Ensure motor is enabled (using raw command 0xF3: Enable Motor. Data: [1] for enable)
    print("Step 1: Ensuring motor is enabled.")
    enable_result = inject_raw_command(MOTOR_ID_TO_TEST, 0xF3, [1])
    if not (enable_result and enable_result.get("success")):
        print(f"Failed to enable motor {MOTOR_ID_TO_TEST}. Exiting example.")
        exit()
    time.sleep(0.5) # Give a moment for the state to update

    # 2. Inject the "Set Target Speed" command
    # Command 0xF6: Set Speed Closed Loop. Data: Speed (4 bytes, LSB first, signed int32)
    target_speed_rpm = 100
    # Convert speed to little-endian byte array (4 bytes)
    speed_data_bytes = list(target_speed_rpm.to_bytes(4, byteorder='little', signed=True))

    print(f"\nStep 2: Setting target speed to {target_speed_rpm} RPM.")
    command_result = inject_raw_command(MOTOR_ID_TO_TEST, 0xF6, speed_data_bytes)

    if command_result and command_result.get("success"):
        print(f"Command to set speed to {target_speed_rpm} RPM was successful.")
        time.sleep(0.2) # Short delay for simulator to process

        # 3. Validate that the target speed was set
        # The key in _get_motor_status for target speed is 'target_speed_rpm'
        print("\nStep 3: Validating target speed.")
        validation_params_target_speed = {"target_speed_rpm": float(target_speed_rpm)}
        validate_motor_state(MOTOR_ID_TO_TEST, validation_params_target_speed, tolerance=1.0)

        # Optional: Validate current_speed_rpm after some time
        # print("\nStep 3a: Validating current speed (may take time to ramp up).")
        # time.sleep(1) # Wait longer for current speed to potentially match target
        # validation_params_current_speed = {"current_speed_rpm": float(target_speed_rpm)}
        # validate_motor_state(MOTOR_ID_TO_TEST, validation_params_current_speed, tolerance=15.0) # Wider tolerance

    else:
        print("Failed to inject set speed command or command reported failure.")

    # 4. Stop the motor
    # Command 0xF7: Stop Motor (Immediate Stop). Data: []
    print("\nStep 4: Stopping the motor.")
    inject_raw_command(MOTOR_ID_TO_TEST, 0xF7, [])
    time.sleep(0.5) # Delay for state to update

    print("\nStep 5: Validating motor is stopped (target speed is 0).")
    validation_params_stopped_target = {"target_speed_rpm": 0.0}
    validate_motor_state(MOTOR_ID_TO_TEST, validation_params_stopped_target, tolerance=1.0)

    # Also check current speed is close to 0 after stop
    # print("\nStep 5a: Validating current speed is near zero.")
    # validation_params_stopped_current = {"current_speed_rpm": 0.0}
    # validate_motor_state(MOTOR_ID_TO_TEST, validation_params_stopped_current, tolerance=10.0) # Wider tolerance

    print("\nCommand and validation example finished.")
