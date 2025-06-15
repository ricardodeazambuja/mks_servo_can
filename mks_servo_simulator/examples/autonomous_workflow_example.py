"""
Example script demonstrating an autonomous workflow using the MKSSimulatorClient
to interact with the MKS Servo Simulator's HTTP Debug API.

This script showcases:
- Connecting to the simulator.
- Fetching system and motor status.
- Injecting a command to a motor.
- Observing state changes.
- Validating an expected state.
- Retrieving command history.
- Basic error handling.
"""
import json
import time
import requests # For requests.exceptions.ConnectionError

# Adjust the import path based on your project structure.
# If 'mks_servo_simulator' is in your PYTHONPATH or installed:
from mks_servo_simulator.mks_simulator.interface.sdk_client import MKSSimulatorClient, SimulatorAPIError

# Configuration
SIMULATOR_BASE_URL = "http://localhost:8765"
TARGET_MOTOR_ID = 1  # Example motor ID to interact with

def pretty_print_dict(data: dict, indent: int = 2):
    """Helper to pretty print dictionaries."""
    print(json.dumps(data, indent=indent, default=str)) # default=str for non-serializable items like datetime

def main():
    """
    Main function to execute the autonomous workflow example.
    """
    print(f"--- MKS Servo Simulator: Autonomous Workflow Example ---")
    print(f"Attempting to connect to simulator at: {SIMULATOR_BASE_URL}")

    client = MKSSimulatorClient(SIMULATOR_BASE_URL)

    try:
        # 1. Basic Checks & Info Gathering
        print("\n[Phase 1: Basic Checks & Info Gathering]")

        health = client.get_health()
        print(f"Simulator health: {health.get('status', 'Unknown')}")
        if health.get('status') != "healthy":
            print("Warning: Simulator is not healthy. Proceeding with caution.")

        root_info = client.get_root_info()
        print("Simulator API Information:")
        pretty_print_dict(root_info)

        system_status = client.get_status()
        print("\nInitial System Status:")
        pretty_print_dict(system_status)

        # 2. Motor Interaction Workflow
        print(f"\n[Phase 2: Interacting with Motor {TARGET_MOTOR_ID}]")
        initial_motor_status = client.get_motor_status(TARGET_MOTOR_ID)
        if not initial_motor_status:
            print(f"Error: Motor {TARGET_MOTOR_ID} not found. Aborting motor interaction phase.")
            return

        print(f"\nInitial status for Motor {TARGET_MOTOR_ID}:")
        pretty_print_dict(initial_motor_status)

        # Example: Inject a command to set target speed (e.g., command 0xF6 for MKS Servos)
        # Data bytes for 0xF6: [Mode (0=Speed), Speed_L, Speed_H, Accel_L, Accel_H]
        # Let's set speed to 100 RPM (0x0064) and default acceleration.
        # This is a guess for a common MKS command. Actual commands depend on simulator's implementation.
        target_speed_rpm = 100
        # Speed bytes (little-endian for 16-bit value)
        speed_bytes = [target_speed_rpm & 0xFF, (target_speed_rpm >> 8) & 0xFF]
        # Acceleration (e.g., 1000 RPM/s^2, if applicable, often default or separate command)
        accel_bytes = [0x00, 0x00] # Using default/no specific accel for this example

        # Command: Set Speed Mode (0), Speed = 100 RPM, Accel = default
        # This assumes command 0xF6 is "Set Operating Mode / Parameters"
        # and data format is [mode, speed_L, speed_H, accel_L, accel_H]
        # Mode 0 might be speed control for some servos.
        # This is a placeholder; a real command set would be needed.
        # For a more reliable demo, one might use a simpler "get_status" command via inject_command
        # if it's known to be safe and observable.

        # Let's try a known template if available, or a simple raw command.
        # Assuming "set_target_speed_rpm" template exists and takes an "rpm" argument.
        # If not, we will fall back to a raw command.
        print(f"\nAttempting to set target speed for Motor {TARGET_MOTOR_ID} to {target_speed_rpm} RPM...")
        command_to_try = "set_target_speed_rpm" # Ideal template
        try:
            templates = client.get_command_templates()
            if command_to_try in templates.get("templates", {}):
                print(f"Using template: {command_to_try}")
                # The server endpoint for templates is: {"motor_id": motor_id, "template_name": template_name, "args": args}
                # The client's inject_template_command matches this.
                injection_result = client.inject_template_command(
                    TARGET_MOTOR_ID,
                    command_to_try,
                    args={"rpm": target_speed_rpm} # Args structure depends on template definition
                )
            else:
                print(f"Template '{command_to_try}' not found. Trying raw command 0xF6 (Set Speed).")
                # Data: [Mode=0 (Speed), Speed_L, Speed_H, Accel_L, Accel_H]
                # This is a common MKS command, but might not be active in the simulator by default.
                raw_cmd_data = [0x00] + speed_bytes + accel_bytes
                injection_result = client.inject_command(
                    TARGET_MOTOR_ID,
                    command_code=0xF6, # Example: Set speed/operating mode
                    data_bytes=raw_cmd_data
                )
            print("Command Injection Result:")
            pretty_print_dict(injection_result)
            if not injection_result.get("success", False):
                print(f"Warning: Command injection reported failure: {injection_result.get('message')}")

        except SimulatorAPIError as e:
            print(f"Error during command injection: {e}")
            print("Skipping further motor state checks due to injection error.")
            return # Or continue to other phases if appropriate

        print("\nWaiting for 1 second for simulator to process...")
        time.sleep(1)

        updated_motor_status = client.get_motor_status(TARGET_MOTOR_ID)
        if not updated_motor_status:
            print(f"Error: Could not retrieve updated status for Motor {TARGET_MOTOR_ID}.")
            return

        print(f"\nUpdated status for Motor {TARGET_MOTOR_ID}:")
        pretty_print_dict(updated_motor_status)

        # Compare initial and updated status (simple check)
        # Note: Field names depend on the simulator's actual status output
        initial_speed = initial_motor_status.get("status", {}).get("target_speed_rpm", initial_motor_status.get("target_speed_rpm", 0))
        updated_speed = updated_motor_status.get("status", {}).get("target_speed_rpm", updated_motor_status.get("target_speed_rpm", 0))

        print(f"\nSpeed comparison: Initial Target Speed = {initial_speed}, Updated Target Speed = {updated_speed}")
        if updated_speed == target_speed_rpm:
            print("Motor target speed appears to be updated successfully.")
        else:
            print("Motor target speed may not have updated as expected, or status field differs.")


        # 3. State Validation
        print("\n[Phase 3: Validating State]")
        # This expected state should match what the command was supposed to achieve.
        # The exact key for target_speed_rpm depends on the simulator's get_motor_status output.
        # Let's assume the status dictionary has a top-level 'target_speed_rpm' key after the command.
        # Or it might be nested, e.g., initial_motor_status.get("parameters", {}).get("target_speed_rpm")
        # Adjust the key according to the actual output of get_motor_status.
        # For this example, we'll assume 'target_speed_rpm' is a key in the motor status dict.
        expected_state_payload = {
            "motors": [ # Note: validate_state expects a list of motor checks
                {
                    "id": TARGET_MOTOR_ID,
                    "target_speed_rpm": target_speed_rpm, # Key needs to match actual status output
                    "tolerance": 10 # Allow some tolerance, e.g., +/- 10 RPM
                }
            ]
        }
        print("Validating state with payload:")
        pretty_print_dict(expected_state_payload)

        validation_result = client.validate_state(expected_state_payload)
        print("\nValidation Result:")
        pretty_print_dict(validation_result)
        if validation_result.get("overall_match"):
            print("State validation PASSED.")
        else:
            print("State validation FAILED. Check details above.")

        # 4. Retrieve Command History
        print("\n[Phase 4: Retrieve Command History]")
        history = client.get_command_history(motor_id=TARGET_MOTOR_ID, limit=5)
        print(f"\nLast 5 commands for Motor {TARGET_MOTOR_ID}:")
        pretty_print_dict(history) # The API returns a dict with a 'history' key

        # 5. (Optional) Configuration Example
        print("\n[Phase 5: Configuration Example]")
        profiles = client.list_config_profiles() # This returns a list directly
        print(f"\nAvailable configuration profiles: {profiles}")

        if profiles:
            profile_to_load = profiles[0]
            print(f"Attempting to load profile: {profile_to_load}...")
            try:
                load_result = client.load_config_profile(profile_to_load)
                print(f"Load result for profile '{profile_to_load}':")
                pretty_print_dict(load_result)
            except SimulatorAPIError as e:
                print(f"Error loading profile '{profile_to_load}': {e}")
        else:
            print("No configuration profiles found to demonstrate loading.")

        # 6. Demonstrate Saving a Configuration Profile
        print("\n[Phase 6: Demonstrating Saving Configuration Profile]")
        current_config_summary = client.get_configuration()
        # The 'name' field might not exist if the config is default/unsaved.
        # It's better to save first, then list to confirm.
        profile_name_to_save = f"my_workflow_profile_{int(time.time())}" # Unique name
        print(f"Attempting to save current configuration as '{profile_name_to_save}'...")
        try:
            save_result = client.save_config_profile(profile_name_to_save)
            print("Save Profile Result:")
            pretty_print_dict(save_result)
            if save_result.get("success"):
                print(f"Profile '{profile_name_to_save}' saved successfully.")
                profiles_after_save = client.list_config_profiles()
                print(f"Available profiles after save: {profiles_after_save}")
                if profile_name_to_save in profiles_after_save:
                    print(f"Confirmed '{profile_name_to_save}' is in the list of profiles.")
                    # Optional Cleanup: Delete the profile just saved
                    print(f"\nAttempting to delete profile '{profile_name_to_save}'...")
                    delete_result = client.delete_config_profile(profile_name_to_save)
                    print("Delete Profile Result:")
                    pretty_print_dict(delete_result)
                    if delete_result.get("success"):
                        print(f"Profile '{profile_name_to_save}' deleted successfully.")
                        profiles_after_delete = client.list_config_profiles()
                        print(f"Available profiles after delete: {profiles_after_delete}")
            else:
                print(f"Failed to save profile: {save_result.get('error', 'Unknown reason')}")
        except SimulatorAPIError as e:
            print(f"Error during saving/deleting profile: {e}")


        # 7. Demonstrate Applying a Motor Template
        print("\n[Phase 7: Demonstrating Applying Motor Template]")
        motor_templates_config = client.get_motor_templates_config()
        available_templates = list(motor_templates_config.get("templates", {}).keys())
        print(f"Available motor templates: {available_templates}")

        if available_templates:
            template_to_apply = available_templates[0] # Pick the first one
            motor_id_for_template = TARGET_MOTOR_ID
            print(f"Attempting to apply template '{template_to_apply}' to motor {motor_id_for_template}...")
            try:
                # Get status before applying template
                status_before_template = client.get_motor_status(motor_id_for_template)
                print(f"Motor {motor_id_for_template} status BEFORE applying template:")
                pretty_print_dict(status_before_template)

                apply_result = client.apply_motor_template(template_name=template_to_apply, motor_id=motor_id_for_template)
                print("Apply Template Result:")
                pretty_print_dict(apply_result)

                if apply_result.get("success"):
                    print(f"Template '{template_to_apply}' applied successfully to motor {motor_id_for_template}.")
                    status_after_template = client.get_motor_status(motor_id_for_template)
                    print(f"Motor {motor_id_for_template} status AFTER applying template:")
                    pretty_print_dict(status_after_template)
                    # Further checks could compare status_before_template and status_after_template
                else:
                    print(f"Failed to apply template: {apply_result.get('error', 'Unknown reason')}")
            except SimulatorAPIError as e:
                print(f"Error applying motor template: {e}")
        else:
            print("No motor templates available to demonstrate application.")


        # 8. Demonstrate Updating a Live Parameter
        print("\n[Phase 8: Demonstrating Updating Live Parameter]")
        param_to_update = "latency_ms" # A common example parameter

        try:
            # Get initial value from current configuration
            initial_config = client.get_configuration()
            initial_value = initial_config.get("simulation_parameters", {}).get(param_to_update)
            # Fallback if not in simulation_parameters, check top level (adjust based on actual config structure)
            if initial_value is None:
                 initial_value = initial_config.get(param_to_update)

            if initial_value is not None:
                print(f"Initial value of '{param_to_update}': {initial_value}")
                new_value = float(initial_value) + 2.5 if isinstance(initial_value, (int, float)) else 5.5 # Example new value

                print(f"Attempting to update '{param_to_update}' to {new_value}...")
                update_result = client.update_live_parameter(param_to_update, new_value)
                print("Update Parameter Result:")
                pretty_print_dict(update_result)

                if update_result.get("success"):
                    print(f"Parameter '{param_to_update}' updated successfully.")
                    config_after_update = client.get_configuration()
                    updated_value = config_after_update.get("simulation_parameters", {}).get(param_to_update)
                    if updated_value is None:
                        updated_value = config_after_update.get(param_to_update)

                    print(f"Value of '{param_to_update}' after update: {updated_value}")
                    if updated_value is not None and abs(updated_value - new_value) < 0.01: # Compare floats with tolerance
                        print(f"Successfully verified change of '{param_to_update}'.")
                    else:
                        print(f"Verification of '{param_to_update}' change failed or value not found at expected key.")

                    # Optional: Revert the change
                    print(f"Attempting to revert '{param_to_update}' back to {initial_value}...")
                    revert_result = client.update_live_parameter(param_to_update, initial_value)
                    print("Revert Parameter Result:")
                    pretty_print_dict(revert_result)
                    final_config = client.get_configuration()
                    final_value = final_config.get("simulation_parameters", {}).get(param_to_update)
                    if final_value is None:
                        final_value = final_config.get(param_to_update)
                    print(f"Value of '{param_to_update}' after reverting: {final_value}")

                else:
                    print(f"Failed to update parameter: {update_result.get('message', 'Unknown reason')}")
            else:
                print(f"Could not retrieve initial value for '{param_to_update}' from configuration. Skipping update demonstration.")
        except SimulatorAPIError as e:
            print(f"Error during live parameter update: {e}")
        except Exception as e: # Catch other potential errors like float conversion
            print(f"An unexpected error occurred during parameter update demo: {e}")


        print("\n--- Autonomous Workflow Example Completed ---")

    except SimulatorAPIError as e:
        print(f"\nAPI Error: {e}")
        if e.response_data:
            print("Error Response Data:")
            pretty_print_dict(e.response_data)
    except requests.exceptions.ConnectionError:
        print(f"\nConnection Error: Could not connect to the simulator at {SIMULATOR_BASE_URL}.")
        print("Please ensure the MKS Servo Simulator is running and accessible.")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")

if __name__ == "__main__":
    main()
