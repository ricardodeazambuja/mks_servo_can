import requests
import json

# Base URL for the MKS Servo Simulator HTTP Debug API
SIMULATOR_BASE_URL = "http://localhost:8765"

def get_api_info():
    """Fetches and prints basic API information and endpoints."""
    try:
        response = requests.get(f"{SIMULATOR_BASE_URL}/")
        response.raise_for_status()  # Raise an exception for HTTP errors
        print("--- API Information ---")
        print(json.dumps(response.json(), indent=2))
    except requests.exceptions.RequestException as e:
        print(f"Error fetching API info: {e}")

def get_health_status():
    """Fetches and prints the health status of the simulator."""
    try:
        response = requests.get(f"{SIMULATOR_BASE_URL}/health")
        response.raise_for_status()
        print("\n--- Health Status ---")
        print(json.dumps(response.json(), indent=2))
    except requests.exceptions.RequestException as e:
        print(f"Error fetching health status: {e}")

def get_system_status():
    """Fetches and prints the overall system status."""
    try:
        response = requests.get(f"{SIMULATOR_BASE_URL}/status")
        response.raise_for_status()
        status_data = response.json()
        print("\n--- System Status ---")
        print(f"Timestamp: {status_data.get('timestamp')}")
        print(f"Uptime (seconds): {status_data.get('uptime_seconds')}")
        print(f"Simulator Status: {status_data.get('simulator_status')}")
        print(f"Motor Count: {status_data.get('motor_count')}")
        # print(json.dumps(status_data, indent=2)) # Optionally print full status
        return status_data
    except requests.exceptions.RequestException as e:
        print(f"Error fetching system status: {e}")
        return None

def get_motor_details(motor_id: int):
    """Fetches and prints detailed status for a specific motor."""
    try:
        response = requests.get(f"{SIMULATOR_BASE_URL}/motors/{motor_id}")
        response.raise_for_status()
        print(f"\n--- Motor {motor_id} Details ---")
        print(json.dumps(response.json(), indent=2))
    except requests.exceptions.RequestException as e:
        print(f"Error fetching status for motor {motor_id}: {e}")

if __name__ == "__main__":
    print("Querying MKS Servo Simulator HTTP Debug API (Status Example)...")
    print(f"Targeting simulator at: {SIMULATOR_BASE_URL}\n")

    get_api_info()
    get_health_status()
    system_status = get_system_status()

    if system_status and system_status.get("motors"):
        motor_ids = list(system_status["motors"].keys())
        if motor_ids:
            try:
                # Attempt to get details for the first motor ID found.
                # Motor IDs in the status response are strings by default from JSON keys.
                first_motor_id_str = motor_ids[0]
                first_motor_id_int = int(first_motor_id_str)
                get_motor_details(first_motor_id_int)
            except ValueError:
                print(f"Could not parse motor ID: {motor_ids[0]} as an integer.")
            except KeyError:
                print("Could not find any motor IDs in system_status['motors'].")
        else:
            print("\nNo motors listed in system status.")
    elif system_status:
        print("\nNo motors found in system status object.")
    else:
        print("\nCould not retrieve system status.")

    print("\nStatus example script finished.")
