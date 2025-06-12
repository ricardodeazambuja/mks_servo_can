# Running the Simulator

The `mks-servo-simulator` is a command-line tool that emulates the behavior of MKS servo motors, allowing you to develop and test the `mks-servo-can` library without physical hardware.

## Prerequisites

* Ensure you have installed the `mks-servo-simulator` package. If not, refer to the [Installation Guide](./installation.md).
* The simulator uses `click`, which should have been installed as a dependency. [cite: mks_servo_can/requirements.txt]

## Starting the Simulator

To run the simulator, open your terminal and use the `mks-servo-simulator` command.

**Basic Usage:**

`mks-servo-simulator`

This will start the simulator with default settings:

* 1 simulated motor. [cite: mks_servo_can/mks_servo_simulator/mks_simulator/cli.py]
* Starting CAN ID of 1. [cite: mks_servo_can/mks_servo_simulator/mks_simulator/cli.py]
* Simulator listening on `localhost:6789`. [cite: mks_servo_can/mks_servo_simulator/mks_simulator/cli.py]
* Default simulated CAN bus latency (e.g., 2.0 ms round trip). [cite: mks_servo_can/mks_servo_simulator/mks_simulator/cli.py]

**Common Options:**

You can customize the simulator's behavior using command-line options. Here are some of the most common ones:

* **`--num-motors <count>`:** Specify the number of motors to simulate.
  Example: `mks-servo-simulator --num-motors 3`

* **`--start-can-id <id>`:** Set the CAN ID for the first simulated motor. Subsequent motors will have incrementing IDs.
  Example: `mks-servo-simulator --num-motors 2 --start-can-id 10`
  (This would simulate motors with CAN IDs 10 and 11).

* **`--host <hostname>`:** Set the host address for the simulator to listen on.
  Example: `mks-servo-simulator --host 0.0.0.0`
  (To listen on all available network interfaces).

* **`--port <port_number>`:** Set the TCP port for the simulator.
  Example: `mks-servo-simulator --port 12345`

* **`--latency-ms <milliseconds>`:** Simulate round-trip CAN bus latency in milliseconds. This helps mimic real-world communication delays.
  Example: `mks-servo-simulator --latency-ms 5.0`

* **`--log-level <level>`:** Set the logging level for the simulator (DEBUG, INFO, WARNING, ERROR).
  Example: `mks-servo-simulator --log-level DEBUG`

* **`--steps-per-rev <steps>`:** Set the encoder steps per revolution for the simulated motors (default is 16384, matching `ENCODER_PULSES_PER_REVOLUTION`). [cite: mks_servo_can/mks_servo_simulator/mks_simulator/cli.py] [cite: mks_servo_can/mks_servo_simulator/mks_simulator/motor_model.py]
  Example: `mks-servo-simulator --steps-per-rev 8000`

**Example: Running a simulator for two motors (IDs 1 and 2) with 5ms latency:**

`mks-servo-simulator --num-motors 2 --start-can-id 1 --latency-ms 5.0`
[cite: mks_servo_can/README.md]

**Viewing All Options:**

To see all available command-line options and their descriptions, use the `--help` flag:

`mks-servo-simulator --help`

## Simulator Output

When the simulator is running, it will output log messages to the console. These messages indicate:

* The simulator's configuration (host, port, number of motors, etc.).
* When a client (your `mks-servo-can` library instance) connects or disconnects.
* The CAN messages it receives from the client.
* The responses it sends back to the client.
* Internal state changes of the simulated motors.

This output is useful for debugging your control application and understanding the interaction between the library and the (simulated) motors.

Example log output:
```text
INFO:MKSSimulatorCLI:Starting MKS Servo CAN Simulator...
INFO:MKSSimulatorCLI:Config: Host=localhost, Port=6789, NumMotors=1, StartID=1, Type=GENERIC, Latency=2.0ms
INFO:VirtualCANBus:Added simulated motor with CAN ID 001 to virtual bus.
INFO:SimulatedMotor:SimulatedMotor CAN ID 001 initialized. Pos: 0.0 steps. Work Mode: 5. Enabled: False
INFO:VirtualCANBus:Virtual CAN Bus server listening on ('127.0.0.1', 6789)
INFO:SimulatedMotor:SimulatedMotor 1 update task started.
INFO:VirtualCANBus:Starting all simulated motors...
INFO:VirtualCANBus:All simulated motors have been issued a start command.
INFO:MKSSimulatorCLI:Simulator server running. Press Ctrl+C to stop.
INFO:VirtualCANBus:Client ('127.0.0.1', <client_port>) connected to virtual CAN bus.
INFO:CANInterface: Sending to simulator: SIM_CAN_SEND 001 2 3132
INFO:SimulatedMotor:Motor 1: Processing CMD 0x31, Data:
INFO:VirtualCANBus:VirtualBus sending to lib: SIM_CAN_RECV 001 8 3100000000000032
```

## Stopping the Simulator
To stop the simulator, press `Ctrl+C` in the terminal where it is running. The simulator is designed to perform a graceful shutdown, stopping its internal tasks and closing connections.

## Connecting to the Simulator

Once the simulator is running, you can connect to it from your Python scripts. The library will automatically detect and connect to a running simulator:

```python
import asyncio
from mks_servo_can import CANInterface, Axis
from mks_servo_can.kinematics import RotaryKinematics

async def test_simulator_connection():
    # CANInterface will auto-detect the running simulator
    can_interface = CANInterface()
    
    try:
        await can_interface.connect()
        print("Connected to simulator!")
        
        # Create and test an axis
        axis = Axis(can_interface, motor_can_id=1, 
                   kinematics=RotaryKinematics())
        await axis.initialize()
        await axis.enable_motor()
        
        # Test movement
        await axis.move_to_position_abs_user(90.0, speed_user=30.0)
        position = await axis.get_current_position_user()
        print(f"Motor position: {position:.2f}°")
        
        await axis.disable_motor()
        
    finally:
        await can_interface.disconnect()

# Run the test
# asyncio.run(test_simulator_connection())
```

## Typical Development Workflow

1. **Start the simulator** in one terminal:
   ```bash
   mks-servo-simulator --num-motors 4 --start-can-id 1 --latency-ms 5
   ```

2. **Run your Python scripts** in another terminal that use the library to connect to the simulator.

3. **Monitor the simulator output** to see the CAN messages and motor state changes.

4. **Stop the simulator** with `Ctrl+C` when done.

## Troubleshooting

### Simulator Won't Start
- Check if another instance is already running on the same port
- Try a different port: `mks-servo-simulator --port 6790`
- Ensure the simulator package is properly installed

### Library Can't Connect to Simulator  
- Verify the simulator is running and shows "Simulator server running"
- Check that no firewall is blocking the connection
- Ensure you're using the correct host/port if not using defaults

### Performance Issues
- Reduce the number of simulated motors if experiencing lag
- Increase latency if your system is slow: `--latency-ms 10`
- Use DEBUG logging to identify bottlenecks: `--log-level DEBUG`

## Next Steps

With the simulator running, proceed to:
* [Connecting to Motors](./connecting.md) to learn how to establish connections
* [Basic Motor Control](../user_guides/library/basic_control.md) to start controlling motors
* [Movement Commands](../user_guides/library/movements.md) to learn about motor movements