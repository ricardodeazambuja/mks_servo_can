# Troubleshooting Guide

This guide provides solutions and suggestions for common issues encountered when working with the `mks-servo-can` library, MKS servo motors, and the `mks-servo-simulator`.

## 1. Connection Issues

### a. Cannot Connect to Real Hardware (`CANError`)

* **Symptom:** `exceptions.CANError` is raised during `can_interface.connect()`.
* **Possible Causes & Solutions:**
    * **CAN Adapter Not Detected:**
        * Ensure the CAN adapter is securely connected to your computer (USB, etc.).
        * Verify drivers for the CAN adapter are correctly installed and loaded by your OS.
        * Check `dmesg` (Linux) or Device Manager (Windows) to see if the adapter is recognized.
    * **Incorrect `interface_type` or `channel`:**
        * Double-check the `interface_type` (e.g., `socketcan`, `canable`, `pcan`, `kvaser`, `serial`) and `channel` (e.g., `can0`, `slcan0`, `/dev/ttyUSB0`, `COM3`) passed to `CANInterface`. These are specific to your adapter and OS. Refer to the [python-can documentation](https://python-can.readthedocs.io/en/stable/interfaces.html).
        * For `socketcan` on Linux: Ensure the interface is up (e.g., `sudo ip link set can0 up type can bitrate 500000`).
        * For serial-based adapters (like CANable with `slcan`): Ensure the correct serial port is specified and no other application is using it. `slcand` might need to be running for some setups.
    * **Bitrate Mismatch:**
        * The `bitrate` in `CANInterface` (default 500000) **must** match the bitrate set on all MKS servo motors on the bus. Verify motor DIP switch settings or configured parameters.
    * **CAN Bus Wiring Issues:**
        * Check CAN_H and CAN_L wiring continuity.
        * Ensure twisted-pair cabling is used.
        * Verify proper bus termination: A 120 Ohm resistor is required at **each physical end** of the CAN bus. MKS motors often have built-in selectable termination. Only enable it on the two end devices.
    * **Motor Power:**
        * Ensure all MKS servo motors are powered on with the correct voltage.
    * **`python-can` Not Installed:**
        * Although this might also raise `ConfigurationError`, ensure `python-can` is installed in your environment (`pip show python-can`).

### b. Cannot Connect to Simulator (`SimulatorError`)

* **Symptom:** `exceptions.SimulatorError` is raised during `can_interface.connect()` when `use_simulator=True`.
* **Possible Causes & Solutions:**
    * **Simulator Not Running:**
        * Ensure you have started the `mks-servo-simulator` CLI tool in a separate terminal. See [Running the Simulator](../getting_started/running_simulator.md).
    * **Incorrect Host/Port:**
        * Verify the `simulator_host` (default `localhost`) and `simulator_port` (default `6789`) in `CANInterface` match how the simulator was started.
    * **Firewall Issues:**
        * A local firewall might be blocking the TCP connection. Check your firewall settings for Python or the specified port.
    * **Network Configuration:**
        * If running the simulator on a different machine or in a container, ensure network connectivity and correct IP/hostname.

### c. `ConfigurationError` on Connection

* **Symptom:** `exceptions.ConfigurationError` is raised.
* **Possible Causes & Solutions:**
    * `python-can` missing when `use_simulator=False`. Install it: `pip install python-can`.
    * Required parameters like `interface_type` or `channel` not provided for hardware mode.

## 2. Motor Communication Issues (after successful connection)

### a. `MotorTimeoutError` (Motor Not Responding)

* **Symptom:** A command to an `Axis` (e.g., `ping()`, `move_absolute()`) raises `exceptions.MotorTimeoutError`.
* **Possible Causes & Solutions:**
    * **Incorrect Motor CAN ID:**
        * Verify the `can_id` used when creating the `Axis` object matches the ID set on the physical motor (DIP switches or configured parameter).
    * **Motor Powered Off / Disconnected:**
        * Ensure the specific motor is powered and still connected to the CAN bus.
    * **CAN Bus Issues (Post-Connection):**
        * Intermittent wiring problems, loose connectors, or bus errors can develop. Check bus health.
    * **Duplicate CAN IDs:**
        * Ensure no two devices on the CAN bus (including other motors or devices) share the same CAN ID. This can cause unpredictable communication.
    * **Timeout Value Too Short:**
        * If the operation is complex or the bus is very latent, the default timeout might be too short. Try increasing `default_timeout_ms` on the `Axis` or the per-command `timeout_ms`.
    * **Motor in Severe Error State:**
        * Some motor errors might prevent it from responding. Try power cycling the motor.

### b. `MotorOperationError` (Motor Responds but Rejects Command)

* **Symptom:** The motor responds, but the command is rejected, raising `exceptions.MotorOperationError`.
* **Possible Causes & Solutions:**
    * **Motor Not Enabled:**
        * Most movement commands require the motor to be enabled first (`await axis.enable()`).
    * **Invalid Command Parameters:**
        * Values for position, speed, or other parameters might be outside the motor's acceptable range. Consult the MKS motor manual.
    * **Motor Internal Error State:**
        * The motor might have an active error (e.g., stall, over-voltage, over-temperature).
        * Read the motor's error code: `await axis.update_status()` then check `axis.error_code` and `axis.get_error_description()`.
        * Some errors need to be cleared (e.g., via `axis.write_parameter(const.PARAM_SYSTEM_COMMAND, const.SYSTEM_COMMAND_CLEAR_ERROR)`) before normal operation can resume.
    * **Command Not Applicable in Current Mode:**
        * Certain commands may only be valid in specific motor operating modes.

### c. `CRCError` (Data Corruption)

* **Symptom:** `exceptions.CRCError` is raised, indicating a received message failed its CRC check.
* **Possible Causes & Solutions:**
    * **CAN Bus Noise:**
        * Electrical noise can corrupt messages. Ensure proper shielding, grounding, and routing of CAN cables away from noise sources (like motor power lines).
    * **Faulty Wiring/Connectors:**
        * Check for loose connections, damaged cables, or poor-quality connectors.
    * **Improper Termination:**
        * Incorrect or missing termination can cause reflections and signal integrity issues.
    * **Hardware Issue:**
        * Rarely, a fault in the CAN adapter or the motor's CAN transceiver.
    * **CRC Disabled on One End:** 
        * If CRC is handled differently by another device on the bus or if settings are mismatched (though the library aims for MKS compatibility).

## 3. Movement Issues

### a. Motor Doesn't Move

* **Checklist:**
    1. Is the `CANInterface` connected successfully?
    2. Is the `Axis` object initialized with the correct `can_id`?
    3. Can you `ping()` the motor successfully?
    4. Is the motor **enabled** (`await axis.enable()`)? Check `axis.is_enabled()`.
    5. Is the motor free of critical errors? Check `axis.error_code` after `await axis.update_status()`.
    6. Are you providing valid speed and position/distance values to movement commands?
    7. If using kinematics, are the kinematics parameters (`steps_per_revolution`, `pitch`, `gear_ratio`) correct for your mechanical setup? An incorrect kinematics setup can result in tiny (imperceptible) or excessively large (hitting limits) target step counts.
    8. Is the motor mechanically free to move (not jammed)?
    9. Is the power supply adequate for the motor's current draw during movement?

### b. Motor Moves Unexpectedly (Wrong Distance, Direction, Speed)

* **Possible Causes & Solutions:**
    * **Kinematics Configuration:**
        * This is the most common cause. Double-check all parameters in your `RotaryKinematics` or `LinearKinematics` object (or custom kinematics).
        * Ensure `steps_per_revolution` matches the motor's native encoder pulses (e.g., 16384 for MKS SERVO42D/57D).
        * Verify `gear_ratio` and `pitch` (for linear) accurately reflect your mechanics.
    * **Unit Mismatch:**
        * Ensure the units you think you're commanding (e.g., mm, degrees) match the `units` string in your kinematics object and your understanding of the `pitch` or `gear_ratio`.
    * **`use_kinematics` Flag:**
        * If you accidentally pass `use_kinematics=False` to a movement command, position/speed values will be interpreted as raw steps, leading to very different movement.
    * **Coordinate System / Sign Convention:**
        * Ensure your application's definition of positive/negative direction matches the motor's behavior or your kinematics setup. You might need to invert signs in your commands or adjust kinematics.
    * **Motor Parameters:**
        * Check motor's configured max speed, acceleration. If your commanded speed is too high, the motor might cap it.
    * **Relative vs. Absolute Moves:**
        * Ensure you're using `move_relative` when you intend to move *by* an amount, and `move_absolute` when you intend to move *to* a specific coordinate.

### c. Jerky Movement or Stalling

* **Possible Causes & Solutions:**
    * **PID Tuning:** The motor's internal PID controller might need tuning for your specific load and mechanics. This is typically done via MKS configuration software.
    * **Mechanical Issues:** Binding, excessive friction, or a load too heavy for the motor.
    * **Power Supply Issues:** Insufficient current capacity can cause voltage drops under load, leading to poor performance or resets.
    * **Low Acceleration/Deceleration Settings:** If accel/decel values are too low, moves might appear sluggish. If too high for the system, it can cause jerks or missed steps (though closed-loop servos like MKS try to compensate).
    * **CAN Bus Errors:** Frequent CRC errors or message loss can disrupt smooth command flow.

## 4. Simulator Issues

### a. Simulator Doesn't Reflect Expected Behavior

* **Possible Causes & Solutions:**
    * **Simulator Model Limitations:** The simulator models basic motor behavior but may not perfectly replicate all nuances of real hardware (e.g., complex PID responses, dynamic load effects).
    * **Latency Setting:** The `--latency-ms` setting in the simulator introduces a fixed delay. If your application is sensitive to this, it might behave differently than with real hardware where latency is variable.
    * **Parameters:** Ensure the simulator's motor parameters (like `--steps-per-rev`) match your expectations or the real hardware you're trying to simulate.
    * **Bugs:** While efforts are made for correctness, the simulator itself could have bugs.

## General Debugging Tips

* **Enable Logging:**
    * The `mks-servo-can` library uses Python's `logging` module. Increase verbosity to see more details about CAN messages and internal operations:
        ```python
        import logging
        logging.basicConfig(level=logging.DEBUG) 
        # Or for specific library loggers:
        # logging.getLogger("mks_servo_can.can_interface").setLevel(logging.DEBUG)
        # logging.getLogger("mks_servo_can.axis").setLevel(logging.DEBUG)
        ```
* **Simplify:** Start with the simplest possible setup (e.g., one motor, basic commands like `ping` and `enable`).
* **Use Examples:** Refer to the scripts in the `examples/` directory.
* **Check `axis.error_code`:** After any failed operation or unexpected behavior, call `await axis.update_status()` and inspect `axis.error_code` along with `axis.get_error_description()`.
* **Consult MKS Manual:** The official MKS SERVO42D/57D_CAN User Manual is the ultimate reference for motor parameters and error codes.

If you encounter an issue not covered here, consider opening an issue on the project's GitHub repository with detailed information about your setup, code, and the problem observed.