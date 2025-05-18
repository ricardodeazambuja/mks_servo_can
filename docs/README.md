# MKS Servo CAN Control Project - Documentation

Welcome to the documentation for the MKS Servo CAN Control Project. This section aims to provide comprehensive information to help you understand, use, and contribute to the `mks-servo-can` library and the `mks-servo-simulator`.

## Table of Contents

1.  **Introduction**
    * [Project Overview](introduction/project_overview.md)
    * [Key Features](introduction/key_features.md)
    * [Target Audience](introduction/target_audience.md)

2.  **Getting Started**
    * [Prerequisites](getting_started/prerequisites.md)
    * [Installation Guide](getting_started/installation.md) (Covering library and simulator)
    * [Basic Setup for Hardware](getting_started/hardware_setup.md) (CAN interface, wiring, termination)
    * [Running the Simulator](getting_started/running_simulator.md)

3.  **User Guides**
    * **`mks-servo-can` Library Usage**
        * [Connecting to Motors (Hardware & Simulator)](user_guides/library/connecting.md) (`CANInterface`)
        * [Basic Motor Control](user_guides/library/basic_control.md) (`Axis` class: enable, disable, ping)
        * [Movement Commands](user_guides/library/movement.md) (Relative, Absolute, Speed Mode)
        * [Using Kinematics](user_guides/library/kinematics.md) (Linear, Rotary, Custom)
        * [Reading Motor Status & Parameters](user_guides/library/reading_status.md)
        * [Working with Multiple Axes](user_guides/library/multi_axis.md) (`MultiAxisController`)
        * [Controlling Robot Models](user_guides/library/robot_control.md) (Using `robot_kinematics.py` models like `CartesianRobot`, `TwoLinkArmPlanar`, `RRRArm`)
        * [Error Handling & Exceptions](user_guides/library/error_handling.md)
        * [Advanced Configuration](user_guides/library/advanced_config.md) (e.g., timeouts, motor parameters)
    * **`mks-servo-simulator` CLI Usage**
        * [Command-Line Options](user_guides/simulator/cli_options.md)
        * [Interpreting Simulator Logs](user_guides/simulator/logs.md)
        * [Simulating Latency and Multiple Motors](user_guides/simulator/advanced_simulation.md)

4.  **API Reference**
    * **`mks_servo_can` (Library)**
        * [`can_interface.py`](api_reference/library/can_interface.md)
        * [`low_level_api.py`](api_reference/library/low_level_api.md)
        * [`axis.py`](api_reference/library/axis.md)
        * [`multi_axis_controller.py`](api_reference/library/multi_axis_controller.md)
        * [`kinematics/`](api_reference/library/kinematics.md) (Covering base and specific kinematics classes)
        * [`robot_kinematics.py`](api_reference/library/robot_kinematics.md) (Covering `RobotModelBase` and specific robot models)
        * [`constants.py`](api_reference/library/constants.md)
        * [`exceptions.py`](api_reference/library/exceptions.md)
    * **`mks_simulator` (Simulator Internals - for contributors)**
        * [`cli.py`](api_reference/simulator/cli.md)
        * [`motor_model.py`](api_reference/simulator/motor_model.md)
        * [`virtual_can_bus.py`](api_reference/simulator/virtual_can_bus.md)

5.  **Tutorials & Examples**
    * [Controlling a Single Axis (Simulator)](tutorials/single_axis_sim.md)
    * [Controlling a Single Axis (Real Hardware)](tutorials/single_axis_hw.md)
    * [Synchronizing Multiple Axes](tutorials/multi_axis_sync.md)
    * [Implementing Custom Kinematics](tutorials/custom_kinematics.md)
    * [Controlling a 2-Link Planar Arm (`TwoLinkArmPlanar`)](tutorials/two_link_planar_arm_example.md)
    * [Controlling a 3-DOF Cartesian Robot (`CartesianRobot`)](tutorials/cartesian_robot_example.md)
    * [Controlling a 3-DOF RRR Arm (`RRRArm`)](tutorials/rrr_arm_example.md)
    * (General link to `examples/` directory scripts with explanations)

6.  **Advanced Topics**
    * [CAN Protocol Details (MKS Specifics)](advanced_topics/can_protocol.md)
    * [Timing and Determinism](advanced_topics/timing_determinism.md)
    * [Troubleshooting Common Issues](advanced_topics/troubleshooting.md)

7.  **Development & Contribution**
    * [Setting up Development Environment](development/setup.md)
    * [Coding Standards & Style Guide](development/coding_standards.md)
    * [Running Tests](development/running_tests.md)
    * [How to Contribute](development/contributing.md)
    * [Project Roadmap (Future Plans)](development/roadmap.md)

8.  **Appendices**
    * [Glossary of Terms](appendices/glossary.md)
    * [MKS Servo Motor Parameter Reference (Summary)](appendices/mks_parameters.md)

## How to Use This Documentation

* Start with the **Getting Started** section if you are new to the project.
* Refer to the **User Guides** for practical instructions on using the library and simulator.
* Consult the **API Reference** for detailed information on specific modules, classes, and functions.
* Explore **Tutorials & Examples** for step-by-step walkthroughs of common use cases.

## Contributing to Documentation

Documentation improvements are always welcome! If you find errors, omissions, or areas that could be clearer, please feel free to open an issue or submit a pull request.

---

*This `README.md` serves as the main entry point for the detailed documentation. Each linked section above should ideally correspond to a separate Markdown file within the `docs/` directory structure.*