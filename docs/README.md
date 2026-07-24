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
        * [Movement Commands](user_guides/library/movement.md) *(planned)* (Relative, Absolute, Speed Mode)
        * [Using Kinematics](user_guides/library/kinematics.md) (Linear, Rotary, Custom)
        * [Reading Motor Status & Parameters](user_guides/library/reading_status.md)
        * [Working with Multiple Axes](user_guides/library/multi_axis.md) (`MultiAxisController`)
        * [Controlling Robot Models](user_guides/library/robot_control.md) *(planned)* (Using `robot_kinematics.py` models like `CartesianRobot`, `TwoLinkArmPlanar`, `RRRArm`)
        * [Error Handling & Exceptions](user_guides/library/error_handling.md)
        * [Advanced Configuration](user_guides/library/advanced_config.md) (e.g., timeouts, motor parameters)
    * **`mks-servo-simulator` CLI Usage**
        * [Command-Line Options](user_guides/simulator/cli_options.md) *(planned)*
        * [Interpreting Simulator Logs](user_guides/simulator/logs.md) *(planned)*
        * [Simulating Latency and Multiple Motors](user_guides/simulator/advanced_simulation.md) *(planned)*
        * [HTTP Debug API Guide](user_guides/http_debug_api.md)

4.  **API Reference**
    * **`mks_servo_can` (Library)**
        * [`can_interface.py`](api_reference/library/can_interface.md) *(planned)*
        * [`low_level_api.py`](api_reference/library/low_level_api.md) *(planned)*
        * [`axis.py`](api_reference/library/axis.md) *(planned)*
        * [`multi_axis_controller.py`](api_reference/library/multi_axis_controller.md) *(planned)*
        * [`kinematics/`](api_reference/library/kinematics.md) *(planned)* (Covering base and specific kinematics classes)
        * [`robot_kinematics.py`](api_reference/library/robot_kinematics.md) (Covering `RobotModelBase` and specific robot models)
        * [`constants.py`](api_reference/library/constants.md) *(planned)*
        * [`exceptions.py`](api_reference/library/exceptions.md) *(planned)*
    * **`mks_simulator` (Simulator Internals - for contributors)**
        * [`cli.py`](api_reference/simulator/cli.md) *(planned)*
        * [`motor_model.py`](api_reference/simulator/motor_model.md) *(planned)*
        * [`virtual_can_bus.py`](api_reference/simulator/virtual_can_bus.md) *(planned)*

5.  **Tutorials & Examples**
    * [CAN on Linux: A Quick Survival Guide](tutorials/survival_guide_can_on_linux.md)
    * [Controlling a Single Axis (Simulator)](tutorials/single_axis_sim.md) *(planned)*
    * [Controlling a Single Axis (Real Hardware)](tutorials/single_axis_hw.md) *(planned)*
    * [Synchronizing Multiple Axes](tutorials/multi_axis_sync.md)
    * [Implementing Custom Kinematics](tutorials/custom_kinematics.md)
    * [Controlling a 2-Link Planar Arm (`TwoLinkArmPlanar`)](tutorials/two_link_planar_arm_example.md) *(planned)*
    * [Controlling a 3-DOF Cartesian Robot (`CartesianRobot`)](tutorials/cartesian_robot_example.md) *(planned)*
    * [Controlling a 3-DOF RRR Arm (`RRRArm`)](tutorials/rrr_arm_example.md) *(planned)*
    * (General link to `examples/` directory scripts with explanations)

6.  **Advanced Topics**
    * [CAN Protocol Details (MKS Specifics)](advanced_topics/can_protocol.md)
    * [Timing and Determinism](advanced_topics/timing_determinism.md)
    * [Troubleshooting Common Issues](advanced_topics/troubleshooting.md)
    * [Asynchronous Control with asyncio](advanced_topics/asynchronous_control_with_asyncio.md)
    * [Using mks-servo-can in Synchronous Applications](advanced_topics/asyncio_with_synchronous_code.md)
    * [Python's Role in Automation: Strengths and Limitations](advanced_topics/python_and_automation.md)

7.  **Development & Contribution**
    * [Setting up Development Environment](development/setup.md) *(planned)*
    * [Coding Standards & Style Guide](development/coding_standards.md) *(planned)*
    * [Running Tests](development/running_tests.md) *(planned)*
    * [How to Contribute](development/contributing.md) *(planned)*
    * [Roadmap — planned work in priority order](development/roadmap.md) (open defects, sequencing, and how to verify each)

8.  **Appendices**
    * [Glossary of Terms](appendices/glossary.md) *(planned)*
    * [MKS Servo Motor Parameter Reference (Summary)](appendices/mks_parameters.md) *(planned)*

## How to Use This Documentation

Entries marked ***(planned)*** describe documents that do not exist yet. The
table of contents above was written as an outline of the intended structure and
most of it was never filled in — 27 of its 52 links pointed at nothing, with no
indication of which. Rather than delete the outline and lose the intent, the
gaps are now labelled. `tests/test_docs_api.py` will fail if a *new* dead link
appears, and the existing ones are tracked as debt in
`tests/fixtures/docs_known_issues.json`.

Be aware that the documents which *do* exist have not all kept pace with the
code: the same test tracks 26 known references to methods and parameters that no
longer exist. Where a document and the code disagree, trust the code, and check
`docs/development/roadmap.md` for what is known to be stale.

* Start with the **Getting Started** section if you are new to the project.
* Refer to the **User Guides** for practical instructions on using the library and simulator.
* Consult the **API Reference** for detailed information on specific modules, classes, and functions.
* Explore **Tutorials & Examples** for step-by-step walkthroughs of common use cases.

## Contributing to Documentation

Documentation improvements are always welcome! If you find errors, omissions, or areas that could be clearer, please feel free to open an issue or submit a pull request.

---

*This `README.md` serves as the main entry point for the detailed documentation. Each linked section above should ideally correspond to a separate Markdown file within the `docs/` directory structure.*