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
        * [Movement Commands](user_guides/library/movements.md) (Relative, Absolute, Speed Mode)
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
        * [HTTP Debug API Guide](user_guides/http_debug_api.md)

4.  **API Reference**

    **The docstrings are the API reference.** Every public module, class and
    function carries a Google-style docstring with `Args:`, `Returns:` and
    `Raises:`, and many carry the *reason* a decision was made — which reading
    of the MKS manual was chosen, and why. A prose copy of a signature drifts
    away from the code within a release; this project had eight such pages
    planned and none written, and the pages that did exist accumulated 26
    references to methods that no longer existed. So rather than restate them,
    read them where they are — `help(Axis.move_to_position_abs_user)` in a REPL,
    or the module itself:

    * **`mks_servo_can` (Library)** — source under [`mks_servo_can_library/mks_servo_can/`](../mks_servo_can_library/mks_servo_can/)
        * [`can_interface.py`](../mks_servo_can_library/mks_servo_can/can_interface.py) — the connection, hardware or simulator
        * [`low_level_api.py`](../mks_servo_can_library/mks_servo_can/low_level_api.py) — one method per manual command
        * [`axis.py`](../mks_servo_can_library/mks_servo_can/axis.py) — one motor, in user units
        * [`multi_axis_controller.py`](../mks_servo_can_library/mks_servo_can/multi_axis_controller.py) — named axes moved as a group
        * [`kinematics/`](../mks_servo_can_library/mks_servo_can/kinematics/) — steps ↔ user units
        * [`robot_kinematics.py`](api_reference/library/robot_kinematics.md) — `RobotModelBase` and the robot models *(prose reference; the models are geometric contracts worth stating once in full)*
        * [`constants.py`](../mks_servo_can_library/mks_servo_can/constants.py) — command codes, work modes, limits, each annotated with its manual section
        * [`exceptions.py`](../mks_servo_can_library/mks_servo_can/exceptions.py) — the error hierarchy
        * [`data/manual_commands_v106.json`](../mks_servo_can_library/mks_servo_can/data/) — the transcription of the manual itself, including the `errata` block recording where it is ambiguous
    * **`mks_simulator` (Simulator Internals - for contributors)** — source under [`mks_servo_simulator/mks_simulator/`](../mks_servo_simulator/mks_simulator/)
        * [`cli.py`](../mks_servo_simulator/mks_simulator/cli.py) — options are documented in [Command-Line Options](user_guides/simulator/cli_options.md) and by `mks-servo-simulator --help`
        * [`motor_model.py`](../mks_servo_simulator/mks_simulator/motor_model.py) — the simulated motor
        * [`virtual_can_bus.py`](../mks_servo_simulator/mks_simulator/virtual_can_bus.py) — the socket server and frame routing

5.  **Tutorials & Examples**
    * [CAN on Linux: A Quick Survival Guide](tutorials/survival_guide_can_on_linux.md)
    * [Synchronizing Multiple Axes](tutorials/multi_axis_sync.md)
    * [Implementing Custom Kinematics](tutorials/custom_kinematics.md)

    For everything else, run the script. [`examples/`](../examples/) holds
    working programs that are linted and kept in step with the library; a prose
    walkthrough of the same code is one more thing to drift, and five of them
    were outlined here and never written.

    | want to | run |
    |---|---|
    | control one axis against the simulator | [`multi_axis_simulator.py`](../examples/multi_axis_simulator.py), [`comprehensive_simulator_test.py`](../examples/comprehensive_simulator_test.py) |
    | control one axis on real hardware | [`single_axis_real_hw.py`](../examples/single_axis_real_hw.py) |
    | drive the printed pan/tilt gimbal by hand | [`gimbal_cli.py`](../examples/gimbal_cli.py), and [the build](../hardware/gimbal/README.md) |
    | drive a 2-link planar arm (`TwoLinkArmPlanar`) | [`two_link_planar_arm.py`](../examples/two_link_planar_arm.py) |
    | drive a 3-DOF Cartesian robot (`CartesianRobot`) | [`cartesian_3dof_robot.py`](../examples/cartesian_3dof_robot.py) |
    | drive a 3-DOF RRR arm (`RRRArm`) | [`three_link_arm.py`](../examples/three_link_arm.py) |
    | use the library from synchronous code | [`sync_mks_axis.py`](../examples/sync_mks_axis.py) |
    | record and replay a motion | [`motor_digitizer.py`](../examples/motor_digitizer.py), and [its guide](../examples/README_Motor_Digitizer.md) |
    | follow a Cartesian path | [`calligraphy_plotter_manual_interpolation.py`](../examples/calligraphy_plotter_manual_interpolation.py), and [the plotter notes](../examples/svg_plotter_documentation.md) |
    | drive the HTTP debug API | [`http_debug_api_status_example.py`](../examples/http_debug_api_status_example.py), [`http_debug_api_command_example.py`](../examples/http_debug_api_command_example.py) |

    Most of these need a simulator running first — `mks-servo-simulator
    --num-motors 3` covers the multi-axis ones.

6.  **Advanced Topics**
    * [CAN Protocol Details (MKS Specifics)](advanced_topics/can_protocol.md)
    * [Timing and Determinism](advanced_topics/timing_determinism.md)
    * [Troubleshooting Common Issues](advanced_topics/troubleshooting.md)
    * [Asynchronous Control with asyncio](advanced_topics/asynchronous_control_with_asyncio.md)
    * [Using mks-servo-can in Synchronous Applications](advanced_topics/asyncio_with_synchronous_code.md)
    * [Python's Role in Automation: Strengths and Limitations](advanced_topics/python_and_automation.md)

7.  **Printed Hardware**
    * [Designing Printed Parts](../hardware/DESIGNING_PRINTED_PARTS.md) — the
      toolchain with pinned versions and its caveats, the FDM rules these parts
      are held to, and the traps that cost time here
    * [The pan/tilt gimbal](../hardware/gimbal/README.md) — four printable parts
      for two MKS servos, with the scripts that measure their clearances, screw
      access, printability, balance and stress rather than asserting them

8.  **Development & Contribution**
    * [Setting up Development Environment](development/setup.md)
    * [Coding Standards & Style Guide](development/coding_standards.md)
    * [Running Tests](development/running_tests.md)
    * [How to Contribute](development/contributing.md)
    * [Roadmap — planned work in priority order](development/roadmap.md) (open defects, sequencing, and how to verify each)

9.  **Appendices**
    * [Glossary of Terms](appendices/glossary.md)
    * **MKS servo parameter reference** — the manual's command table is
      transcribed into
      [`mks_servo_can/data/manual_commands_v106.json`](../mks_servo_can_library/mks_servo_can/data/),
      which ships inside the wheel because the simulator and the conformance
      tests both read it at runtime. That file, not a summary of it, is what the
      code is checked against — and it carries an `errata` block recording where
      the manual is ambiguous or contradicts itself. The
      [PDF](<MKS SERVO42&57D_CAN User Manual V1.0.6.pdf>) is here too.
      `constants.py` names every command code with its manual section.

## How to Use This Documentation

Every link above resolves. That was not always true — this table of contents
began as an outline of an intended structure and 27 of its 52 links pointed at
nothing, with no indication of which. Each gap has since been closed one of two
ways: the page was written, or the entry was replaced by a pointer to something
executable — the source, the docstrings, a script in `examples/`. Where a page
would only have restated a signature or narrated a script, the pointer is the
better answer, because the thing it points at cannot drift out of step with the
code.

`tests/test_docs_api.py` keeps it that way. It parses every Python block on
these pages, checks that the classes, methods, constructor arguments and imports
they mention actually exist, and checks that every internal link resolves. It is
a ratchet: a new problem fails the build, and a recorded problem that no longer
occurs also fails, with an instruction to delete it. The list can only shrink.

Where a document and the code disagree, trust the code —
`docs/development/roadmap.md` says what is planned next and how each item will
be verified.

* Start with the **Getting Started** section if you are new to the project.
* Refer to the **User Guides** for practical instructions on using the library and simulator.
* Consult the **API Reference** for detailed information on specific modules, classes, and functions.
* Explore **Tutorials & Examples** for step-by-step walkthroughs of common use cases.

## Contributing to Documentation

Documentation improvements are always welcome! If you find errors, omissions, or areas that could be clearer, please feel free to open an issue or submit a pull request.

---

*This `README.md` serves as the main entry point for the detailed documentation. Each linked section above should ideally correspond to a separate Markdown file within the `docs/` directory structure.*