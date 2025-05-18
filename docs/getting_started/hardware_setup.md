# Basic Hardware Setup for MKS Servo CAN Motors

Connecting to physical MKS SERVO42D or SERVO57D motors requires careful attention to the hardware setup to ensure proper communication and prevent damage to your components. This guide outlines the essential steps.

**Always consult the official "MKS SERVO42D/57D_CAN User Manual" for detailed specifications, wiring diagrams, and safety precautions specific to your motor version.**

## 1. Safety First!

* **Power Off:** Ensure all power to motors and control electronics is OFF before making any connections or disconnections.
* **Correct Voltage:** Verify that your power supply matches the voltage requirements of your MKS servo motors. Over-voltage can damage the motors.
* **Secure Connections:** Ensure all wiring is secure and properly insulated to prevent shorts.

## 2. Components Needed

* **MKS Servo Motor(s):** SERVO42D, SERVO57D, or similar CAN-bus enabled MKS servos.
* **CAN Adapter:** A `python-can` compatible CAN interface (e.g., CANable, Kvaser, PCAN, SocketCAN device). Refer to the [Prerequisites](./prerequisites.md) for more examples.
* **Host Computer:** A computer to run your Python scripts using the `mks-servo-can` library.
* **Power Supply:** A suitable DC power supply for your servo motors (e.g., 24V, with sufficient current capacity).
* **CAN Bus Wiring:**
    * Twisted-pair cable is recommended for CAN_H and CAN_L lines to improve noise immunity.
    * Connectors (if applicable for your motors and adapter).
* **Termination Resistors:** Two 120 Ohm resistors. Most MKS servo motors and some CAN adapters have built-in, selectable termination resistors.

## 3. Wiring the CAN Bus

The CAN bus is a two-wire differential bus consisting of CAN_H (CAN High) and CAN_L (CAN Low) lines.

* **Daisy Chaining (Recommended):** Motors should ideally be connected in a daisy-chain fashion (a single bus line with devices tapped off it) rather than a star topology.
* **CAN_H to CAN_H:** Connect the CAN_H pin of your adapter to the CAN_H pin of the first motor. Then connect the CAN_H of the first motor to the CAN_H of the second motor, and so on.
* **CAN_L to CAN_L:** Similarly, connect the CAN_L pin of your adapter to the CAN_L pin of the first motor, and then to subsequent motors.
* **Ground (GND):** It's good practice to have a common ground reference between the CAN adapter and the motors, especially if they are powered by separate supplies or if the CAN adapter is not galvanically isolated. However, consult your motor and adapter manuals, as direct GND connection on the CAN bus itself is sometimes debated and can cause ground loops if not done carefully. Often, the power supply ground provides the necessary reference.

## 4. CAN Bus Termination

Proper termination is critical for CAN bus reliability. The bus must be terminated at **both physical ends** with a 120 Ohm resistor between CAN_H and CAN_L.

* **Motor Termination:** MKS SERVO42D/57D motors typically have a built-in 120 Ohm termination resistor that can be enabled or disabled via a DIP switch or jumper (labeled "CAN_R" or similar).
    * **First and Last Motor:** The motors at the two physical ends of your CAN bus chain should have their termination resistors **enabled**.
    * **Intermediate Motors:** Any motors in the middle of the chain should have their termination resistors **disabled**.
* **Adapter Termination:** Some CAN adapters also have built-in selectable termination.
    * If your CAN adapter is one end of the bus, enable its termination.
    * If your adapter is in the middle of a segment (less common for typical setups), disable its termination and ensure the actual ends of the bus are terminated.

**Common Setup:**
* PC with CAN adapter <-> Motor 1 <-> Motor 2 <-> ... <-> Motor N
* Termination enabled on the CAN adapter (if it's an end-point) OR on the first motor if the adapter is not at the physical end.
* Termination enabled on the LAST motor in the chain (Motor N).
* All intermediate motors have their termination disabled.

## 5. Motor Power

* Connect your MKS servo motors to an appropriate power supply (e.g., 24V DC).
* Ensure the power supply can provide sufficient current for all motors, especially during acceleration or high torque conditions.
* Double-check polarity (+V and GND) before applying power.

## 6. CAN Adapter Connection to Host

* Connect your CAN adapter to your host computer (e.g., via USB, Ethernet, etc.).
* Install any necessary drivers for the adapter as specified by its manufacturer and the `python-can` documentation for that interface type.
* Configure the CAN interface on your operating system if required (e.g., setting up a SocketCAN interface on Linux using `ip link`).

## 7. Setting Motor CAN IDs and Bitrate

* **CAN ID:** Each motor on the CAN bus must have a unique CAN ID. MKS servo motors usually allow setting the CAN ID via DIP switches or through software configuration (refer to the motor manual). Ensure the IDs you set on the hardware match what you will use in your software.
* **Bitrate:** All devices on the CAN bus (adapter and all motors) must be configured to operate at the **same bitrate** (e.g., 500000 bps). This is also typically set via DIP switches or software on the MKS motors. The `mks-servo-can` library defaults to 500000 bps but can be configured when initializing `CANInterface`.

## Example Checklist Before Power-On:

1.  [ ] All power OFF.
2.  [ ] CAN_H and CAN_L correctly wired between adapter and all motors.
3.  [ ] Bus termination (120 Ohm) correctly enabled at ONLY the two physical ends of the bus.
4.  [ ] Unique CAN ID set for each motor.
5.  [ ] All motors and the CAN adapter configured for the same CAN bitrate.
6.  [ ] Motor power supply connected with correct voltage and polarity.
7.  [ ] CAN adapter connected to the host computer.

Once these steps are completed and verified, you can proceed to power on the system and attempt to communicate with the motors using the `mks-servo-can` library as shown in `examples/single_axis_real_hw.py`.