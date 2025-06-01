# CAN on Linux: A Quick Survival Guide 🐧

This guide will help you get started with using CAN (Controller Area Network) devices on Linux, focusing on common interfaces like `gs_usb` and `slcan`, and the essential `can-utils` software.

## The Core: SocketCAN

At the heart of CAN communication on Linux is **SocketCAN**. It's a set of CAN drivers and a networking stack that makes CAN devices appear as regular network interfaces. This is fantastic because it means you can use familiar networking tools (like `ifconfig` or `ip` from `iproute2`) and programming paradigms (sockets) with CAN.

---

## Common CAN Interface Types & Setup

There are several ways your CAN hardware might connect to your Linux system. Here are two prevalent ones:

### 1. `gs_usb` (Geschwister Schneider USB/CAN) Devices

* **What they are:** These are USB-to-CAN adapters that implement the `gs_usb` protocol. Many modern CAN adapters use this.
* **How they work:** The kernel driver for `gs_usb` creates a SocketCAN network interface, typically named `can0`, `can1`, etc.
* **`socketcand`:** This daemon is primarily used when you want to share a single CAN interface (that might not be a native SocketCAN interface, or you want to provide access over a network) or when dealing with some specific hardware that benefits from this userspace daemon. For many `gs_usb` devices that have direct kernel support, `socketcand` might not be strictly necessary for basic operation, as the kernel driver handles the SocketCAN interface creation directly.
* **Setup Steps (Typical for direct kernel support):**
    1.  **Install `can-utils`:**
        ```bash
        sudo apt update
        sudo apt install can-utils
        ```
    2.  **Connect the `gs_usb` device:** Plug it into a USB port.
    3.  **Identify the interface:** Check `dmesg` for messages indicating the device was found and a `canX` interface was created.
        ```bash
        dmesg | grep can
        ```
    4.  **Bring up the interface:** Use `ip link` to configure and activate the CAN interface. You'll need to set the bitrate. For example, for 250 kbit/s:
        ```bash
        sudo ip link set can0 type can bitrate 250000
        sudo ip link set up can0
        ```
        Replace `can0` with your actual interface name and `250000` with your required bitrate.
    5.  **Verify:** Use `ifconfig can0` or `ip addr show can0` to see if the interface is up and configured.

### 2. `slcan` (Serial Line CAN) Devices

* **What they are:** These are typically simpler, often lower-cost, USB-to-CAN adapters (or even RS232-to-CAN adapters) that use a text-based serial protocol to transmit and receive CAN frames.
* **How they work:** You need a userspace daemon called `slcand` to bridge the serial communication from the device to a SocketCAN network interface. `slcand` listens on the serial port (e.g., `/dev/ttyUSB0`, `/dev/ttyACM0`) and translates the serial data into CAN frames for SocketCAN, creating a `slcanX` interface (e.g., `slcan0`).
* **Setup Steps:**
    1.  **Install `can-utils`**.
    2.  **Connect the `slcan` device:** Plug it in. It should appear as a serial device.
    3.  **Identify the serial port:** Check `dmesg` or `ls /dev/tty*`.
        ```bash
        dmesg | grep tty
        ```
    4.  **Start `slcand`:**
        ```bash
        sudo slcand -o -c -sX /dev/ttyUSB0 slcan0
        ```
        * `-o`: Opens the serial port.
        * `-c`: Closes the port on exit.
        * `-sX`: Sets the CAN bitrate. `X` is a speed grade (e.g., `-s6` for 500 kbit/s, `-s5` for 250 kbit/s). Consult your adapter's documentation or use `-b <bitrate>` if supported.
        * `/dev/ttyUSB0`: The serial port your adapter is on.
        * `slcan0`: The name for the SocketCAN interface.
    5.  **Bring up the SocketCAN interface:**
        ```bash
        sudo ip link set up slcan0
        ```
    6.  **Verify:** Use `ifconfig slcan0` or `ip addr show slcan0`.

---

## Using `can-utils` 🛠️

Once your `canX` (for `gs_usb`) or `slcanX` (for `slcan`) interface is up and running, you can use `can-utils` to interact with the CAN bus:

* **`candump`**: Displays received CAN frames.
    ```bash
    candump can0
    ```
* **`cansend`**: Sends a CAN frame.
    ```bash
    cansend can0 123#11223344AABBCCDD  # ID 123, 8 data bytes
    cansend can0 7DF#0201050000000000 # Example OBD-II request
    cansend can0 001#FE000F64000554cb # Move (FE absolute motion by pulses) at speed x000F and acc 0x64 to position 0x000554 with crc 0xcb
    ```
* **`cangen`**: Generates random CAN traffic.
    ```bash
    cangen can0
    ```
* **`canplayer`**: Replays CAN frames from a log file.
    ```bash
    canplayer -I candump.log
    ```
* **`cansniffer`**: Shows only changing data bytes for specific IDs.
    ```bash
    cansniffer can0
    ```

---

## Key Differences & Summary

| Feature         | `gs_usb` (Direct Kernel)                                  | `slcan` (via `slcand`)                                         |
| :-------------- | :-------------------------------------------------------- | :------------------------------------------------------------- |
| **Driver** | Kernel module (e.g., `gs_usb.ko`)                         | Userspace daemon (`slcand`) + generic serial kernel driver      |
| **Interface** | `canX` (e.g., `can0`)                                     | `slcanX` (e.g., `slcan0`) after `slcand` runs                 |
| **Bitrate Setup**| `sudo ip link set canX type can bitrate <rate>`           | Via `slcand` arguments (e.g., `-sX` or `-b <rate>`)            |
| **`socketcand`**| Generally not needed for basic operation if kernel support is present. | Not directly used; `slcand` provides the SocketCAN bridge.    |

**`socketcand` in more detail:**
Think of `socketcand` as a versatile tool. It can:
1.  **Bridge non-SocketCAN hardware to SocketCAN:** If you have CAN hardware that doesn't have a native SocketCAN kernel driver.
2.  **Network CAN:** Expose a local CAN interface over a network (TCP/IP).
3.  **Multiplexing/Demultiplexing:** Allow multiple applications to access a single CAN interface.

For many common `gs_usb` devices, the kernel handles making it a SocketCAN interface directly.

---

## Python Integration 🐍

With your CAN interface (`can0`, `slcan0`, etc.) configured and up, Python libraries like your `mks_servo_can` or the popular `python-can` library can then use this SocketCAN interface. They typically expect the SocketCAN interface to be already configured and active (e.g., `channel='can0'`, `bustype='socketcan'`).

