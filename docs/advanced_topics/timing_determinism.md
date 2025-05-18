# Timing and Determinism in mks-servo-can

## What are Timing and Determinism?

**Timing** refers to when events occur and the duration between them. In motor control, this could be:
- The time taken for a command to be sent, processed by the motor, and for a response to be received (round-trip time or latency)
- The consistency of sending commands at regular intervals

**Determinism** refers to the predictability of these timings. A deterministic system is one where the time taken for operations is consistent and predictable, with minimal jitter (variation in timing). Real-time systems often strive for a high degree of determinism.

## Factors Affecting Timing and Determinism

### 1. Python and asyncio

- **Global Interpreter Lock (GIL)**: Python's GIL means that even with asyncio, CPU-bound tasks in different threads don't truly run in parallel on multi-core processors within a single Python process. However, asyncio excels at I/O-bound tasks (like waiting for network or CAN responses) by allowing other tasks to run while one is waiting.
- **Event Loop Latency**: The asyncio event loop itself has some overhead in scheduling and running tasks. High load or long-running synchronous code within an async function can block the event loop, affecting the timing of other scheduled tasks.
- **Garbage Collection**: Python's garbage collector can pause execution, introducing unpredictable delays.

### 2. CAN Bus Characteristics

- **Bitrate**: Higher bitrates (e.g., 1 Mbps vs. 250 Kbps) mean messages are transmitted faster. The default for this library and many MKS motors is 500 Kbps.
- **Bus Load**: A busy CAN bus (many devices transmitting frequently) can lead to arbitration delays, where messages wait for the bus to become free.
- **Message Prioritization**: CAN uses message IDs for arbitration; lower IDs have higher priority.
- **Error Frames**: Errors on the bus (e.g., from noise or faulty nodes) can cause retransmissions and delays.

### 3. Operating System (OS)

- **Scheduling**: Standard OS schedulers are not designed for hard real-time. Other processes and system interrupts can preempt your Python application, causing jitter.
- **Driver Latency**: Drivers for USB CAN adapters or serial ports can introduce their own latencies.

### 4. CAN Adapter Hardware

- **Processing Power**: The microcontroller on the CAN adapter has its own processing time for converting USB/serial commands to CAN frames and vice versa.
- **Buffering**: Adapters buffer messages, which can smooth out flow but also add latency.
- **Firmware**: The adapter's firmware plays a significant role.

### 5. Motor Response Time

- The MKS servo motor itself takes time to process a received CAN command and prepare a response. This internal processing time can vary slightly depending on the command and the motor's current state.

## Library Design Considerations for Timing

The mks-servo-can library is built on asyncio, which is well-suited for managing I/O-bound operations like CAN communication.

- **Asynchronous Operations**: Sending a command and waiting for a response is handled asynchronously, allowing other Python tasks to run instead of blocking.
- **Response Matching**: The CANInterface efficiently matches incoming CAN messages to pending command futures.

However, the library itself cannot overcome the inherent limitations of Python or a non-real-time OS for achieving hard determinism.

## Using the Simulator for Latency Simulation

The mks-servo-simulator includes a `--latency-ms` option. This simulates a fixed round-trip delay for messages passing through the virtual CAN bus.

```bash
mks-servo-simulator --latency-ms 5.0
```

This adds a 5ms delay (2.5ms one-way) to mimic network or bus latency. While this is a simplified model (real-world latency often includes jitter), it can help test how your application logic copes with delays.

## Benchmark Examples

- **examples/benchmark_command_latency.py**: This script sends a simple command (like reading a parameter) repeatedly to a motor (either real or simulated) and measures the round-trip time for each command-response pair. It then calculates and prints statistics like minimum, maximum, average latency, and standard deviation.
- **examples/timing_benchmark.py**: This script focuses on the timing of sending commands at regular intervals. It attempts to send commands at a fixed rate (e.g., every 10ms) and measures how accurately these intervals are maintained. This can highlight issues with asyncio event loop congestion or OS scheduling impacts.

## Striving for Determinism (Tips)

While hard determinism is difficult, here are some tips for improving timing consistency:

1. **Keep Async Code Non-Blocking**: Ensure that async functions await I/O operations promptly and avoid long-running synchronous (CPU-bound) computations within them, as these will block the event loop.
2. **Offload CPU-bound work**: Separate CPU-bound tasks into threads or processes for better performance.
3. **Optimize System Performance**: Reduce background processes and system load on the machine running your Python application.
4. **Use Real-Time OS (Advanced)**: For applications requiring higher determinism, consider using a real-time operating system (RTOS) or a Linux kernel with PREEMPT_RT patches. This is an advanced step and may require more specialized hardware and software configuration.
5. **Use Dedicated Hardware**: Use a dedicated microcontroller (e.g., an ESP32, Raspberry Pi Pico, or a motion controller board) for time-critical motor control tasks, which then communicates with a higher-level Python application for coordination.

## Conclusion

The mks-servo-can library provides an asynchronous interface for controlling MKS servo motors, suitable for a wide range of applications. While it leverages asyncio for efficient I/O, users should be aware of the various factors that can influence timing and determinism.