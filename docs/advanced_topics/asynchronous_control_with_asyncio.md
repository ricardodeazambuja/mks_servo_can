# Responsive Automation: Asynchronous Control with asyncio

The mks-servo-can library is fundamentally designed around Python's asyncio framework. This choice was made to enable the creation of highly responsive, scalable, and efficient automation and robotics applications. This document explains the motivation behind this design, its advantages, and the best practices for using it effectively.

## The "Why": Moving Beyond Synchronous Control

In traditional synchronous programming, when you send a command that has to wait for a response (like getting data from a motor over a CAN bus), your entire program freezes until the response arrives.

Consider a simple synchronous example:

```python
# Synchronous Pseudocode - THIS IS NOT HOW THE LIBRARY WORKS
def synchronous_control():
    print("Moving motor 1...")
    motor1.move_absolute(180)  # This call BLOCKS for 5 seconds until the move is done
    
    # Nothing else can happen for those 5 seconds.
    # The UI is frozen, other motors can't be started, sensor data isn't processed.
    
    print("Motor 1 finished. Moving motor 2...")
    motor2.move_absolute(90) # This also blocks.
```

This approach is inefficient and leads to unresponsive applications. asyncio solves this problem by using a cooperative multitasking model managed by an event loop.

## Core Concept: The asyncio Event Loop

Instead of blocking, asyncio allows you to "await" the result of a slow operation. While waiting, your program yields control back to the event loop, which can then run other tasks that are ready.

The mks-servo-can library is built on this principle:
- Every command that involves CAN bus communication is an async function.
- When you await a command like `axis.get_current_position()`, your program is efficiently waiting for the motor's response without freezing.

## Design Decisions in mks-servo-can

The library's architecture is built to leverage asyncio for maximum benefit.

### 1. The Asynchronous CANInterface

The CANInterface class runs a background listener task that continuously watches for incoming CAN messages. When a message arrives, it is processed and used to resolve the Future object of a corresponding command that was waiting for it. This entire process happens without blocking your main application logic.

### 2. Non-Blocking Movement Commands

This is the most powerful feature of the asynchronous design. All movement methods in the Axis class, like `move_absolute` or `move_relative`, have a wait parameter.

#### a. Blocking Wait (wait=True)

This is the simplest way to use a move command. The coroutine will pause until the motor signals that the move is complete.

```python
# This coroutine pauses for the duration of the move,
# but the asyncio event loop can still run other tasks.
print("Starting move and waiting for it to complete...")
await axis.move_relative_user(90, speed_user=180.0, wait=True)
print("Move complete.")
```

#### b. Non-Blocking Initiation (wait=False)

For true concurrency, you can initiate a move and immediately continue with other logic.

```python
print("Starting a non-blocking move...")
# The command is sent, and the code continues immediately.
await axis.move_relative_user(360, speed_user=360.0, wait=False)
print("Move command dispatched. The motor is moving in the background.")

# Perform other operations while the motor is in motion
for i in range(5):
    print(f"  ... doing other work (step {i+1})")
    await asyncio.sleep(0.2)

print("Finished with other work.")
```

### 3. Synchronizing with wait_for_move_completion()

After starting a non-blocking move, you might later need to ensure it's finished. The `wait_for_move_completion()` method is designed for this exact purpose.

```python
# ...following the non-blocking move example above...

if not axis.is_move_complete():
    print("Move is still in progress. Now waiting for it to finish...")
    # This will pause until the move is done.
    await axis.wait_for_move_completion(timeout=10.0)
    print("Move is now confirmed complete.")
else:
    print("Move had already finished.")
```

## Advantages of the Asynchronous Approach

**High Responsiveness**: Your application remains interactive. A user interface would not freeze, and other events can be handled while motors are in motion.

**Concurrent Multi-Axis Control**: It becomes trivial to command multiple motors to start moving at the same time. The MultiAxisController uses `asyncio.gather` internally to dispatch commands to all axes concurrently.

```python
# Using MultiAxisController to start moves on two axes simultaneously
print("Starting concurrent move on multiple axes...")
await multi_controller.move_all_to_positions_abs_user(
    positions_user={"AxisX": 100.0, "AxisY": 50.0},
    wait_for_all=False  # Returns immediately after dispatching
)
print("Commands dispatched to both axes. They are now moving together.")
# You can later use multi_controller.wait_for_all_moves_to_complete()
```

**Improved Efficiency**: Instead of wasting CPU cycles in a blocking wait (`time.sleep()`), asyncio allows the CPU to service other parts of your application, leading to better overall performance.

## Best Practices

**Structure your application around asyncio**: To get the full benefit, your main application logic should live inside async functions.

**Use `await asyncio.sleep()` instead of `time.sleep()`**: A `time.sleep()` call will block the entire event loop, defeating the purpose of asyncio. Always use `await asyncio.sleep()` in your coroutines.

**Leverage non-blocking calls**: For any physical movement that takes time, use `wait=False` and structure your code to perform other tasks concurrently. Use `await axis.wait_for_move_completion()` only when you absolutely need to synchronize.

**Handle exceptions**: asyncio tasks can be cancelled or raise exceptions. Use try...except blocks around await calls to gracefully handle errors like MotorTimeoutError or CommunicationError.

By embracing this asynchronous paradigm, you can build sophisticated, non-blocking control systems that are far more capable than their traditional synchronous counterparts.