# Bridging Worlds: Using mks-servo-can in Synchronous Applications

The `mks-servo-can` library is built on `asyncio` to provide highly responsive, non-blocking control over your hardware. However, many existing applications, from simple scripts to complex GUI applications (like Tkinter/PyQt) or web frameworks (like Flask), are synchronous.

A common sentiment among developers is that `asyncio` can feel like a "virus": once you use an `async` library, you're forced to make your entire call stack `async`. While there's truth to this, Python provides clear and official ways to bridge the asynchronous and synchronous worlds, allowing you to use `mks-servo-can` effectively within any application.

This guide explains how.

## The Core Problem: Calling `async` from `def`

You cannot directly `await` an asynchronous function from a standard synchronous function. The `await` keyword is only valid inside an `async def` function.

```python
# This will NOT work and raises a SyntaxError
def my_synchronous_function():
    # SyntaxError: 'await' outside async function
    position = await my_axis.get_current_position_user()
    print(f"Position is {position}")
```

To call an async function, you need a running asyncio event loop. The key is how you access or create that loop from your synchronous code.

## Solution 1: The Simple Script (`asyncio.run`)

This is the most common and straightforward scenario. You have a standard Python script, and you want to use the `mks-servo-can` library to perform a sequence of actions.

The solution is to write your main logic in an `async` function and then use `asyncio.run()` to start the event loop and run that single function until it completes.

**Best for:** Standalone scripts, command-line tools, and programs where the primary entry point can be asynchronous.

### Example

```python
import asyncio
from mks_servo_can import CANInterface, Axis

# All your library logic goes inside an async function
async def main():
    print("Starting the main async logic...")
    can_if = CANInterface()
    await can_if.connect()

    axis = Axis(can_if, motor_can_id=1)
    await axis.initialize()

    await axis.enable_motor()
    await axis.move_relative_user(distance_user=90.0, speed_user=30.0, wait=True)
    position = await axis.get_current_position_user()

    print(f"Final position: {position:.2f} degrees")

    await can_if.disconnect()

# The synchronous part of your script
if __name__ == "__main__":
    print("This is the synchronous entry point.")
    # asyncio.run() creates an event loop, runs the 'main' coroutine,
    # and closes the loop when it's done.
    asyncio.run(main())
    print("The async part is finished, back in sync context.")
```

## Solution 2: The Integrated Application (Threads)

This is the solution for more complex scenarios, such as:

- Integrating `mks-servo-can` into a GUI application (e.g., calling a motor from a button-click handler)
- Using the library within a synchronous web framework like Flask

In these cases, the main application already has its own blocking event loop (e.g., `app.run()` or `root.mainloop()`). You cannot call `asyncio.run()` as it would block this main loop.

The solution is to run the asyncio event loop in a separate, dedicated background thread. Your synchronous code can then safely submit tasks to that loop and wait for their results.

**Best for:** GUI apps, synchronous web servers, or any long-running application where `mks-servo-can` is just one component.

### Example

This example demonstrates how to create a `MotorController` class that can be safely called from any synchronous code.

```python
import asyncio
import threading
import time
from mks_servo_can import CANInterface, Axis

class ThreadSafeMotorController:
    """
    A class that manages the asyncio event loop in a background thread,
    allowing synchronous code to safely call async library functions.
    """
    def __init__(self):
        self._loop = None
        self._thread = None
        self._axis = None
        self.is_running = threading.Event()

    def _run_event_loop(self):
        """This function runs in the background thread."""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        # We now have an event loop running and can signal that we're ready.
        self.is_running.set()
        self._loop.run_forever()

    async def _setup_motor(self):
        """An async method to set up our hardware."""
        can_if = CANInterface()
        await can_if.connect()
        self._axis = Axis(can_if, motor_can_id=1)
        await self._axis.initialize()
        await self._axis.enable_motor()
        print("Motor setup complete in background thread.")

    def start(self):
        """Starts the background thread and the event loop."""
        if self._thread is not None:
            return  # Already started

        self._thread = threading.Thread(target=self._run_event_loop, daemon=True)
        self._thread.start()
        # Wait until the loop is actually running in the thread
        self.is_running.wait()
        # Schedule the async setup routine to run on the loop
        self.run_async(self._setup_motor())
        print("Motor controller started.")

    def run_async(self, coro):
        """
        Submits a coroutine to the event loop in the background thread
        and blocks until it completes.

        Args:
            coro: The coroutine to run (e.g., axis.get_position()).

        Returns:
            The result of the coroutine.
        """
        if not self.is_running.is_set() or self._loop is None:
            raise RuntimeError("Event loop is not running.")

        # Submit the coroutine to the loop from our current (sync) thread.
        # This is thread-safe.
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)

        # Block and wait for the result from the other thread.
        return future.result()

    def stop(self):
        """Stops the event loop and joins the thread."""
        if self._loop:
            self.run_async(self._axis.can_interface.disconnect())
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread:
            self._thread.join()
        print("Motor controller stopped.")

# --- Synchronous Main Application Logic ---
if __name__ == "__main__":
    controller = ThreadSafeMotorController()
    controller.start()

    print("\n--- Now in the main synchronous code ---")
    print("We can call async methods as if they were synchronous.")

    # Each of these calls will block until the async operation in the
    # background thread is complete, but they don't block each other.

    # Move the motor
    print("Sync: Moving motor...")
    controller.run_async(controller._axis.move_relative_user(distance_user=45.0, speed_user=30.0, wait=True))
    print("Sync: Move complete.")

    time.sleep(1)

    # Get the position
    print("Sync: Getting position...")
    pos = controller.run_async(controller._axis.get_current_position_user())
    print(f"Sync: Got position -> {pos:.2f} degrees")

    controller.stop()
```

## Summary

This threaded pattern provides a clean and robust bridge, allowing you to "contain" the asyncio parts of your application without letting it take over your entire codebase.

### Key Takeaways

- **For simple scripts:** Use `asyncio.run()` to wrap your main async logic
- **For integrated applications:** Use a background thread with `asyncio.run_coroutine_threadsafe()`
- Both approaches allow you to use `mks-servo-can` effectively without making your entire application asynchronous