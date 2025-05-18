"""
CAN Communication Layer for MKS Servo Control.
Handles raw CAN bus communication using python-can for real hardware
and provides a "virtual" backend for the simulator.
"""
from typing import Any, Callable, Dict, List, Optional, Tuple # Added Tuple

import asyncio
import logging
import time

try:
    import can
    from can.notifier import Notifier # For advanced listening
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False

    # Create dummy can classes/exceptions if can is not installed for basic loading
    class can: # type: ignore
        """
        Dummy 'can' module placeholder used when the 'python-can' library is not installed.
        This allows the mks_servo_can library to be imported and for basic type
        checking or offline development, but it does not provide real CAN functionality.
        """
        class Listener: # Add dummy Listener
            """
            Dummy placeholder for the can.Listener class from python-can.
            Intended to be subclassed by user-defined listeners for CAN messages.
            """
            def on_message_received(self, msg):
                """
                Dummy method called when a message is received. Subclasses should override this.

                Args:
                    msg: The received CAN message.
                """
                pass
            def on_error(self, exc):
                """
                Dummy method called when an error occurs. Subclasses should override this.

                Args:
                    exc: The exception that occurred.
                """
                pass

        class Notifier: # Add dummy Notifier
             """
             Dummy placeholder for the can.Notifier class from python-can.
             Manages message distribution from a bus to a list of listeners.
             """
             def __init__(self, bus, listeners, timeout=None):
                 """
                 Initializes a dummy Notifier.

                 Args:
                     bus: The CAN bus to listen to (dummy).
                     listeners: A list of listener objects (dummy).
                     timeout (Optional[float]): An optional timeout.
                 """
                 pass
             def stop(self, timeout=None):
                 """
                 Stops the dummy Notifier.

                 Args:
                     timeout (Optional[float]): An optional timeout for stopping.
                 """
                 pass

        class BusABC:
            """
            Dummy placeholder for the can.BusABC class from the python-can library.
            Represents an abstract CAN bus.
            """
            pass

        class Message:
            """
            Dummy placeholder for the can.Message class from the python-can library.
            Represents a single CAN message with minimal attributes.
            """
            def __init__(
                self,
                arbitration_id=0,
                data=None, # type: ignore
                is_extended_id=False,
                timestamp=0.0,
                dlc=None # Added dlc to dummy
            ):
                """
                Initializes a dummy CAN Message object.

                Args:
                    arbitration_id (int): The arbitration ID of the message.
                    data (Optional[bytes]): The data payload of the message. Defaults to empty bytes.
                    is_extended_id (bool): Flag indicating if an extended ID is used.
                    timestamp (float): Timestamp of the message.
                    dlc (Optional[int]): Data Length Code. If None, it's derived from data.
                """
                self.arbitration_id = arbitration_id
                self.data = data if data is not None else b""
                if dlc is None:
                    self.dlc = len(self.data)
                else:
                    self.dlc = dlc
                self.is_extended_id = is_extended_id
                self.timestamp = timestamp
                self.channel = None
                self.is_rx = True  # Assume received
                self.is_error_frame = False
                self.is_remote_frame = False

        class CanError(Exception):
            """
            Dummy placeholder for the can.CanError exception from python-can.
            Base class for CAN-related errors in the dummy module.
            """
            pass

        class CanOperationError(
            CanError # type: ignore[name-defined] # if can is dummy
        ):
            """
            Dummy placeholder for the can.CanOperationError exception from python-can.
            Indicates an error during a CAN operation.
            """
            pass


from .constants import CAN_DEFAULT_BITRATE
from .constants import CAN_TIMEOUT_SECONDS
from .exceptions import CANError
from .exceptions import CommunicationError
from .exceptions import ConfigurationError
from .exceptions import SimulatorError

logger = logging.getLogger(__name__)


class AsyncioCanListener(can.Listener if CAN_AVAILABLE else object): # type: ignore[misc] # if can is dummy
    """
    A can.Listener that puts received messages onto an asyncio.Queue.

    This class acts as a bridge between the synchronous `python-can` notifier thread
    (which calls `on_message_received` and `on_error`) and an asyncio event loop.
    It allows asynchronous processing of CAN messages within an asyncio application.
    """
    def __init__(self, queue: asyncio.Queue, loop: asyncio.AbstractEventLoop):
        """
        Initializes the AsyncioCanListener.

        Args:
            queue: The asyncio.Queue instance where received CAN messages will be placed.
                   This queue should be consumed by an asyncio task.
            loop: The asyncio event loop that the queue and its consumer are running in.
                  This is used to schedule the `put_nowait` call safely from the notifier thread.
        """
        self.queue = queue
        self.loop = loop
        super().__init__() # Important for can.Listener initialization if not a dummy

    def on_message_received(self, msg: can.Message): # type: ignore[name-defined]
        """
        Callback method invoked by `python-can`'s Notifier when a new CAN message is received.

        This method places the received message onto the asyncio.Queue in a thread-safe manner,
        allowing it to be processed by an asyncio task running in the event loop.

        Args:
            msg: The `can.Message` object received from the bus.
        """
        if self.loop.is_running():
            self.loop.call_soon_threadsafe(self.queue.put_nowait, msg)
        else:
            logger.warning("AsyncioCanListener: Event loop not running, CAN message dropped.")

    def on_error(self, exc: Exception) -> None:
        """
        Callback method invoked by `python-can`'s Notifier if an error occurs
        within the Notifier's internal mechanisms or the bus it's attached to.

        Logs the received exception.

        Args:
            exc: The Exception object that occurred.
        """
        logger.error(f"AsyncioCanListener: Error in CAN listener/notifier: {exc}", exc_info=True)


class CANInterface:
    """
    Manages CAN communication, supporting both real hardware and a simulator.

    This class provides a unified interface for sending and receiving CAN messages,
    abstracting the differences between a physical CAN bus (via `python-can`)
    and a TCP-based simulator. It handles connection management, message queuing,
    response future management for request-response patterns, and allows registration
    of general message handlers.
    """

    def __init__(
        self,
        interface_type: str = "canable",
        channel: Optional[str] = None,
        bitrate: int = CAN_DEFAULT_BITRATE,
        simulator_host: str = "localhost",
        simulator_port: int = 6789,
        use_simulator: bool = False,
        loop: Optional[asyncio.AbstractEventLoop] = None,
    ):
        """
        Initializes the CAN interface.

        Args:
            interface_type (str): Type of CAN interface (e.g., 'canable', 'kvaser', 'socketcan').
                                  Only used if `use_simulator` is False.
            channel (Optional[str]): Channel specific to the interface (e.g., 'slcan0', '/dev/ttyUSB0').
                                     Only used if `use_simulator` is False.
            bitrate (int): CAN bus bitrate (e.g., 125000, 250000, 500000, 1000000).
                           Only used if `use_simulator` is False.
            simulator_host (str): Hostname or IP address for the simulator.
                                  Used if `use_simulator` is True.
            simulator_port (int): TCP port for the simulator.
                                  Used if `use_simulator` is True.
            use_simulator (bool): If True, connects to a simulator instead of real hardware.
                                  Defaults to False.
            loop (Optional[asyncio.AbstractEventLoop]): The asyncio event loop to use.
                                                        If None, `asyncio.get_event_loop()` is called.
        """
        self.use_simulator = use_simulator
        self.bus: Optional[can.BusABC] = None # type: ignore[name-defined]
        self._loop = loop if loop else asyncio.get_event_loop()
        self._message_handlers: Dict[
            int, List[Callable[[can.Message], None]] # type: ignore[name-defined]
        ] = {}  # CAN ID -> list of handlers
        self._response_futures: Dict[
            Tuple[int, int], List[Tuple[asyncio.Future, Optional[Callable[[can.Message], bool]]]] # type: ignore[name-defined]
        ] = {}  # (can_id, command_code) -> list of (Future, Predicate)
        self._is_listening = False
        self._listener_task: Optional[asyncio.Task] = None

        # For Notifier-based hardware listening
        self._message_queue: Optional[asyncio.Queue] = None
        self._notifier: Optional[can.Notifier] = None # type: ignore[name-defined]


        if self.use_simulator:
            self.simulator_host = simulator_host
            self.simulator_port = simulator_port
            self._sim_reader: Optional[asyncio.StreamReader] = None
            self._sim_writer: Optional[asyncio.StreamWriter] = None
            logger.info(
                f"CANInterface configured to use simulator at {simulator_host}:{simulator_port}"
            )
        else:
            if not CAN_AVAILABLE:
                raise ConfigurationError(
                    "python-can library is not installed. Please install it to use real hardware."
                )
            self.interface_type = interface_type
            self.channel = channel
            self.bitrate = bitrate
            logger.info(
                f"CANInterface configured for hardware: {interface_type} on {channel} @ {bitrate} bps"
            )

    async def connect(self):
        """
        Establishes the connection to the configured CAN bus (real hardware) or simulator.

        If connecting to hardware, it initializes the `python-can` bus and sets up
        a notifier with `AsyncioCanListener` to process incoming messages asynchronously.
        If connecting to the simulator, it establishes a TCP connection.
        Starts the message listening loop upon successful connection.

        Raises:
            SimulatorError: If connection to the simulator fails (e.g., connection refused, other network errors).
            CANError: If initialization of the `python-can` bus fails for hardware (e.g., adapter not found, driver issues).
            ConfigurationError: If `python-can` is required but not installed, or if essential parameters like
                                `channel` are missing for hardware mode.
        """
        if self.is_connected:
            logger.info(f"CANInterface already connected {'to simulator' if self.use_simulator else 'to hardware'}.")
            return

        if self.use_simulator:
            try:
                logger.info(
                    f"Attempting to connect to simulator at {self.simulator_host}:{self.simulator_port}..."
                )
                self._sim_reader, self._sim_writer = (
                    await asyncio.open_connection(
                        self.simulator_host, self.simulator_port
                    )
                )
                logger.info("Successfully connected to simulator.")
            except ConnectionRefusedError as e:
                raise SimulatorError(
                    f"Connection to simulator refused at {self.simulator_host}:{self.simulator_port}. Is it running?"
                ) from e
            except Exception as e:
                raise SimulatorError(
                    f"Failed to connect to simulator: {e}"
                ) from e
        else: # Hardware
            if not self.channel and self.interface_type not in [
                "virtual" # python-can's virtual interface might not need a channel
            ]:
                raise ConfigurationError(
                    f"Channel must be specified for {self.interface_type} interface."
                )
            try:
                logger.info(
                    f"Attempting to connect to CAN hardware: type={self.interface_type}, channel={self.channel}, bitrate={self.bitrate}"
                )
                self.bus = can.interface.Bus( # type: ignore[name-defined]
                    bustype=self.interface_type,
                    channel=self.channel,
                    bitrate=self.bitrate,
                )
                logger.info(
                    f"Successfully connected to CAN bus: {self.bus.channel_info if hasattr(self.bus, 'channel_info') else 'N/A'}"
                )
                # Setup for Notifier-based listening for hardware
                self._message_queue = asyncio.Queue()
                async_listener = AsyncioCanListener(self._message_queue, self._loop)
                if self.bus: # Ensure bus was initialized
                    self._notifier = can.Notifier(self.bus, [async_listener], timeout=0.1) # type: ignore[name-defined]
                    logger.info("CAN Notifier created for hardware.")
                else: # Should not happen if Bus() call was successful
                    raise CANError("CAN Bus object not available after attempting initialization.")

            except can.CanError as e: # type: ignore[name-defined]
                logger.error(
                    f"Failed to connect to CAN bus: {e}", exc_info=True
                )
                raise CANError(
                    f"Failed to initialize CAN bus ({self.interface_type} on {self.channel}): {e}"
                ) from e
            except Exception as e: # pylint: disable=broad-except
                logger.error(
                    f"An unexpected error occurred during CAN bus connection: {e}",
                    exc_info=True,
                )
                raise CANError(
                    f"Unexpected error connecting to CAN bus: {e}"
                ) from e

        self.start_listening()

    async def disconnect(self):
        """
        Disconnects from the CAN bus or simulator and cleans up resources.

        This stops the message listener, closes any open connections (TCP for simulator,
        `python-can` bus shutdown for hardware), and clears internal handlers and futures.
        """
        self.stop_listening() # Ensure listener task is stopped first
        if self._listener_task and not self._listener_task.done():
            try:
                await asyncio.wait_for(self._listener_task, timeout=1.0)
            except asyncio.TimeoutError:
                logger.warning("Listener task did not finish gracefully on disconnect timeout.")
            except asyncio.CancelledError:
                logger.info("Listener task cancelled on disconnect.")
        self._listener_task = None


        if self.use_simulator:
            if self._sim_writer:
                try:
                    if not self._sim_writer.is_closing():
                        self._sim_writer.close()
                        await self._sim_writer.wait_closed()
                    logger.info("Disconnected from simulator.")
                except Exception as e: # pylint: disable=broad-except
                    logger.warning(f"Error during simulator disconnection: {e}")
                finally:
                    self._sim_reader = None
                    self._sim_writer = None
        else: # Hardware
            if self._notifier:
                try:
                    self._notifier.stop(timeout=1.0) # Give notifier's thread time to stop
                    logger.info("CAN Notifier stopped.")
                except Exception as e: # pylint: disable=broad-except
                    logger.warning(f"Error stopping CAN Notifier: {e}")
                finally:
                    self._notifier = None
            if self.bus:
                try:
                    self.bus.shutdown()
                    logger.info("Disconnected from CAN bus.")
                except can.CanError as e: # type: ignore[name-defined]
                    logger.warning(f"Error during CAN bus shutdown: {e}")
                finally:
                    self.bus = None
        
        self._message_handlers.clear()
        # Cancel and clear pending futures
        for key_tuple in list(self._response_futures.keys()): # Iterate over a copy
            future_predicate_list = self._response_futures.get(key_tuple, [])
            for future, _ in future_predicate_list:
                if not future.done():
                    future.cancel("CANInterface disconnecting")
        self._response_futures.clear()


    def _can_message_to_sim_protocol(self, msg: can.Message) -> bytes: # type: ignore[name-defined]
        """
        Converts a `python-can.Message` object to the string/bytes protocol format expected by the simulator.

        The simulator protocol is: "SIM_CAN_SEND <id_hex> <dlc_int> <data_hex_no_space>\n".

        Args:
            msg: The `can.Message` object to convert.

        Returns:
            A bytes object representing the message in simulator protocol format.
        """
        if msg.dlc == 0: # type: ignore[union-attr] # if can is dummy
            data_str = ""
        else:
            data_str = "".join(f"{b:02X}" for b in msg.data) # type: ignore[union-attr]
        return f"SIM_CAN_SEND {msg.arbitration_id:03X} {msg.dlc} {data_str}\n".encode() # type: ignore[union-attr]

    def _sim_protocol_to_can_message(self, line: str) -> Optional[can.Message]: # type: ignore[name-defined]
        """
        Converts a simulator protocol string back into a `python-can.Message` object.

        The simulator sends messages prefixed with "SIM_CAN_RECV".
        Format: "SIM_CAN_RECV <id_hex> <dlc_int> <data_hex_no_space>"

        Args:
            line: The string line received from the simulator.

        Returns:
            A `can.Message` object if parsing is successful, otherwise None.
        """
        parts = line.strip().split()
        if not parts or parts[0] != "SIM_CAN_RECV" or len(parts) < 3: 
            logger.warning(
                f"Received unknown or malformed data from simulator: {line}"
            )
            return None
        try:
            can_id = int(parts[1], 16)
            dlc = int(parts[2])
            hex_data = parts[3] if len(parts) > 3 else ""

            if len(hex_data) != dlc * 2:
                logger.error(
                    f"Simulator DLC mismatch: expected {dlc*2} hex chars for {dlc} bytes, got {len(hex_data)} for data '{hex_data}' from line '{line}'"
                )
                return None

            data = bytes.fromhex(hex_data) if dlc > 0 else b"" 

            return can.Message( # type: ignore[name-defined]
                arbitration_id=can_id,
                data=data,
                dlc=dlc,
                is_extended_id=False,
                timestamp=time.time(),
            )
        except (ValueError, IndexError) as e:
            logger.error(f"Error parsing simulator message '{line}': {e}")
            return None

    async def send_message(
        self, msg: can.Message, timeout: float = CAN_TIMEOUT_SECONDS # type: ignore[name-defined]
    ):
        """
        Sends a CAN message to the connected bus or simulator.

        Args:
            msg: The `can.Message` object to send.
            timeout: Timeout in seconds for the send operation.
                     For hardware, this is passed to `bus.send()`.
                     For the simulator, this is used for `writer.drain()`.

        Raises:
            SimulatorError: If not connected to the simulator, or if sending to the simulator fails.
            CANError: If not connected to a hardware CAN bus, or if `bus.send()` fails.
            CommunicationError: If the send operation times out (applies to simulator drain).
        """
        if self.use_simulator:
            if not self._sim_writer or self._sim_writer.is_closing():
                raise SimulatorError("Not connected to simulator or writer is closing.")
            try:
                sim_data = self._can_message_to_sim_protocol(msg)
                logger.info(f"CANInterface: Sending to simulator: {sim_data.decode().strip()}")
                self._sim_writer.write(sim_data)
                await asyncio.wait_for(
                    self._sim_writer.drain(), timeout=timeout
                )
            except asyncio.TimeoutError as exc:
                raise CommunicationError(
                    f"Timeout sending message to simulator: {msg.arbitration_id:03X} {msg.data.hex()}" # type: ignore[union-attr]
                ) from exc
            except ConnectionResetError as exc:
                 raise SimulatorError(f"Simulator connection reset while sending: {exc}") from exc
            except Exception as e: # pylint: disable=broad-except
                raise SimulatorError(
                    f"Failed to send message to simulator: {e}"
                ) from e
        else: # Hardware
            if not self.bus:
                raise CANError("CAN bus not connected.")
            try:
                self.bus.send(msg, timeout=timeout) # type: ignore[union-attr]
                logger.debug(
                    f"Sent on CAN bus: ID={msg.arbitration_id:03X}, DLC={msg.dlc}, Data={' '.join(f'{b:02X}' for b in msg.data)}" # type: ignore[union-attr]
                )
            except can.CanOperationError as e: # type: ignore[name-defined]
                logger.error(
                    f"CAN bus send operation failed: {e}", exc_info=True
                )
                raise CANError(f"CAN bus send operation failed: {e}") from e
            except can.CanError as e: # type: ignore[name-defined]
                logger.error(f"Failed to send CAN message: {e}", exc_info=True)
                raise CANError(f"Failed to send CAN message: {e}") from e

    async def _listen_for_messages_hw(self):
        """
        Internal background task for listening to messages from the hardware CAN bus.

        This task runs in the asyncio event loop. It consumes messages from an
        `asyncio.Queue` which is populated by the `AsyncioCanListener` (which, in turn,
        is driven by the `python-can` Notifier running in a separate thread).
        Received messages are then passed to `_process_received_message`.
        """
        if not self._message_queue:
            logger.error("Hardware listener: Message queue not initialized.")
            self._is_listening = False # Ensure it's marked as not listening
            return
        
        logger.info("Hardware CAN listener started (Notifier-based).")
        try:
            while self._is_listening:
                try:
                    msg = await asyncio.wait_for(self._message_queue.get(), timeout=1.0)
                    if msg:
                        await self._process_received_message(msg)
                    self._message_queue.task_done()
                except asyncio.TimeoutError:
                    # This is normal if no messages are received within the timeout
                    if not self._is_listening: # Check if we should break
                        break
                    continue
                except asyncio.CancelledError: # Handle task cancellation
                    logger.info("Hardware CAN listener task was cancelled.")
                    break 
        except Exception as e: # pylint: disable=broad-except
            if self._is_listening: # Log only if error happened while actively listening
                logger.error(
                    f"Unexpected hardware listener error: {e}", exc_info=True
                )
        finally:
            logger.info("Hardware CAN listener (Notifier-based) stopped.")


    async def _listen_for_messages_sim(self):
        """
        Internal background task for listening to messages from the simulator.

        This task runs in the asyncio event loop, reading lines from the TCP
        connection to the simulator. Each line is parsed using
        `_sim_protocol_to_can_message` and then processed by
        `_process_received_message`.
        """
        if not self._sim_reader:
            logger.error("Simulator listener: Reader not initialized.")
            self._is_listening = False
            return
        logger.info("Simulator listener started.")
        try:
            while self._is_listening:
                if self._sim_reader.at_eof():
                    logger.info("Simulator connection closed by peer (EOF).")
                    break
                try:
                    line_bytes = await asyncio.wait_for(
                        self._sim_reader.readline(), timeout=1.0 
                    )
                    if not line_bytes:
                        logger.info("Simulator connection closed by peer (empty read).")
                        break
                    line_str = line_bytes.decode().strip()
                    if line_str:
                        logger.info(f"CANInterface: Received from simulator: {line_str}")
                        msg = self._sim_protocol_to_can_message(line_str)
                        if msg:
                            await self._process_received_message(msg)
                except asyncio.TimeoutError:
                    if not self._is_listening: break
                    continue 
                except (asyncio.IncompleteReadError, ConnectionResetError) as e:
                    logger.warning(f"Simulator connection issue: {e}. Stopping listener.")
                    break
                except asyncio.CancelledError:
                    logger.info("Simulator listener task was cancelled.")
                    break
        except Exception as e: # pylint: disable=broad-except
            if self._is_listening: 
                logger.error(f"Simulator listener error: {e}", exc_info=True)
        finally:
            logger.info("Simulator listener stopped.")
            if self._is_listening: # If stopped by error, ensure flag is correct
                self._is_listening = False


    async def _process_received_message(self, msg: can.Message): # type: ignore[name-defined]
        """
        Processes a received CAN message (from hardware or simulator).

        It attempts to resolve any pending response futures that match the message's
        CAN ID and command code. If a predicate was associated with the future,
        it's also checked. If no future handles the message, it's passed to
        any general message handlers registered for that CAN ID.

        Args:
            msg: The `can.Message` object that was received.
        """
        cmd_str = f"{msg.data[0]:02X}" if msg.data else "N/A"
        logger.info(
            f"CANInterface: Processing received: ID={msg.arbitration_id:03X}, CMD={cmd_str}, Data={msg.data.hex()}"
        )

        cmd_code = msg.data[0] if msg.data else None
        key_tuple = (msg.arbitration_id, cmd_code)
        
        futures_for_key = self._response_futures.get(key_tuple, [])
        remaining_futures_for_key = []
        resolved_this_message = False

        for future, predicate in futures_for_key:
            if not future.done():
                if predicate: # Custom predicate provided
                    if predicate(msg):
                        future.set_result(msg)
                        resolved_this_message = True
                        # If a message satisfies a specific predicate, we assume it's consumed by that future.
                        # Depending on desired behavior, we might break or continue checking other futures
                        # for the same (ID, CMD) if multiple predicates could match.
                        # For now, if predicate matches, it's consumed by this future.
                    else:
                        remaining_futures_for_key.append((future, predicate)) # Keep if predicate didn't match
                elif not resolved_this_message: # No predicate, and message not yet resolved by another predicate-less future for this key
                    future.set_result(msg)
                    resolved_this_message = True # Mark that a generic future for this ID/CMD resolved it
                else: # Has no predicate but message already resolved by another generic future, keep it if not done
                    remaining_futures_for_key.append((future, predicate))


        if remaining_futures_for_key:
            self._response_futures[key_tuple] = remaining_futures_for_key
        elif key_tuple in self._response_futures: # List became empty
            del self._response_futures[key_tuple]
        
        if resolved_this_message:
             return # Message handled by a future

        # If not handled by a future, try general message handlers
        if msg.arbitration_id in self._message_handlers:
            for handler in self._message_handlers[msg.arbitration_id]:
                try:
                    if asyncio.iscoroutinefunction(handler):
                        self._loop.create_task(handler(msg))
                    else:
                        handler(msg)
                except Exception as e: # pylint: disable=broad-except
                    logger.error(
                        f"Error in message handler for ID {msg.arbitration_id:03X}: {e}",
                        exc_info=True,
                    )

    def add_message_handler(
        self, can_id: int, handler: Callable[[can.Message], Any] # type: ignore[name-defined]
    ):
        """
        Registers a handler function to be called for any message received with a specific CAN ID.

        This is for general, unsolicited message handling, not for specific request-response pairs
        (which use response futures).

        Args:
            can_id: The CAN ID to listen for.
            handler: A callable (function or coroutine function) that takes a `can.Message`
                     as its argument.
        """
        if can_id not in self._message_handlers:
            self._message_handlers[can_id] = []
        if handler not in self._message_handlers[can_id]: # Avoid duplicate handlers
            self._message_handlers[can_id].append(handler)
            logger.info(f"Added handler for CAN ID {can_id:03X}.")

    def remove_message_handler(
        self, can_id: int, handler: Callable[[can.Message], Any] # type: ignore[name-defined]
    ):
        """
        Removes a previously registered message handler for a specific CAN ID.

        Args:
            can_id: The CAN ID the handler was registered for.
            handler: The handler function to remove.
        """
        if (
            can_id in self._message_handlers
            and handler in self._message_handlers[can_id]
        ):
            self._message_handlers[can_id].remove(handler)
            if not self._message_handlers[can_id]: # If list is empty, remove key
                del self._message_handlers[can_id]
            logger.info(f"Removed handler for CAN ID {can_id:03X}.")

    def create_response_future(
        self, can_id: int, command_code: int,
        response_predicate: Optional[Callable[[can.Message], bool]] = None # type: ignore[name-defined]
    ) -> asyncio.Future:
        """
        Creates and stores an `asyncio.Future` that will be resolved when a message
        matching the given `can_id` and `command_code` is received.

        An optional `response_predicate` can be provided for more fine-grained matching
        of the response message. This is used by `send_and_wait_for_response`.

        Args:
            can_id: The expected CAN ID of the response message.
            command_code: The expected command code (first data byte) in the response message.
            response_predicate: An optional callable that takes a `can.Message` and returns
                                True if it's the desired specific response, False otherwise.

        Returns:
            An `asyncio.Future` object that the caller can await.
        """
        key_tuple = (can_id, command_code)
        
        # Clean up any old, done futures for this key_tuple before adding new one
        if key_tuple in self._response_futures:
            self._response_futures[key_tuple] = [
                (f, p) for f, p in self._response_futures[key_tuple] if not f.done()
            ]
            if not self._response_futures[key_tuple]: # If list became empty
                del self._response_futures[key_tuple]

        future = self._loop.create_future()
        
        if key_tuple not in self._response_futures:
            self._response_futures[key_tuple] = []
            
        self._response_futures[key_tuple].append((future, response_predicate))
        
        logger.debug(f"Created future for CAN ID {can_id:03X}, CMD {command_code:02X}{' with predicate' if response_predicate else ''}. Total waiters for key: {len(self._response_futures[key_tuple])}")
        return future

    def start_listening(self):
        """
        Starts the background message listening task appropriate for the mode (hardware or simulator).

        If not already listening and connected, it creates and schedules the listener task.
        """
        if not self.is_connected: # Check connection status
            logger.warning("Cannot start listener: CANInterface not connected.")
            return

        if not self._is_listening:
            self._is_listening = True
            if self.use_simulator:
                if not self._sim_reader or not self._sim_writer or self._sim_writer.is_closing():
                    logger.warning(
                        "Cannot start simulator listener: not connected or writer closed."
                    )
                    self._is_listening = False
                    return
                self._listener_task = self._loop.create_task(
                    self._listen_for_messages_sim()
                )
            else: # Hardware
                if not self.bus or not self._notifier: # Check bus and notifier for hardware
                    logger.warning(
                        "Cannot start hardware listener: not connected or notifier not initialized."
                    )
                    self._is_listening = False
                    return
                # Notifier is started implicitly when created with can.Notifier(bus, listeners)
                # The listener task here is for processing messages from the queue populated by the notifier.
                self._listener_task = self._loop.create_task(
                    self._listen_for_messages_hw()
                )
            logger.info("Message listener initiated.")
        else:
            logger.info("Message listener already running.")

    def stop_listening(self):
        """
        Signals the background message listening task to stop and cancels it if running.
        """
        if self._is_listening:
            self._is_listening = False 
            if self._listener_task and not self._listener_task.done():
                self._listener_task.cancel()
            # For hardware, notifier is stopped in disconnect()
            logger.info("Message listener stop signalled.")
        # self._listener_task = None # Clearing here might be premature if disconnect needs to await it. Let disconnect handle final clear.

    async def send_and_wait_for_response(
        self,
        msg_to_send: can.Message, # type: ignore[name-defined]
        expected_response_can_id: int,
        expected_response_command_code: int,
        timeout: float = CAN_TIMEOUT_SECONDS,
        response_predicate: Optional[Callable[[can.Message], bool]] = None # type: ignore[name-defined]
    ) -> can.Message: # type: ignore[name-defined]
        """
        Sends a CAN message and waits for a specific response from the bus/simulator.

        This is a core method for request-response communication patterns.
        It uses `create_response_future` to set up an expectation for a particular
        response (matching CAN ID and command code, plus optional predicate).

        Args:
            msg_to_send: The `can.Message` to send.
            expected_response_can_id: The CAN ID expected in the response message.
            expected_response_command_code: The command code (first data byte) expected
                                            in the response message.
            timeout: Maximum time in seconds to wait for the response after sending.
            response_predicate: An optional callable to further filter incoming messages
                                for a more specific match.

        Returns:
            The received `can.Message` object that matches the criteria.

        Raises:
            CommunicationError: If not connected, if the send operation fails,
                                if the wait times out, or if the wait is cancelled.
        """
        if not self.is_connected:
            raise CommunicationError("Cannot send message: Not connected.")
        if not self._is_listening:
            logger.warning("Listener not active when send_and_wait_for_response called. Starting it.")
            self.start_listening()
            if not self._is_listening: # If start_listening failed
                 raise CommunicationError("Failed to start listener for send_and_wait_for_response.")


        future = self.create_response_future(
            expected_response_can_id, expected_response_command_code, response_predicate
        )

        try:
            await self.send_message(msg_to_send, timeout=timeout)
        except Exception: 
            if not future.done():
                future.cancel("Send operation failed")
            raise # Re-raise the send error

        try:
            response_msg = await asyncio.wait_for(future, timeout=timeout)
            return response_msg
        except asyncio.TimeoutError as exc:
            # Clean up the specific future if it's still in the list and is this one
            key = (expected_response_can_id, expected_response_command_code)
            if key in self._response_futures:
                self._response_futures[key] = [(f, p) for f, p in self._response_futures[key] if f is not future or f.done()]
                if not self._response_futures[key]:
                    del self._response_futures[key]
            
            cmd_byte_str = f"{msg_to_send.data[0]:02X}" if msg_to_send.data else "N/A"
            payload_str = msg_to_send.data.hex() if msg_to_send.data else "N/A"
            error_msg = (
                f"Timeout waiting for response to command {cmd_byte_str} "
                f"from CAN ID {expected_response_can_id:03X} (expected echoed cmd code "
                f"{expected_response_command_code:02X}). Sent: {payload_str}"
            )
            logger.warning(error_msg)
            raise CommunicationError(error_msg) from exc
        except asyncio.CancelledError as exc:
            error_msg = (f"Response wait for CAN ID {expected_response_can_id:03X}, "
                         f"Cmd {expected_response_command_code:02X} was cancelled.")
            logger.warning(error_msg)
            raise CommunicationError(error_msg) from exc

    @property
    def is_connected(self) -> bool:
        """
        Checks if the CAN interface is currently connected and operational.

        For simulator mode, it checks if the stream writer is active and not closing.
        For hardware mode, it primarily checks if the `python-can` bus object exists
        and, if `python-can` is fully available, that the notifier is also set up
        (implying a successful `connect()` call).

        Returns:
            True if the interface is considered connected, False otherwise.
        """
        if self.use_simulator:
            return (
                self._sim_writer is not None
                and not self._sim_writer.is_closing()
            )
        else: # Hardware
            # Consider bus to be connected if bus object exists AND notifier is running (or configured)
            # self._notifier might be None if CAN_AVAILABLE is False even if self.bus exists (dummy)
            if CAN_AVAILABLE and self._notifier:
                 # A more robust check might involve querying notifier's thread status if possible,
                 # or relying on successful bus initialization.
                 return self.bus is not None
            return self.bus is not None # Basic check if notifier isn't used or as fallback
        