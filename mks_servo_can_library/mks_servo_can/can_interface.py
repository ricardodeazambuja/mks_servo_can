"""
CAN Communication Layer for MKS Servo Control.
Handles raw CAN bus communication using python-can for real hardware
and provides a "virtual" backend for the simulator.
"""
from typing import Any, Callable, Dict, List, Optional

import asyncio
import logging
import time

try:
    import can

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
                self.data = data if data is not None else b"" # Changed to b""
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
            CanError
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


class CANInterface:
    """
    Manages CAN communication, supporting both real hardware and a simulator.
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
            interface_type: Type of CAN interface (e.g., 'canable', 'kvaser', 'socketcan').
                            Only used if use_simulator is False.
            channel: Channel specific to the interface (e.g., 'slcan0', '/dev/ttyUSB0').
                     Only used if use_simulator is False.
            bitrate: CAN bus bitrate (e.g., 125000, 250000, 500000, 1000000).
                     Only used if use_simulator is False.
            simulator_host: Hostname for the simulator. Used if use_simulator is True.
            simulator_port: Port for the simulator. Used if use_simulator is True.
            use_simulator: If True, connects to a simulator instead of real hardware.
            loop: asyncio event loop.
        """
        self.use_simulator = use_simulator
        self.bus: Optional[can.BusABC] = None # type: ignore
        self._loop = loop if loop else asyncio.get_event_loop()
        self._message_handlers: Dict[
            int, List[Callable[[can.Message], None]]
        ] = {}  # CAN ID -> list of handlers
        self._response_futures: Dict[int, Dict[int, asyncio.Future]] = (
            {}
        )  # can_id -> {command_code: Future}
        self._is_listening = False
        self._listener_task: Optional[asyncio.Task] = None

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
        """Establishes connection to the CAN bus or simulator."""
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
        else:
            if not self.channel and self.interface_type not in [
                "virtual"
            ]:
                raise ConfigurationError(
                    f"Channel must be specified for {self.interface_type} interface."
                )
            try:
                logger.info(
                    f"Attempting to connect to CAN hardware: type={self.interface_type}, channel={self.channel}, bitrate={self.bitrate}"
                )
                self.bus = can.interface.Bus( # type: ignore
                    bustype=self.interface_type,
                    channel=self.channel,
                    bitrate=self.bitrate,
                )
                logger.info(
                    f"Successfully connected to CAN bus: {self.bus.channel_info if hasattr(self.bus, 'channel_info') else 'N/A'}"
                )
            except can.CanError as e:
                logger.error(
                    f"Failed to connect to CAN bus: {e}", exc_info=True
                )
                raise CANError(
                    f"Failed to initialize CAN bus ({self.interface_type} on {self.channel}): {e}"
                ) from e
            except Exception as e:
                logger.error(
                    f"An unexpected error occurred during CAN bus connection: {e}",
                    exc_info=True,
                )
                raise CANError(
                    f"Unexpected error connecting to CAN bus: {e}"
                ) from e

        self.start_listening()

    async def disconnect(self):
        """Disconnects from the CAN bus or simulator."""
        self.stop_listening() # Ensure listener task is stopped first
        # Wait for listener task to finish if it was running
        if self._listener_task and not self._listener_task.done():
            try:
                await asyncio.wait_for(self._listener_task, timeout=1.0)
            except asyncio.TimeoutError:
                logger.warning("Listener task did not finish gracefully on disconnect timeout.")
            except asyncio.CancelledError:
                logger.info("Listener task cancelled on disconnect.")


        if self.use_simulator:
            if self._sim_writer:
                try:
                    if not self._sim_writer.is_closing():
                        self._sim_writer.close()
                        await self._sim_writer.wait_closed()
                    logger.info("Disconnected from simulator.")
                except Exception as e:
                    logger.warning(f"Error during simulator disconnection: {e}")
                finally:
                    self._sim_reader = None
                    self._sim_writer = None
        else:
            if self.bus:
                try:
                    self.bus.shutdown()
                    logger.info("Disconnected from CAN bus.")
                except can.CanError as e:
                    logger.warning(f"Error during CAN bus shutdown: {e}")
                finally:
                    self.bus = None
        self._message_handlers.clear()
        # Cancel and clear pending futures
        for can_id_futures in self._response_futures.values():
            for future in can_id_futures.values():
                if not future.done():
                    future.cancel("CANInterface disconnecting")
        self._response_futures.clear()


    def _can_message_to_sim_protocol(self, msg: can.Message) -> bytes:
        """Converts a python-can Message object to a simulator protocol string/bytes."""
        if msg.dlc == 0:
            data_str = ""
        else:
            data_str = "".join(f"{b:02X}" for b in msg.data)
        return f"SIM_CAN_SEND {msg.arbitration_id:03X} {msg.dlc} {data_str}\n".encode()

    def _sim_protocol_to_can_message(self, line: str) -> Optional[can.Message]:
        """Converts a simulator protocol string/bytes to a python-can Message object."""
        parts = line.strip().split()
        if not parts or parts[0] != "SIM_CAN_RECV" or len(parts) < 3: # DLC can be 0, so data is optional
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

            data = bytes.fromhex(hex_data) if dlc > 0 else b"" # Use b"" for empty data

            return can.Message(
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
        self, msg: can.Message, timeout: float = CAN_TIMEOUT_SECONDS
    ):
        """
        Sends a CAN message.
        """
        if self.use_simulator:
            if not self._sim_writer or self._sim_writer.is_closing():
                raise SimulatorError("Not connected to simulator or writer is closing.")
            try:
                sim_data = self._can_message_to_sim_protocol(msg)
                # INFO level log added here:
                logger.info(f"CANInterface: Sending to simulator: {sim_data.decode().strip()}")
                self._sim_writer.write(sim_data)
                await asyncio.wait_for(
                    self._sim_writer.drain(), timeout=timeout
                )
            except asyncio.TimeoutError as exc:
                raise CommunicationError(
                    f"Timeout sending message to simulator: {msg.arbitration_id:03X} {msg.data.hex()}"
                ) from exc
            except ConnectionResetError as exc:
                 raise SimulatorError(f"Simulator connection reset while sending: {exc}") from exc
            except Exception as e:
                raise SimulatorError(
                    f"Failed to send message to simulator: {e}"
                ) from e
        else:
            if not self.bus:
                raise CANError("CAN bus not connected.")
            try:
                self.bus.send(msg, timeout=timeout)
                logger.debug(
                    f"Sent on CAN bus: ID={msg.arbitration_id:03X}, DLC={msg.dlc}, Data={' '.join(f'{b:02X}' for b in msg.data)}"
                )
            except can.CanOperationError as e:
                logger.error(
                    f"CAN bus send operation failed: {e}", exc_info=True
                )
                raise CANError(f"CAN bus send operation failed: {e}") from e
            except can.CanError as e:
                logger.error(f"Failed to send CAN message: {e}", exc_info=True)
                raise CANError(f"Failed to send CAN message: {e}") from e

    async def _listen_for_messages_hw(self):
        """Internal task for listening to messages from hardware CAN bus."""
        if not self.bus:
            logger.error("Hardware listener: Bus not initialized.")
            return
        logger.info("Hardware CAN listener started.")
        try:
            while self._is_listening:
                msg = self.bus.recv(timeout=0.1)
                if msg:
                    await self._process_received_message(msg)
                else: # Yield control if no message
                    await asyncio.sleep(0.001)
        except can.CanError as e:
            if self._is_listening:
                logger.error(f"CAN hardware listener error: {e}", exc_info=True)
        except Exception as e:
            if self._is_listening:
                logger.error(
                    f"Unexpected hardware listener error: {e}", exc_info=True
                )
        finally:
            logger.info("Hardware CAN listener stopped.")

    async def _listen_for_messages_sim(self):
        """Internal task for listening to messages from the simulator."""
        if not self._sim_reader:
            logger.error("Simulator listener: Reader not initialized.")
            return
        logger.info("Simulator listener started.")
        try:
            while self._is_listening:
                if self._sim_reader.at_eof():
                    logger.info("Simulator connection closed by peer (EOF).")
                    break
                try:
                    line_bytes = await asyncio.wait_for(
                        self._sim_reader.readline(), timeout=1.0 # Keep timeout for readline
                    )
                    if not line_bytes:
                        logger.info("Simulator connection closed by peer (empty read).")
                        break
                    line_str = line_bytes.decode().strip()
                    if line_str:
                        # INFO level log for received simulator data
                        logger.info(f"CANInterface: Received from simulator: {line_str}")
                        msg = self._sim_protocol_to_can_message(line_str)
                        if msg:
                            await self._process_received_message(msg)
                except asyncio.TimeoutError:
                    continue # Normal if no messages
                except (asyncio.IncompleteReadError, ConnectionResetError) as e:
                    logger.warning(f"Simulator connection issue: {e}. Stopping listener.")
                    break
        except Exception as e:
            if self._is_listening: # Log only if error happened while actively listening
                logger.error(f"Simulator listener error: {e}", exc_info=True)
        finally:
            logger.info("Simulator listener stopped.")
            if self._is_listening:
                self._is_listening = False # Ensure state is correct if stopped unexpectedly
                # Consider further action if it stopped unexpectedly, e.g. reconnect attempt or error propagation


    async def _process_received_message(self, msg: can.Message):
        """Processes a received CAN message and dispatches it."""
        # Corrected f-string for logging CMD
        cmd_str = f"{msg.data[0]:02X}" if msg.data else "N/A"
        logger.info(
            f"CANInterface: Processing received: ID={msg.arbitration_id:03X}, CMD={cmd_str}, Data={msg.data.hex()}"
        )

        cmd_code = msg.data[0] if msg.data else None

        if (
            msg.arbitration_id in self._response_futures
            and cmd_code is not None # Ensure cmd_code is not None before dict lookup
            and cmd_code in self._response_futures[msg.arbitration_id]
        ):
            future = self._response_futures[msg.arbitration_id].pop(cmd_code)
            if not future.done():
                future.set_result(msg)
            else:
                logger.warning(f"Future for ID {msg.arbitration_id:03X} Cmd {cmd_code:02X} was already done.")
            if not self._response_futures[msg.arbitration_id]:
                del self._response_futures[msg.arbitration_id]
            return

        if msg.arbitration_id in self._message_handlers:
            for handler in self._message_handlers[msg.arbitration_id]:
                try:
                    if asyncio.iscoroutinefunction(handler):
                        self._loop.create_task(handler(msg))
                    else:
                        handler(msg)
                except Exception as e:
                    logger.error(
                        f"Error in message handler for ID {msg.arbitration_id:03X}: {e}",
                        exc_info=True,
                    )

    def add_message_handler(
        self, can_id: int, handler: Callable[[can.Message], Any]
    ):
        """Registers a handler function for messages with a specific CAN ID."""
        if can_id not in self._message_handlers:
            self._message_handlers[can_id] = []
        if handler not in self._message_handlers[can_id]:
            self._message_handlers[can_id].append(handler)
            logger.info(f"Added handler for CAN ID {can_id:03X}.")

    def remove_message_handler(
        self, can_id: int, handler: Callable[[can.Message], Any]
    ):
        """Removes a specific handler for a CAN ID."""
        if (
            can_id in self._message_handlers
            and handler in self._message_handlers[can_id]
        ):
            self._message_handlers[can_id].remove(handler)
            if not self._message_handlers[can_id]:
                del self._message_handlers[can_id]
            logger.info(f"Removed handler for CAN ID {can_id:03X}.")

    def create_response_future(
        self, can_id: int, command_code: int
    ) -> asyncio.Future:
        """Creates and stores a future for an expected response."""
        if can_id not in self._response_futures:
            self._response_futures[can_id] = {}

        if (
            command_code in self._response_futures[can_id]
            and not self._response_futures[can_id][command_code].done()
        ):
            logger.warning(
                f"Overwriting an existing pending future for CAN ID {can_id:03X}, Cmd {command_code:02X}."
            )
            self._response_futures[can_id][command_code].cancel("Superseded")

        future = self._loop.create_future()
        self._response_futures[can_id][command_code] = future
        return future

    def start_listening(self):
        """Starts the background message listening task."""
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
            else:
                if not self.bus:
                    logger.warning(
                        "Cannot start hardware listener: not connected."
                    )
                    self._is_listening = False
                    return
                self._listener_task = self._loop.create_task(
                    self._listen_for_messages_hw()
                )
            logger.info("Message listener initiated.")
        else:
            logger.info("Message listener already running.")

    def stop_listening(self):
        """Stops the background message listening task."""
        if self._is_listening:
            self._is_listening = False # Signal listener to stop
            if self._listener_task and not self._listener_task.done():
                self._listener_task.cancel()
                # Do not await here, let disconnect handle awaiting if necessary
            self._listener_task = None # Clear task reference
            logger.info("Message listener stop signalled.")


    async def send_and_wait_for_response(
        self,
        msg_to_send: can.Message,
        expected_response_can_id: int,
        expected_response_command_code: int,
        timeout: float = CAN_TIMEOUT_SECONDS,
    ) -> can.Message:
        """Sends a message and waits for a specific response."""
        if not self._is_listening:
            logger.warning("Listener not active when send_and_wait_for_response called. Starting it.")
            self.start_listening()


        future = self.create_response_future(
            expected_response_can_id, expected_response_command_code
        )

        try:
            await self.send_message(msg_to_send, timeout=timeout)
        except Exception: 
            if not future.done():
                future.cancel("Send operation failed")
            raise

        try:
            response_msg = await asyncio.wait_for(future, timeout=timeout)
            return response_msg
        except asyncio.TimeoutError as exc:
            if future and not future.done(): 
                 if (
                    expected_response_can_id in self._response_futures and
                    expected_response_command_code in self._response_futures[expected_response_can_id] and
                    self._response_futures[expected_response_can_id][expected_response_command_code] is future
                 ):
                    del self._response_futures[expected_response_can_id][expected_response_command_code]
                    if not self._response_futures[expected_response_can_id]:
                        del self._response_futures[expected_response_can_id]

            error_msg = (
                f"Timeout waiting for response to command {msg_to_send.data[0]:02X if msg_to_send.data else 'N/A'} "
                f"from CAN ID {expected_response_can_id:03X} (expected echoed cmd code "
                f"{expected_response_command_code:02X}). Sent: {msg_to_send.data.hex() if msg_to_send.data else 'N/A'}"
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
        Checks if the CAN interface is currently connected.

        For simulator mode, it checks if the stream writer is active and not closing.
        For hardware mode, it checks if the python-can bus object exists.

        Returns:
            True if connected, False otherwise.
        """
        if self.use_simulator:
            return (
                self._sim_writer is not None
                and not self._sim_writer.is_closing()
            )
        else:
            return self.bus is not None
        