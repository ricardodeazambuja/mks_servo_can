# mks_servo_can/mks_servo_can_library/mks_servo_can/can_interface.py
"""
CAN Communication Layer for MKS Servo Control.
Handles raw CAN bus communication using python-can for real hardware
and provides a "virtual" backend for the simulator.
"""
import asyncio
import time
import logging
from typing import Optional, Callable, Any, List, Dict, Union

try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False
    # Create dummy can classes/exceptions if can is not installed for basic loading
    class can:
        class BusABC:
            pass
        class Message:
            def __init__(self, arbitration_id=0, data=None, is_extended_id=False, timestamp=0.0):
                self.arbitration_id = arbitration_id
                self.data = data if data is not None else []
                self.dlc = len(self.data)
                self.is_extended_id = is_extended_id
                self.timestamp = timestamp
                self.channel = None
                self.is_rx = True # Assume received
                self.is_error_frame = False
                self.is_remote_frame = False

        class CanError(Exception):
            pass
        class CanOperationError(CanError): # Added for more specific error handling
            pass


from .exceptions import CANError, CommunicationError, SimulatorError, ConfigurationError
from .constants import CAN_DEFAULT_BITRATE, CAN_TIMEOUT_SECONDS, SIM_DEFAULT_LATENCY_MS

logger = logging.getLogger(__name__)

class CANInterface:
    """
    Manages CAN communication, supporting both real hardware and a simulator.
    """
    def __init__(self,
                 interface_type: str = 'canable',
                 channel: Optional[str] = None,
                 bitrate: int = CAN_DEFAULT_BITRATE,
                 simulator_host: str = 'localhost',
                 simulator_port: int = 6789,
                 use_simulator: bool = False,
                 loop: Optional[asyncio.AbstractEventLoop] = None):
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
        self.bus = None
        self._loop = loop if loop else asyncio.get_event_loop()
        self._message_handlers: Dict[int, List[Callable[[can.Message], None]]] = {} # CAN ID -> list of handlers
        self._response_futures: Dict[int, Dict[int, asyncio.Future]] = {} # can_id -> {command_code: Future}
        self._is_listening = False
        self._listener_task: Optional[asyncio.Task] = None

        if self.use_simulator:
            self.simulator_host = simulator_host
            self.simulator_port = simulator_port
            self._sim_reader: Optional[asyncio.StreamReader] = None
            self._sim_writer: Optional[asyncio.StreamWriter] = None
            logger.info(f"CANInterface configured to use simulator at {simulator_host}:{simulator_port}")
        else:
            if not CAN_AVAILABLE:
                raise ConfigurationError("python-can library is not installed. Please install it to use real hardware.")
            self.interface_type = interface_type
            self.channel = channel
            self.bitrate = bitrate
            logger.info(f"CANInterface configured for hardware: {interface_type} on {channel} @ {bitrate} bps")

    async def connect(self):
        """Establishes connection to the CAN bus or simulator."""
        if self.use_simulator:
            try:
                logger.info(f"Attempting to connect to simulator at {self.simulator_host}:{self.simulator_port}...")
                self._sim_reader, self._sim_writer = await asyncio.open_connection(
                    self.simulator_host, self.simulator_port
                )
                logger.info("Successfully connected to simulator.")
                # Send a handshake or initial message if required by simulator protocol
                # self._sim_writer.write(b"CONNECT_LIB\n")
                # await self._sim_writer.drain()
            except ConnectionRefusedError as e:
                raise SimulatorError(f"Connection to simulator refused at {self.simulator_host}:{self.simulator_port}. Is it running?") from e
            except Exception as e:
                raise SimulatorError(f"Failed to connect to simulator: {e}") from e
        else:
            if not self.channel and self.interface_type not in ['virtual']: # Some interfaces might not need a channel
                 raise ConfigurationError(f"Channel must be specified for {self.interface_type} interface.")
            try:
                logger.info(f"Attempting to connect to CAN hardware: type={self.interface_type}, channel={self.channel}, bitrate={self.bitrate}")
                self.bus = can.interface.Bus(bustype=self.interface_type, channel=self.channel, bitrate=self.bitrate)
                logger.info(f"Successfully connected to CAN bus: {self.bus.channel_info if hasattr(self.bus, 'channel_info') else 'N/A'}")
            except can.CanError as e:
                logger.error(f"Failed to connect to CAN bus: {e}", exc_info=True)
                raise CANError(f"Failed to initialize CAN bus ({self.interface_type} on {self.channel}): {e}") from e
            except Exception as e: # Catch other potential errors like serial port issues for slcan
                logger.error(f"An unexpected error occurred during CAN bus connection: {e}", exc_info=True)
                raise CANError(f"Unexpected error connecting to CAN bus: {e}") from e
        
        self.start_listening()

    async def disconnect(self):
        """Disconnects from the CAN bus or simulator."""
        self.stop_listening()
        if self.use_simulator:
            if self._sim_writer:
                try:
                    # self._sim_writer.write(b"DISCONNECT_LIB\n") # Optional: send disconnect signal
                    # await self._sim_writer.drain()
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
        self._response_futures.clear()


    def _can_message_to_sim_protocol(self, msg: can.Message) -> bytes:
        """Converts a python-can Message object to a simulator protocol string/bytes."""
        # Example protocol: "CAN <id_hex> <data_hex_space_separated>\n"
        # e.g., "CAN 01 3031\n"
        # Ensure data is at least two bytes as per manual for cmd + crc
        if msg.dlc == 0: # Should not happen for MKS commands
            data_str = ""
        else:
            data_str = "".join(f"{b:02X}" for b in msg.data)
        
        # Simulator expects: arbitration_id (hex), dlc (int), data (hex string)
        return f"SIM_CAN_SEND {msg.arbitration_id:03X} {msg.dlc} {data_str}\n".encode()

    def _sim_protocol_to_can_message(self, line: str) -> Optional[can.Message]:
        """Converts a simulator protocol string/bytes to a python-can Message object."""
        # Example protocol: "CAN_RESP <id_hex> <dlc_int> <data_hex_no_space>\n"
        # e.g., "CAN_RESP 001 8 3000000000000031\n"
        parts = line.strip().split()
        if not parts or parts[0] != "SIM_CAN_RECV":
            logger.warning(f"Received unknown or malformed data from simulator: {line}")
            return None
        try:
            can_id = int(parts[1], 16)
            dlc = int(parts[2])
            hex_data = parts[3] if len(parts) > 3 else ""

            if len(hex_data) != dlc * 2:
                logger.error(f"Simulator DLC mismatch: expected {dlc*2} hex chars, got {len(hex_data)} for data '{hex_data}'")
                return None # Or raise error

            data = bytes.fromhex(hex_data) if dlc > 0 else []
            
            return can.Message(
                arbitration_id=can_id,
                data=data,
                dlc=dlc,
                is_extended_id=False, # MKS Servos use standard IDs
                timestamp=time.time() # Simulate timestamp
            )
        except (ValueError, IndexError) as e:
            logger.error(f"Error parsing simulator message '{line}': {e}")
            return None

    async def send_message(self, msg: can.Message, timeout: float = CAN_TIMEOUT_SECONDS):
        """
        Sends a CAN message.

        Args:
            msg: The can.Message object to send.
            timeout: Timeout for the send operation (primarily for simulator).

        Raises:
            CANError: If not connected or send fails on hardware.
            SimulatorError: If send fails on simulator.
            CommunicationError: If timeout occurs.
        """
        if self.use_simulator:
            if not self._sim_writer:
                raise SimulatorError("Not connected to simulator.")
            try:
                sim_data = self._can_message_to_sim_protocol(msg)
                logger.debug(f"Sending to simulator: {sim_data.strip()}")
                self._sim_writer.write(sim_data)
                await asyncio.wait_for(self._sim_writer.drain(), timeout=timeout)
            except asyncio.TimeoutError:
                raise CommunicationError(f"Timeout sending message to simulator.")
            except Exception as e:
                raise SimulatorError(f"Failed to send message to simulator: {e}") from e
        else:
            if not self.bus:
                raise CANError("CAN bus not connected.")
            try:
                self.bus.send(msg, timeout=timeout) # python-can's timeout
                logger.debug(f"Sent on CAN bus: ID={msg.arbitration_id:03X}, DLC={msg.dlc}, Data={' '.join(f'{b:02X}' for b in msg.data)}")
            except can.CanOperationError as e: # More specific error from python-can
                logger.error(f"CAN bus send operation failed: {e}", exc_info=True)
                raise CANError(f"CAN bus send operation failed: {e}") from e
            except can.CanError as e: # General python-can error
                logger.error(f"Failed to send CAN message: {e}", exc_info=True)
                raise CANError(f"Failed to send CAN message: {e}") from e


    async def _listen_for_messages_hw(self):
        """Internal task for listening to messages from hardware CAN bus."""
        if not self.bus:
            logger.error("Hardware listener: Bus not initialized.")
            return
        logger.info("Hardware CAN listener started.")
        try:
            reader = can.AsyncBufferedReader()
            # The Notifier is generally more robust for threaded/async scenarios
            # but for a simple async loop, iterating over the bus can work if non-blocking.
            # However, bus itself is blocking. So, using a Notifier with an asyncio bridge is better.
            # For simplicity here, we'll assume a non-blocking recv or use a short timeout.
            # A proper implementation might use can.Notifier with an asyncio.Queue.
            # This is a simplified version. For robust non-blocking, use `can.Notifier`
            # and an `asyncio.Queue` to bridge the threaded notifier to the asyncio loop.
            
            # Simplified approach (might block if bus.recv() is blocking without timeout):
            # For truly async, `python-can` recommends using a Notifier in a separate thread
            # and an `asyncio.Queue` to pass messages to the asyncio loop.
            # This is a placeholder for such a robust implementation.
            while self._is_listening:
                msg = self.bus.recv(timeout=0.1) # Non-blocking or short timeout
                if msg:
                    await self._process_received_message(msg)
                await asyncio.sleep(0.001) # Yield control
        except can.CanError as e:
            if self._is_listening: # Avoid error if stopping
                logger.error(f"CAN hardware listener error: {e}", exc_info=True)
                # Potentially try to reconnect or signal critical failure
        except Exception as e:
            if self._is_listening:
                logger.error(f"Unexpected hardware listener error: {e}", exc_info=True)
        finally:
            logger.info("Hardware CAN listener stopped.")


    async def _listen_for_messages_sim(self):
        """Internal task for listening to messages from the simulator."""
        if not self._sim_reader:
            logger.error("Simulator listener: Reader not initialized.")
            return
        logger.info("Simulator listener started.")
        try:
            while self._is_listening and not self._sim_reader.at_eof():
                try:
                    # Read up to a newline, common for text-based protocols
                    line_bytes = await asyncio.wait_for(self._sim_reader.readline(), timeout=1.0)
                    if not line_bytes: # EOF
                        logger.info("Simulator connection closed by peer.")
                        break
                    line_str = line_bytes.decode().strip()
                    if line_str:
                        logger.debug(f"Received from simulator: {line_str}")
                        msg = self._sim_protocol_to_can_message(line_str)
                        if msg:
                            await self._process_received_message(msg)
                except asyncio.TimeoutError:
                    continue # Just a timeout waiting for a line, continue listening
                except ConnectionResetError:
                    logger.warning("Simulator connection reset.")
                    break # Stop listening if connection is gone
        except Exception as e:
            if self._is_listening:
                logger.error(f"Simulator listener error: {e}", exc_info=True)
        finally:
            logger.info("Simulator listener stopped.")
            if self._is_listening: # If it stopped unexpectedly
                self._is_listening = False # Ensure state is correct
                # Consider attempting to reconnect or notifying higher levels

    async def _process_received_message(self, msg: can.Message):
        """Processes a received CAN message and dispatches it."""
        logger.debug(f"Processing: ID={msg.arbitration_id:03X}, DLC={msg.dlc}, Data={' '.join(f'{b:02X}' for b in msg.data)}")
        
        # Check for command-specific futures first
        # Assuming command code is the first byte of data for MKS
        cmd_code = msg.data[0] if msg.data else None

        if msg.arbitration_id in self._response_futures and \
           cmd_code in self._response_futures[msg.arbitration_id]:
            future = self._response_futures[msg.arbitration_id].pop(cmd_code)
            if not future.done():
                future.set_result(msg)
            # Clean up dict if empty
            if not self._response_futures[msg.arbitration_id]:
                del self._response_futures[msg.arbitration_id]
            return # Future handled, no need for general handlers for this specific response

        # General handlers
        if msg.arbitration_id in self._message_handlers:
            for handler in self._message_handlers[msg.arbitration_id]:
                try:
                    # If handler is async, schedule it
                    if asyncio.iscoroutinefunction(handler):
                        self._loop.create_task(handler(msg))
                    else: # If it's a sync function, run it (careful about blocking the loop)
                          # Consider running sync handlers in an executor if they are long
                        handler(msg)
                except Exception as e:
                    logger.error(f"Error in message handler for ID {msg.arbitration_id}: {e}", exc_info=True)

    def add_message_handler(self, can_id: int, handler: Callable[[can.Message], Any]):
        """
        Registers a handler function for messages with a specific CAN ID.
        
        Args:
            can_id: The CAN ID to listen for.
            handler: A callable (sync or async) that accepts a can.Message object.
        """
        if can_id not in self._message_handlers:
            self._message_handlers[can_id] = []
        if handler not in self._message_handlers[can_id]:
            self._message_handlers[can_id].append(handler)
            logger.info(f"Added handler for CAN ID {can_id:03X}.")

    def remove_message_handler(self, can_id: int, handler: Callable[[can.Message], Any]):
        """Removes a specific handler for a CAN ID."""
        if can_id in self._message_handlers and handler in self._message_handlers[can_id]:
            self._message_handlers[can_id].remove(handler)
            if not self._message_handlers[can_id]: # If list is empty, remove key
                del self._message_handlers[can_id]
            logger.info(f"Removed handler for CAN ID {can_id:03X}.")

    def create_response_future(self, can_id: int, command_code: int) -> asyncio.Future:
        """
        Creates and stores a future for an expected response to a specific command.
        
        Args:
            can_id: The CAN ID from which the response is expected.
            command_code: The command code byte of the expected response.
        
        Returns:
            asyncio.Future: A future that will be resolved with the response message.
        """
        if can_id not in self._response_futures:
            self._response_futures[can_id] = {}
        
        # If a future already exists for this exact can_id and command_code,
        # it might mean a previous request timed out or was not cleaned up.
        # We'll overwrite it, but this could be logged.
        if command_code in self._response_futures[can_id] and \
           not self._response_futures[can_id][command_code].done():
            logger.warning(f"Overwriting an existing pending future for CAN ID {can_id:03X}, Cmd {command_code:02X}.")
            self._response_futures[can_id][command_code].cancel() # Cancel previous

        future = self._loop.create_future()
        self._response_futures[can_id][command_code] = future
        return future

    def start_listening(self):
        """Starts the background message listening task."""
        if not self._is_listening:
            self._is_listening = True
            if self.use_simulator:
                if not self._sim_reader:
                    logger.warning("Cannot start simulator listener: not connected.")
                    self._is_listening = False
                    return
                self._listener_task = self._loop.create_task(self._listen_for_messages_sim())
            else:
                if not self.bus:
                    logger.warning("Cannot start hardware listener: not connected.")
                    self._is_listening = False
                    return
                # For hardware, a more robust solution uses can.Notifier
                # This is a simplified direct polling approach for asyncio
                self._listener_task = self._loop.create_task(self._listen_for_messages_hw())
            logger.info("Message listener initiated.")
        else:
            logger.info("Message listener already running.")


    def stop_listening(self):
        """Stops the background message listening task."""
        if self._is_listening:
            self._is_listening = False
            if self._listener_task and not self._listener_task.done():
                self._listener_task.cancel()
                # Optionally wait for the task to finish cancellation
                # try:
                #     await self._listener_task
                # except asyncio.CancelledError:
                #     logger.info("Listener task cancelled successfully.")
                # except Exception as e:
                #     logger.error(f"Error while stopping listener task: {e}")
            self._listener_task = None
            logger.info("Message listener stopped.")

    async def send_and_wait_for_response(self,
                                         msg_to_send: can.Message,
                                         expected_response_can_id: int,
                                         expected_response_command_code: int,
                                         timeout: float = CAN_TIMEOUT_SECONDS) -> can.Message:
        """
        Sends a CAN message and waits for a specific response.

        Args:
            msg_to_send: The can.Message object to send.
            expected_response_can_id: The CAN ID of the expected response.
            expected_response_command_code: The command code (first data byte) of the expected response.
            timeout: How long to wait for the response in seconds.

        Returns:
            The received can.Message object for the response.

        Raises:
            CommunicationError: If a timeout occurs or another communication issue arises.
            CANError/SimulatorError: For issues during sending.
        """
        if not self._is_listening:
            self.start_listening() # Ensure listener is active

        future = self.create_response_future(expected_response_can_id, expected_response_command_code)
        
        try:
            await self.send_message(msg_to_send, timeout=timeout) # Use same timeout for send
        except (CANError, SimulatorError) as e:
            # If send fails, we should clean up the future
            if not future.done():
                future.cancel()
            raise # Re-raise the original sending error

        try:
            response_msg = await asyncio.wait_for(future, timeout=timeout)
            return response_msg
        except asyncio.TimeoutError:
            # Clean up the future if it timed out
            if expected_response_can_id in self._response_futures and \
               expected_response_command_code in self._response_futures[expected_response_can_id]:
                del self._response_futures[expected_response_can_id][expected_response_command_code]
                if not self._response_futures[expected_response_can_id]:
                    del self._response_futures[expected_response_can_id]

            error_msg = (f"Timeout waiting for response to command {msg_to_send.data[0]:02X} "
                         f"from CAN ID {expected_response_can_id:03X} (expected cmd code "
                         f"{expected_response_command_code:02X}).")
            logger.warning(error_msg)
            raise CommunicationError(error_msg) from None
        except asyncio.CancelledError:
            logger.warning(f"Response future for CAN ID {expected_response_can_id:03X}, Cmd {expected_response_command_code:02X} was cancelled.")
            raise CommunicationError("Response wait was cancelled.") from None

    @property
    def is_connected(self) -> bool:
        if self.use_simulator:
            return self._sim_writer is not None and not self._sim_writer.is_closing()
        else:
            return self.bus is not None # python-can bus objects don't have a simple is_connected property