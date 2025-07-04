"""
Virtual CAN Bus for the MKS Servo Simulator.
Handles communication between the mks-servo-can library (in sim mode)
and multiple SimulatedMotor instances.
"""
from typing import Dict, List, Tuple, Optional, TYPE_CHECKING

import asyncio
import logging
import time

from .motor_model import SimulatedMotor

if TYPE_CHECKING:
    from .interface.llm_debug_interface import LLMDebugInterface
    from .interface.performance_monitor import PerformanceMonitor

# CRC and constants might be needed if we re-validate here, but motor_model handles it.
try:
    from mks_servo_can import constants as const  # For potential use
    from mks_servo_can.crc import \
        calculate_crc  # For potential use
except ImportError:
    pass  # Handled in motor_model for its own needs

logger = logging.getLogger(__name__)


class VirtualCANBus:
    """
    Simulates a CAN bus, managing communication between one or more connected
    clients (representing the mks-servo-can library) and multiple
    SimulatedMotor instances.

    It listens for client connections, routes incoming simulated CAN messages
    to the appropriate motor(s), and forwards motor responses back to the client.
    It can also simulate bus latency.
    """
    def __init__(self, loop: asyncio.AbstractEventLoop):
        """
        Initializes the VirtualCANBus.

        Args:
            loop: The asyncio event loop to use for server and motor tasks.
        """
        self._loop = loop
        self.simulated_motors: Dict[int, SimulatedMotor] = (
            {}
        )  # CAN ID -> Motor Object
        self.clients: List[
            Tuple[asyncio.StreamWriter, asyncio.StreamReader]
        ] = []  # Store writer, reader tuples
        self.global_latency_ms: float = (
            0  # Milliseconds for command transmission and response
        )
        self.debug_interface: Optional['LLMDebugInterface'] = None
        self.performance_monitor: Optional['PerformanceMonitor'] = None
        
        # Connection tracking for performance monitoring
        self._client_counter = 0
        self._client_ids: Dict[Tuple[asyncio.StreamWriter, asyncio.StreamReader], str] = {}
        
        # Alias for compatibility with LLMDebugInterface
        self.motors = self.simulated_motors

    def add_motor(self, motor: SimulatedMotor):
        """
        Adds a SimulatedMotor instance to the virtual CAN bus.

        If a motor with the same CAN ID already exists, it will be overwritten.

        Args:
            motor: The SimulatedMotor instance to add.
        """
        if motor.can_id in self.simulated_motors:
            logger.warning(
                f"Motor with CAN ID {motor.can_id} already exists on virtual bus. Overwriting."
            )
        self.simulated_motors[motor.can_id] = motor
        logger.info(
            f"Added simulated motor with CAN ID {motor.can_id:03X} to virtual bus."
        )
        # self._loop.create_task(motor.start()) # Start motor's internal simulation loop

    async def start_all_motors(self):
        """
        Starts the internal simulation loop for all registered SimulatedMotor instances.
        This typically involves scheduling their `_update_state` methods as asyncio tasks.
        """
        logger.info("Starting all simulated motors...")
        for motor in self.simulated_motors.values():
            self._loop.create_task(motor.start())
        logger.info("All simulated motors have been issued a start command.")

    async def stop_all_motors(self):
        """
        Stops the internal simulation loop for all registered SimulatedMotor instances.
        This typically involves cancelling their `_update_state` tasks.
        """
        logger.info("Stopping all simulated motors...")
        for motor in self.simulated_motors.values():
            await motor.stop_simulation()  # Ensure it's awaited if stop_simulation is async
        logger.info("All simulated motors have been stopped.")

    def remove_motor(self, can_id: int):
        """
        Removes a simulated motor from the virtual CAN bus.

        Args:
            can_id: The CAN ID of the motor to remove.
        """
        if can_id in self.simulated_motors:
            self.simulated_motors.pop(can_id)
            # self._loop.create_task(motor.stop_simulation()) # Ensure it stops
            logger.info(f"Removed simulated motor with CAN ID {can_id:03X}.")

    def set_latency(self, latency_ms: float):
        """
        Sets the simulated round-trip communication latency for the virtual CAN bus.

        This latency is applied to messages passing through the bus, split
        between receiving a command and sending a response.

        Args:
            latency_ms: The total round-trip latency in milliseconds. Must be non-negative.
        """
        self.global_latency_ms = max(0, latency_ms)
        logger.info(
            f"Virtual CAN bus latency set to {self.global_latency_ms:.2f} ms."
        )

    async def _send_response_to_client(
        self, writer: asyncio.StreamWriter, can_id: int, response_payload: bytes
    ):
        """Sends a formatted CAN response back to the connected library client."""
        # Protocol: "SIM_CAN_RECV <id_hex> <dlc_int> <data_hex_no_space>\n"
        # response_payload includes echoed command code, data, and CRC.
        dlc = (
            len(response_payload) - 1
        )  # CRC is not part of DLC calculation for CAN frame's DLC field
        # But response_payload here is (cmd_echo, data..., crc)
        # Actual frame DLC will be len(cmd_echo, data...)

        # The response_payload IS the data field of the CAN message.
        # So its length is the DLC for the CAN message.
        actual_can_dlc = len(response_payload)

        data_hex_no_space = response_payload.hex()

        response_str = (
            f"SIM_CAN_RECV {can_id:03X} {actual_can_dlc} {data_hex_no_space}\n"
        )
        logger.debug(f"VirtualBus sending to lib: {response_str.strip()}")
        try:
            writer.write(response_str.encode())
            await writer.drain()
        except ConnectionResetError:
            logger.warning(
                "Client connection reset while sending response. Removing client."
            )
            self.clients = [
                (w, r) for w, r in self.clients if w != writer
            ]  # Remove this client
        except Exception as e:
            logger.error(f"Error sending response to client: {e}")

    async def handle_client_message(
        self,
        message: str,
        writer: asyncio.StreamWriter,
        reader: asyncio.StreamReader,
    ):
        """
        Handles a message received from the mks-servo-can library (client).
        Expected message format: "SIM_CAN_SEND <id_hex> <dlc_int> <data_hex_no_space>\n"
        e.g., "SIM_CAN_SEND 001 2 3031\n" (ID 1, DLC 2, Data 0x30, 0x31)
        """
        logger.debug(f"VirtualBus received from lib: {message.strip()}")
        parts = message.strip().split()

        if not parts or parts[0] != "SIM_CAN_SEND" or len(parts) < 4:
            logger.warning(f"Malformed message from client: {message.strip()}")
            # Optionally send an error back to client?
            return

        try:
            target_can_id = int(parts[1], 16)
            # dlc_from_msg = int(parts[2]) # DLC from client message (not directly used by motor model)
            hex_data_payload = parts[
                3
            ]  # This is (command_code + data_bytes + crc_byte)

            # Convert hex_data_payload to bytes
            full_payload_bytes = bytes.fromhex(hex_data_payload)

            if not full_payload_bytes:  # Should have at least command + CRC
                logger.warning(
                    f"Empty data payload from client for ID {target_can_id:03X}"
                )
                return

            command_code = full_payload_bytes[0]
            command_data_bytes = full_payload_bytes[
                1:-1
            ]  # Data between command code and CRC
            # Received CRC is full_payload_bytes[-1], motor model can re-verify if needed

        except (ValueError, IndexError) as e:
            logger.error(
                f"Error parsing client message '{message.strip()}': {e}"
            )
            return

        # Simulate latency for processing the command
        if self.global_latency_ms > 0:
            await asyncio.sleep(
                self.global_latency_ms / 2000.0
            )  # Half for receiving

        # Route to specific motor or broadcast
        motors_to_process: List[SimulatedMotor] = []
        if target_can_id == 0x00:  # Broadcast
            motors_to_process.extend(self.simulated_motors.values())
        elif target_can_id in self.simulated_motors:
            motors_to_process.append(self.simulated_motors[target_can_id])
        else:
            logger.warning(
                f"Received command for unknown CAN ID {target_can_id:03X}. Ignoring."
            )
            return

        for motor in motors_to_process:
            # The motor's process_command should return (response_can_id, response_payload_with_crc)
            # response_payload_with_crc includes the echoed command code, data, and CRC.
            
            # Record command start time for debug interface
            command_start_time = time.time()

            # Define a callback for the motor to send completion messages asynchronously
            async def send_async_completion_to_client(
                resp_can_id: int, resp_payload_with_crc: bytes
            ):
                """
                Callback for SimulatedMotor to send asynchronous completion messages.

                This function is passed to the motor model, allowing it to send
                messages (like move completion notifications) back to the client
                after an initial command has been processed. Incorporates bus latency.

                Args:
                    resp_can_id: The CAN ID to use for the response (typically motor's ID).
                    resp_payload_with_crc: The complete response payload, including
                                           echoed command, data, and CRC.
                """
                if self.global_latency_ms > 0:
                    await asyncio.sleep(
                        self.global_latency_ms / 2000.0
                    )  # Half for sending back
                await self._send_response_to_client(
                    writer, resp_can_id, resp_payload_with_crc
                )

            response_tuple = motor.process_command(
                command_code,
                command_data_bytes,
                send_async_completion_to_client,
            )
            
            # Record command execution for debug interface and performance monitoring
            command_end_time = time.time()
            response_time_ms = (command_end_time - command_start_time) * 1000
            success = response_tuple is not None
            
            # Performance monitoring
            if self.performance_monitor:
                self.performance_monitor.record_command_latency(response_time_ms)
                
                # Get client ID for connection tracking
                client_key = (writer, reader)
                client_id = self._client_ids.get(client_key, "unknown")
                
                # Record command event
                bytes_sent = len(response_tuple[1]) if response_tuple else 0
                bytes_received = len(full_payload_bytes)
                
                self.performance_monitor.record_connection_event(
                    "command", client_id,
                    bytes_sent=bytes_sent,
                    bytes_received=bytes_received
                )
            
            # Debug interface recording
            if self.debug_interface:
                # Get command name from manual or use hex code
                command_name = f"0x{command_code:02X}"
                if hasattr(self.debug_interface, 'MANUAL_COMMANDS'):
                    manual_cmds = getattr(self.debug_interface, 'MANUAL_COMMANDS', {})
                    if f"0x{command_code:02X}" in manual_cmds:
                        command_name = manual_cmds[f"0x{command_code:02X}"].get('name', command_name)
                
                # Create parameters dict from command data
                parameters = {
                    'command_data_hex': command_data_bytes.hex() if command_data_bytes else '',
                    'data_length': len(command_data_bytes),
                    'target_can_id': target_can_id
                }
                
                self.debug_interface.record_command(
                    motor_id=motor.can_id,
                    command_code=command_code,
                    command_name=command_name,
                    parameters=parameters,
                    response_time=response_time_ms / 1000,  # Convert back to seconds for debug interface
                    success=success,
                    error_message=None if success else "No response generated"
                )

            if response_tuple:
                response_can_id, response_payload_with_crc = response_tuple
                # Simulate latency for sending the response
                if self.global_latency_ms > 0:
                    await asyncio.sleep(
                        self.global_latency_ms / 2000.0
                    )  # Half for sending back

                await self._send_response_to_client(
                    writer, response_can_id, response_payload_with_crc
                )
            # If response_tuple is None, the motor handles its response internally (e.g. no direct ack)
            # or will use the send_async_completion_to_client for delayed responses.

    async def client_handler_loop(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ):
        """Handles a single connected client, reading messages in a loop."""
        addr = writer.get_extra_info("peername")
        logger.info(f"Client {addr} connected to virtual CAN bus.")
        self.clients.append((writer, reader))
        
        # Performance monitoring - track connection
        client_key = (writer, reader)
        self._client_counter += 1
        client_id = f"client_{self._client_counter}_{addr[0]}:{addr[1]}"
        self._client_ids[client_key] = client_id
        
        if self.performance_monitor:
            self.performance_monitor.record_connection_event("connect", client_id)
        try:
            while True:
                line_bytes = await reader.readline()
                if not line_bytes:  # Client disconnected
                    logger.info(f"Client {addr} disconnected.")
                    break
                message = line_bytes.decode().strip()
                if message:
                    await self.handle_client_message(message, writer, reader)
        except ConnectionResetError:
            logger.info(f"Client {addr} connection reset.")
        except (
            asyncio.IncompleteReadError
        ):  # Happens on unclean client disconnect
            logger.info(f"Client {addr} disconnected (incomplete read).")
        except Exception as e:
            logger.error(
                f"Error in client handler for {addr}: {e}", exc_info=True
            )
        finally:
            logger.info(f"Closing connection for client {addr}.")
            
            # Performance monitoring - track disconnection
            if self.performance_monitor and client_key in self._client_ids:
                self.performance_monitor.record_connection_event("disconnect", client_id)
                del self._client_ids[client_key]
            
            if (writer, reader) in self.clients:
                self.clients.remove((writer, reader))
            if not writer.is_closing():
                writer.close()
                await writer.wait_closed()

    async def start_server(self, host: str = "localhost", port: int = 6789):
        """Starts the TCP server to listen for library connections."""
        server = await asyncio.start_server(
            self.client_handler_loop, host, port
        )
        addr = server.sockets[0].getsockname()
        logger.info(f"Virtual CAN Bus server listening on {addr}")

        # Start all registered motors
        await self.start_all_motors()

        async with server:
            await server.serve_forever()

        # Cleanup when server stops (e.g. Ctrl+C)
        await self.stop_all_motors()
