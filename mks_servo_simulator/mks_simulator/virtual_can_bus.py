# mks_servo_can_project/mks_servo_simulator/mks_simulator/virtual_can_bus.py
"""
Virtual CAN Bus for the MKS Servo Simulator.
Handles communication between the mks-servo-can library (in sim mode)
and multiple SimulatedMotor instances.
"""
import asyncio
import logging
from typing import Dict, Optional, List, Callable, Tuple

from .motor_model import SimulatedMotor
# CRC and constants might be needed if we re-validate here, but motor_model handles it.
try:
    from mks_servo_can_library.mks_servo_can.crc import calculate_crc # For potential use
    from mks_servo_can_library.mks_servo_can import constants as const # For potential use
except ImportError:
    pass # Handled in motor_model for its own needs

logger = logging.getLogger(__name__)

class VirtualCANBus:
    def __init__(self, loop: asyncio.AbstractEventLoop):
        self._loop = loop
        self.simulated_motors: Dict[int, SimulatedMotor] = {} # CAN ID -> Motor Object
        self.clients: List[Tuple[asyncio.StreamWriter, asyncio.StreamReader]] = [] # Store writer, reader tuples
        self.global_latency_ms: float = 0 # Milliseconds for command transmission and response

    def add_motor(self, motor: SimulatedMotor):
        if motor.can_id in self.simulated_motors:
            logger.warning(f"Motor with CAN ID {motor.can_id} already exists on virtual bus. Overwriting.")
        self.simulated_motors[motor.can_id] = motor
        logger.info(f"Added simulated motor with CAN ID {motor.can_id:03X} to virtual bus.")
        # self._loop.create_task(motor.start()) # Start motor's internal simulation loop

    async def start_all_motors(self):
        logger.info("Starting all simulated motors...")
        for motor in self.simulated_motors.values():
            self._loop.create_task(motor.start())
        logger.info("All simulated motors have been issued a start command.")


    async def stop_all_motors(self):
        logger.info("Stopping all simulated motors...")
        for motor in self.simulated_motors.values():
            await motor.stop_simulation() # Ensure it's awaited if stop_simulation is async
        logger.info("All simulated motors have been stopped.")


    def remove_motor(self, can_id: int):
        if can_id in self.simulated_motors:
            motor = self.simulated_motors.pop(can_id)
            # self._loop.create_task(motor.stop_simulation()) # Ensure it stops
            logger.info(f"Removed simulated motor with CAN ID {can_id:03X}.")

    def set_latency(self, latency_ms: float):
        self.global_latency_ms = max(0, latency_ms)
        logger.info(f"Virtual CAN bus latency set to {self.global_latency_ms:.2f} ms.")

    async def _send_response_to_client(self, writer: asyncio.StreamWriter, can_id: int, response_payload: bytes):
        """Sends a formatted CAN response back to the connected library client."""
        # Protocol: "SIM_CAN_RECV <id_hex> <dlc_int> <data_hex_no_space>\n"
        # response_payload includes echoed command code, data, and CRC.
        dlc = len(response_payload) - 1 # CRC is not part of DLC calculation for CAN frame's DLC field
                                         # But response_payload here is (cmd_echo, data..., crc)
                                         # Actual frame DLC will be len(cmd_echo, data...)
        
        # The response_payload IS the data field of the CAN message.
        # So its length is the DLC for the CAN message.
        actual_can_dlc = len(response_payload)
        
        data_hex_no_space = response_payload.hex()
        
        response_str = f"SIM_CAN_RECV {can_id:03X} {actual_can_dlc} {data_hex_no_space}\n"
        logger.debug(f"VirtualBus sending to lib: {response_str.strip()}")
        try:
            writer.write(response_str.encode())
            await writer.drain()
        except ConnectionResetError:
            logger.warning("Client connection reset while sending response. Removing client.")
            self.clients = [(w, r) for w, r in self.clients if w != writer] # Remove this client
        except Exception as e:
            logger.error(f"Error sending response to client: {e}")


    async def handle_client_message(self, message: str, writer: asyncio.StreamWriter, reader: asyncio.StreamReader):
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
            hex_data_payload = parts[3] # This is (command_code + data_bytes + crc_byte)
            
            # Convert hex_data_payload to bytes
            full_payload_bytes = bytes.fromhex(hex_data_payload)
            
            if not full_payload_bytes: # Should have at least command + CRC
                logger.warning(f"Empty data payload from client for ID {target_can_id:03X}")
                return

            command_code = full_payload_bytes[0]
            command_data_bytes = full_payload_bytes[1:-1] # Data between command code and CRC
            # Received CRC is full_payload_bytes[-1], motor model can re-verify if needed

        except (ValueError, IndexError) as e:
            logger.error(f"Error parsing client message '{message.strip()}': {e}")
            return

        # Simulate latency for processing the command
        if self.global_latency_ms > 0:
            await asyncio.sleep(self.global_latency_ms / 2000.0) # Half for receiving

        # Route to specific motor or broadcast
        motors_to_process: List[SimulatedMotor] = []
        if target_can_id == 0x00: # Broadcast
            motors_to_process.extend(self.simulated_motors.values())
        elif target_can_id in self.simulated_motors:
            motors_to_process.append(self.simulated_motors[target_can_id])
        else:
            logger.warning(f"Received command for unknown CAN ID {target_can_id:03X}. Ignoring.")
            return

        for motor in motors_to_process:
            # The motor's process_command should return (response_can_id, response_payload_with_crc)
            # response_payload_with_crc includes the echoed command code, data, and CRC.
            
            # Define a callback for the motor to send completion messages asynchronously
            async def send_async_completion_to_client(resp_can_id: int, resp_payload_with_crc: bytes):
                if self.global_latency_ms > 0:
                    await asyncio.sleep(self.global_latency_ms / 2000.0) # Half for sending back
                await self._send_response_to_client(writer, resp_can_id, resp_payload_with_crc)

            response_tuple = motor.process_command(command_code, command_data_bytes, send_async_completion_to_client)

            if response_tuple:
                response_can_id, response_payload_with_crc = response_tuple
                # Simulate latency for sending the response
                if self.global_latency_ms > 0:
                    await asyncio.sleep(self.global_latency_ms / 2000.0) # Half for sending back
                
                await self._send_response_to_client(writer, response_can_id, response_payload_with_crc)
            # If response_tuple is None, the motor handles its response internally (e.g. no direct ack)
            # or will use the send_async_completion_to_client for delayed responses.


    async def client_handler_loop(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        """Handles a single connected client, reading messages in a loop."""
        addr = writer.get_extra_info('peername')
        logger.info(f"Client {addr} connected to virtual CAN bus.")
        self.clients.append((writer, reader))
        try:
            while True:
                line_bytes = await reader.readline()
                if not line_bytes: # Client disconnected
                    logger.info(f"Client {addr} disconnected.")
                    break
                message = line_bytes.decode().strip()
                if message:
                    await self.handle_client_message(message, writer, reader)
        except ConnectionResetError:
            logger.info(f"Client {addr} connection reset.")
        except asyncio.IncompleteReadError: # Happens on unclean client disconnect
            logger.info(f"Client {addr} disconnected (incomplete read).")
        except Exception as e:
            logger.error(f"Error in client handler for {addr}: {e}", exc_info=True)
        finally:
            logger.info(f"Closing connection for client {addr}.")
            if (writer,reader) in self.clients:
                self.clients.remove((writer,reader))
            if not writer.is_closing():
                writer.close()
                await writer.wait_closed()

    async def start_server(self, host: str = 'localhost', port: int = 6789):
        """Starts the TCP server to listen for library connections."""
        server = await asyncio.start_server(self.client_handler_loop, host, port)
        addr = server.sockets[0].getsockname()
        logger.info(f'Virtual CAN Bus server listening on {addr}')

        # Start all registered motors
        await self.start_all_motors()

        async with server:
            await server.serve_forever()
            
        # Cleanup when server stops (e.g. Ctrl+C)
        await self.stop_all_motors()