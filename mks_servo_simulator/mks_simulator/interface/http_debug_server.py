"""
HTTP debug server for programmatic access to simulator state.

Provides a REST API that allows LLMs and other tools to programmatically
query simulator state, command history, and perform state validation.
"""

import asyncio
import json
from typing import Optional, Dict, Any
from pathlib import Path

try:
    from fastapi import FastAPI, HTTPException, Request
    from fastapi.responses import JSONResponse
    from fastapi.middleware.cors import CORSMiddleware
    import uvicorn
    FASTAPI_AVAILABLE = True
except ImportError:
    FASTAPI_AVAILABLE = False
    
    # Create stub classes for type hints
    class FastAPI:
        pass
    class HTTPException:
        pass

from .llm_debug_interface import LLMDebugInterface


class DebugHTTPServer:
    """
    HTTP server providing programmatic access to simulator debugging information.
    
    This server exposes REST endpoints that allow LLMs and other automated tools
    to query simulator state, validate expected conditions, and retrieve
    command execution history.
    """
    
    def __init__(self, debug_interface: LLMDebugInterface, port: int = 8765, host: str = "127.0.0.1"):
        """
        Initialize HTTP debug server.
        
        Args:
            debug_interface: LLMDebugInterface instance providing data
            port: Port to bind server to
            host: Host address to bind to
        """
        if not FASTAPI_AVAILABLE:
            raise ImportError("FastAPI and uvicorn are required for HTTP debug server. Install with: pip install fastapi uvicorn")
        
        self.debug_interface = debug_interface
        self.port = port
        self.host = host
        
        # Create FastAPI app
        self.app = FastAPI(
            title="MKS Servo Simulator Debug API",
            description="LLM-friendly API for debugging MKS servo interactions",
            version="1.0.0",
            docs_url="/docs",
            redoc_url="/redoc"
        )
        
        # Add CORS middleware for browser access
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )
        
        self._setup_routes()
    
    def _setup_routes(self):
        """Setup API routes"""
        
        @self.app.get("/", summary="API information")
        async def root():
            """Get API information and available endpoints"""
            return {
                "name": "MKS Servo Simulator Debug API",
                "version": "1.0.0",
                "description": "Provides programmatic access to simulator state for LLM debugging",
                "endpoints": {
                    "/status": "Get complete system status",
                    "/motors/{motor_id}": "Get specific motor status",
                    "/history": "Get command execution history",
                    "/validate": "Validate expected state (POST)",
                    "/commands": "Get available commands",
                    "/summary": "Get concise debug summary",
                    "/health": "Health check",
                    "/docs": "Interactive API documentation"
                }
            }
        
        @self.app.get("/status", summary="Get complete system status")
        async def get_status():
            """
            Get complete system status including all motors and communication stats.
            
            Returns comprehensive state information suitable for LLM analysis.
            """
            return self.debug_interface.get_system_status()
        
        @self.app.get("/motors/{motor_id}", summary="Get specific motor status")
        async def get_motor_status(motor_id: int):
            """
            Get detailed status for a specific motor.
            
            Args:
                motor_id: CAN ID of the motor (1-16 typically)
                
            Returns:
                Motor status dictionary
                
            Raises:
                HTTPException: If motor not found
            """
            status = self.debug_interface.get_motor_status(motor_id)
            if status is None:
                raise HTTPException(status_code=404, detail=f"Motor {motor_id} not found")
            return status
        
        @self.app.get("/history", summary="Get command history")
        async def get_command_history(motor_id: Optional[int] = None, limit: int = 50):
            """
            Get recent command execution history.
            
            Args:
                motor_id: Optional motor ID filter
                limit: Maximum number of commands to return (default 50)
                
            Returns:
                Command history with execution details
            """
            return {
                "history": self.debug_interface.get_command_history(motor_id, limit),
                "total_commands": len(self.debug_interface.command_history),
                "filter": {"motor_id": motor_id, "limit": limit}
            }
        
        @self.app.post("/validate", summary="Validate expected state")
        async def validate_state(expected_state: dict):
            """
            Validate current simulator state against expected state.
            
            This endpoint is particularly useful for LLMs to verify that
            commands have had their expected effect.
            
            Request body example:
            ```json
            {
                "motors": {
                    "1": {
                        "current_position": 1000,
                        "enabled": true
                    }
                },
                "tolerance": 10
            }
            ```
            
            Returns:
                Validation results with pass/fail status and details
            """
            return self.debug_interface.validate_expected_state(expected_state)
        
        @self.app.get("/commands", summary="Get available commands")
        async def get_available_commands():
            """
            Get list of all available CAN commands with descriptions.
            
            Useful for LLMs to understand what commands are available
            and their purposes.
            """
            return self.debug_interface.get_available_commands()
        
        @self.app.get("/summary", summary="Get concise debug summary")
        async def get_debug_summary():
            """
            Get a concise debug summary suitable for LLM context.
            
            Returns a brief text summary of current system state.
            """
            return {
                "summary": self.debug_interface.get_debug_summary(),
                "timestamp": self.debug_interface.get_system_status()["timestamp"]
            }
        
        @self.app.get("/health", summary="Health check")
        async def health_check():
            """
            Simple health check endpoint.
            
            Returns basic health and uptime information.
            """
            status = self.debug_interface.get_system_status()
            return {
                "status": "healthy",
                "uptime": status["uptime_seconds"],
                "motors_count": len(status["motors"]),
                "total_commands": status["communication"]["total_messages"]
            }
        
        @self.app.get("/export", summary="Export status to JSON")
        async def export_status():
            """
            Export complete status as downloadable JSON.
            
            Useful for saving system state for offline analysis.
            """
            status = self.debug_interface.get_system_status()
            return JSONResponse(
                content=status,
                headers={"Content-Disposition": "attachment; filename=simulator_status.json"}
            )
        
        # Error handlers
        @self.app.exception_handler(404)
        async def not_found_handler(request: Request, exc: HTTPException):
            return JSONResponse(
                status_code=404,
                content={"error": "Not found", "detail": str(exc.detail)}
            )
        
        @self.app.exception_handler(500)
        async def internal_error_handler(request: Request, exc: Exception):
            return JSONResponse(
                status_code=500,
                content={"error": "Internal server error", "detail": str(exc)}
            )
    
    async def start_server(self):
        """
        Start the HTTP debug server.
        
        Runs the server indefinitely until cancelled.
        """
        config = uvicorn.Config(
            self.app, 
            host=self.host, 
            port=self.port, 
            log_level="warning",  # Reduce noise
            access_log=False
        )
        server = uvicorn.Server(config)
        await server.serve()
    
    def get_server_info(self) -> Dict[str, Any]:
        """Get server configuration information"""
        return {
            "host": self.host,
            "port": self.port,
            "base_url": f"http://{self.host}:{self.port}",
            "docs_url": f"http://{self.host}:{self.port}/docs",
            "status_url": f"http://{self.host}:{self.port}/status",
            "available": FASTAPI_AVAILABLE
        }


class JSONOutputHandler:
    """
    Handler for JSON output mode for LLM consumption.
    
    Provides structured JSON output suitable for LLM parsing and analysis.
    """
    
    def __init__(self, debug_interface: LLMDebugInterface):
        """
        Initialize JSON output handler.
        
        Args:
            debug_interface: LLMDebugInterface instance
        """
        self.debug_interface = debug_interface
        self.started = False
    
    def emit_event(self, event_type: str, data: Dict[str, Any]):
        """
        Emit a JSON event to stdout.
        
        Args:
            event_type: Type of event (e.g., "status_update", "command_executed")
            data: Event data
        """
        event = {
            "event": event_type,
            "timestamp": self.debug_interface.get_system_status()["timestamp"],
            **data
        }
        print(json.dumps(event, default=str))
    
    def emit_startup(self, config: Dict[str, Any]):
        """Emit simulator startup event"""
        if not self.started:
            self.emit_event("simulator_started", {
                "config": config,
                "motors": list(self.debug_interface.motors.keys())
            })
            self.started = True
    
    def emit_status_update(self):
        """Emit periodic status update"""
        status = self.debug_interface.get_system_status()
        self.emit_event("status_update", status)
    
    def emit_command_executed(self, motor_id: int, command_code: int, success: bool):
        """Emit command execution event"""
        self.emit_event("command_executed", {
            "motor_id": motor_id,
            "command_code": f"0x{command_code:02X}",
            "success": success
        })
    
    def emit_error(self, motor_id: int, error_type: str, description: str):
        """Emit error event"""
        self.emit_event("error", {
            "motor_id": motor_id,
            "error_type": error_type,
            "description": description
        })
    
    async def run_periodic_updates(self, interval: float = 1.0):
        """
        Run periodic status updates.
        
        Args:
            interval: Update interval in seconds
        """
        while True:
            self.emit_status_update()
            await asyncio.sleep(interval)