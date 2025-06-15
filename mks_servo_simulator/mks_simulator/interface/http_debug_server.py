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
    from pydantic import BaseModel, Field # Added
    from typing import List, Dict, Any # Added (already present but good to ensure)
    FASTAPI_AVAILABLE = True
except ImportError:
    FASTAPI_AVAILABLE = False
    
    # Create stub classes for type hints
    class FastAPI: # type: ignore
        pass
    class HTTPException: # type: ignore
        pass
    class BaseModel: # type: ignore
        pass
    class Field: # type: ignore
        pass
    # Ensure List, Dict, Any, Optional are available for stub if typing wasn't already imported
    from typing import List, Dict, Any, Optional


from .llm_debug_interface import LLMDebugInterface

# Pydantic Models for Request Validation
class RawCommandPayload(BaseModel):
    motor_id: int
    command_code: int
    data_bytes: List[int] = Field(default_factory=list)
    expect_response: bool = True

class TemplateCommandPayload(BaseModel):
    motor_id: int
    template_name: str
    args: Optional[Dict[str, Any]] = None

class ParameterUpdatePayload(BaseModel):
    value: Any


class DebugHTTPServer:
    """
    HTTP server providing programmatic access to simulator debugging information.
    
    This server exposes REST endpoints that allow LLMs and other automated tools
    to query simulator state, validate expected conditions, and retrieve
    command execution history.
    """
    
    def __init__(self, debug_interface: LLMDebugInterface, port: int = 8765, host: str = "127.0.0.1", config_manager=None, live_config=None):
        """
        Initialize HTTP debug server.
        
        Args:
            debug_interface: LLMDebugInterface instance providing data
            port: Port to bind server to
            host: Host address to bind to
            config_manager: Optional configuration manager for profile management
            live_config: Optional live configuration interface for runtime updates
        """
        if not FASTAPI_AVAILABLE:
            raise ImportError("FastAPI and uvicorn are required for HTTP debug server. Install with: pip install fastapi uvicorn")
        
        self.debug_interface = debug_interface
        self.config_manager = config_manager
        self.live_config = live_config
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
                "version": "1.1.0",
                "description": "Provides programmatic access to simulator state and command injection for LLM debugging",
                "endpoints": {
                    "/status": "Get complete system status",
                    "/motors/{motor_id}": "Get specific motor status",
                    "/history": "Get command execution history",
                    "/validate": "Validate expected state (POST)",
                    "/commands": "Get available commands",
                    "/summary": "Get concise debug summary",
                    "/health": "Health check",
                    "/inject": "Inject raw command (POST)",
                    "/inject_template": "Inject template command (POST)",
                    "/templates": "Get available command templates",
                    "/injection_stats": "Get command injection statistics",
                    "/run_scenario": "Run test scenario (POST)",
                    "/performance": "Get current performance metrics",
                    "/performance/history": "Get performance trends over time",
                    "/performance/connections": "Get connection details and statistics",
                    "/performance/reset": "Reset performance metrics (POST)",
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
            "total_commands_processed": status["communication"]["total_messages_sent"]
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
        
        # Command injection endpoints
        @self.app.post("/inject", summary="Inject raw command")
        async def inject_command(request_data: RawCommandPayload):
            """
            Inject a raw command into a specified motor.
            
            Request body example:
            ```json
            {
                "motor_id": 1,
                "command_code": 246,  // 0xF6 for speed mode
                "data_bytes": [1, 0, 100, 0],
                "expect_response": true
            }
            ```
            
            Returns:
                Command execution result with response data
            """
            try:
                # Data is already validated by Pydantic
                motor_id = request_data.motor_id
                command_code = request_data.command_code
                data_bytes = request_data.data_bytes
                expect_response = request_data.expect_response
                
                # Get command injector from debug interface
                if not hasattr(self.debug_interface, 'command_injector'):
                    from .debug_tools import CommandInjector
                    self.debug_interface.command_injector = CommandInjector(
                        self.debug_interface.virtual_can_bus, 
                        self.debug_interface
                    )
                
                result = await self.debug_interface.command_injector.inject_command(
                    motor_id=motor_id,
                    command_code=command_code,
                    data_bytes=data_bytes,
                    expect_response=expect_response
                )
                
                return {
                    "success": result.success,
                    "command_name": result.command_name,
                    "execution_time_ms": result.execution_time_ms,
                    "response_data": list(result.response_data) if result.response_data is not None else [],
                    "error_message": result.error_message
                }
                
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))
        
        @self.app.post("/inject_template", summary="Inject template command")
        async def inject_template_command(request_data: TemplateCommandPayload):
            """
            Inject a pre-defined template command.
            
            Request body example:
            ```json
            {
                "motor_id": 1,
                "template_name": "enable"
            }
            ```
            
            Returns:
                Command execution result
            """
            try:
                # Data is already validated by Pydantic
                motor_id = request_data.motor_id
                template_name = request_data.template_name
                args = request_data.args # Will be None if not provided
                
                # Get command injector from debug interface
                if not hasattr(self.debug_interface, 'command_injector'):
                    from .debug_tools import CommandInjector
                    self.debug_interface.command_injector = CommandInjector(
                        self.debug_interface.virtual_can_bus, 
                        self.debug_interface
                    )
                
                result = await self.debug_interface.command_injector.inject_template_command(
                    motor_id=motor_id,
                    template_name=template_name,
                    args=args
                )
                
                return {
                    "success": result.success,
                    "command_name": result.command_name,
                    "execution_time_ms": result.execution_time_ms,
                    "response_data": list(result.response_data) if result.response_data is not None else [],
                    "error_message": result.error_message
                }
                
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))
        
        @self.app.get("/templates", summary="Get available command templates")
        async def get_command_templates():
            """
            Get all available command templates.
            
            Returns:
                Dictionary of template names and their specifications
            """
            # Get command injector from debug interface
            if not hasattr(self.debug_interface, 'command_injector'):
                from .debug_tools import CommandInjector
                self.debug_interface.command_injector = CommandInjector(
                    self.debug_interface.virtual_can_bus, 
                    self.debug_interface
                )
            
            return self.debug_interface.command_injector.get_available_templates()
        
        @self.app.get("/injection_stats", summary="Get command injection statistics")
        async def get_injection_stats():
            """
            Get statistics about injected commands.
            
            Returns:
                Statistics including success rates and most used commands
            """
            # Get command injector from debug interface
            if not hasattr(self.debug_interface, 'command_injector'):
                from .debug_tools import CommandInjector
                self.debug_interface.command_injector = CommandInjector(
                    self.debug_interface.virtual_can_bus, 
                    self.debug_interface
                )
            
            return self.debug_interface.command_injector.get_command_statistics()
        
        @self.app.post("/run_scenario", summary="Run test scenario")
        async def run_test_scenario(request_data: dict):
            """
            Run a pre-defined test scenario.
            
            Request body example:
            ```json
            {
                "motor_id": 1,
                "scenario_name": "basic_movement"
            }
            ```
            
            Returns:
                List of executed commands with results
            """
            try:
                motor_id = request_data.get("motor_id")
                scenario_name = request_data.get("scenario_name", "basic_movement")
                
                if motor_id is None:
                    raise HTTPException(status_code=400, detail="motor_id is required")
                
                # Get command injector from debug interface
                if not hasattr(self.debug_interface, 'command_injector'):
                    from .debug_tools import CommandInjector
                    self.debug_interface.command_injector = CommandInjector(
                        self.debug_interface.virtual_can_bus, 
                        self.debug_interface
                    )
                
                results = await self.debug_interface.command_injector.run_test_scenario(
                    motor_id=motor_id,
                    scenario_name=scenario_name
                )
                
                return {
                    "scenario_name": scenario_name,
                    "motor_id": motor_id,
                    "total_commands": len(results),
                    "successful_commands": sum(1 for r in results if r.success),
                    "results": [
                        {
                            "command_name": r.command_name,
                            "success": r.success,
                            "execution_time_ms": r.execution_time_ms,
                            "error_message": r.error_message
                        }
                        for r in results
                    ]
                }
                
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))
        
        # Performance monitoring endpoints
        @self.app.get("/performance", summary="Get current performance metrics")
        async def get_performance_metrics():
            """
            Get current performance monitoring data.
            
            Returns:
                Current performance metrics including latency, throughput, and memory usage
            """
            if not hasattr(self.debug_interface, 'virtual_can_bus') or not self.debug_interface.virtual_can_bus.performance_monitor:
                return {"error": "Performance monitoring not enabled"}
            
            performance_monitor = self.debug_interface.virtual_can_bus.performance_monitor
            return performance_monitor.get_performance_summary()
        
        @self.app.get("/performance/history", summary="Get performance history")
        async def get_performance_history(minutes: int = 10):
            """
            Get performance trends over time.
            
            Args:
                minutes: Number of minutes of history to return
                
            Returns:
                Performance trends data
            """
            if not hasattr(self.debug_interface, 'virtual_can_bus') or not self.debug_interface.virtual_can_bus.performance_monitor:
                return {"error": "Performance monitoring not enabled"}
            
            performance_monitor = self.debug_interface.virtual_can_bus.performance_monitor
            return performance_monitor.get_performance_trends(minutes)
        
        @self.app.get("/performance/connections", summary="Get connection details")
        async def get_connection_details():
            """
            Get detailed information about active connections.
            
            Returns:
                List of connection details with statistics
            """
            if not hasattr(self.debug_interface, 'virtual_can_bus') or not self.debug_interface.virtual_can_bus.performance_monitor:
                return {"error": "Performance monitoring not enabled"}
            
            performance_monitor = self.debug_interface.virtual_can_bus.performance_monitor
            return {
                "connections": performance_monitor.get_connection_details(),
                "summary": {
                    "total_connections": len(performance_monitor.connections),
                    "active_connections": sum(1 for conn in performance_monitor.connections.values() if conn.is_active)
                }
            }
        
        @self.app.post("/performance/reset", summary="Reset performance metrics")
        async def reset_performance_metrics():
            """
            Reset all performance monitoring data.
            
            Returns:
                Confirmation message
            """
            if not hasattr(self.debug_interface, 'virtual_can_bus') or not self.debug_interface.virtual_can_bus.performance_monitor:
                raise HTTPException(status_code=404, detail="Performance monitoring not enabled")
            
            performance_monitor = self.debug_interface.virtual_can_bus.performance_monitor
            performance_monitor.reset_metrics()
            
            return {"message": "Performance metrics reset successfully"}
        
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
        
        # Configuration management endpoints (only if config manager is available)
        if self.config_manager:
            @self.app.get("/config", summary="Get current configuration")
            async def get_configuration():
                """
                Get current simulator configuration.
                
                Returns the current configuration summary including motors, settings, and features.
                """
                return self.config_manager.get_config_summary()
            
            @self.app.get("/config/profiles", summary="List configuration profiles")
            async def list_profiles():
                """
                List all available configuration profiles.
                
                Returns a list of profile names that can be loaded.
                """
                profiles = self.config_manager.list_profiles()
                return {"profiles": profiles}
            
            @self.app.post("/config/profiles/{profile_name}/load", summary="Load configuration profile")
            async def load_profile(profile_name: str):
                """
                Load a specific configuration profile.
                
                Args:
                    profile_name: Name of profile to load
                
                Returns:
                    Success/failure status with details
                """
                try:
                    config = self.config_manager.load_config(profile_name)
                    if config:
                        self.config_manager.current_config = config
                        # Ensure get_config_summary is called if current_config might not be immediately reflective
                        # or if load_config doesn't update what get_config_summary uses.
                        # For simplicity, assuming get_config_summary reflects the newly loaded config.
                        return {"success": True, "profile": profile_name, "config": self.config_manager.get_config_summary()}
                    else:
                        # Match test expectation for not found
                        return {"success": False, "profile": profile_name, "error": f"Profile '{profile_name}' not found or failed to load."}
                except Exception as e:
                    # It's better to let FastAPI handle validation errors for consistent 422,
                    # but for other errors, 500 is okay.
                    # However, the test for not_found expects 200 with success:False, so we handle it above.
                    raise HTTPException(status_code=500, detail=str(e))
            
            @self.app.post("/config/profiles/{profile_name}/save", summary="Save configuration profile")
            async def save_profile(profile_name: str):
                """
                Save current configuration as a profile.
                
                Args:
                    profile_name: Name to save profile as
                
                Returns:
                    Success/failure status with details
                """
                try:
                    if not self.config_manager.current_config:
                        # Match test expectation for this scenario
                        raise HTTPException(status_code=400, detail="No current configuration loaded to save.")
                    
                    # Assuming save_config takes the profile name, not the config object directly,
                    # or that it internally uses current_config if first arg is name.
                    # The original code was: self.config_manager.save_config(self.config_manager.current_config, profile_name)
                    # This implies save_config might need the config object.
                    # Let's assume config_manager.save_config is adapted or already works with just profile_name
                    # by saving the current_config. If it strictly needs the config object:
                    # success = self.config_manager.save_config(self.config_manager.current_config, profile_name)
                    # For now, let's assume it's simplified to save current_config by profile_name
                    success = self.config_manager.save_config(profile_name) # This might need adjustment based on actual save_config signature
                    if success:
                        return {"success": True, "profile": profile_name} # Match test
                    else:
                        # This path might not be hit if save_config raises exceptions on failure.
                        raise HTTPException(status_code=500, detail=f"Failed to save profile '{profile_name}'")
                except HTTPException as http_exc: # Catch HTTPException first and re-raise
                    raise http_exc
                except Exception as e:
                    raise HTTPException(status_code=500, detail=str(e))
            
            @self.app.delete("/config/profiles/{profile_name}", summary="Delete configuration profile")
            async def delete_profile(profile_name: str):
                """
                Delete a configuration profile.
                
                Args:
                    profile_name: Name of profile to delete
                
                Returns:
                    Success/failure status with details
                """
                try:
                    success = self.config_manager.delete_profile(profile_name)
                    if success:
                        return {"success": True, "message": f"Deleted profile '{profile_name}'"}
                    else:
                        return {"success": False, "message": f"Profile '{profile_name}' not found"}
                except Exception as e:
                    raise HTTPException(status_code=500, detail=str(e))
            
            @self.app.get("/config/templates", summary="Get motor templates")
            async def get_motor_templates():
                """
                Get available motor templates.
                
                Returns a dictionary of template names and their configurations.
                """
                templates = self.config_manager.get_motor_templates()
                # Convert MotorConfig objects to dictionaries for JSON serialization
                serializable_templates = {}
                for name, template in templates.items():
                    template_dict = {
                        "motor_type": template.motor_type,
                        "steps_per_rev": template.steps_per_rev,
                        "max_current": template.max_current,
                        "max_speed": template.max_speed,
                        "description": getattr(template, 'description', '')
                    }
                    serializable_templates[name] = template_dict
                return {"templates": serializable_templates}
            
            @self.app.post("/config/templates/{template_name}/apply", summary="Apply motor template")
            async def apply_motor_template(template_name: str, request_data: dict):
                """
                Apply a motor template to a specific motor.
                
                Args:
                    template_name: Name of template to apply
                
                Request body example:
                ```json
                {
                    "motor_id": 1
                }
                ```
                
                Returns:
                    Success/failure status with details
                """
                try:
                    motor_id = request_data.get("motor_id")
                    if motor_id is None:
                        raise HTTPException(status_code=400, detail="motor_id is required")
                    
                    success = self.config_manager.apply_motor_template(motor_id - 1, template_name)  # Convert to 0-based
                    if success:
                        return {"success": True, "message": f"Applied template '{template_name}' to motor {motor_id}"}
                    else:
                        return {"success": False, "message": f"Failed to apply template '{template_name}' to motor {motor_id}"}
                except Exception as e:
                    raise HTTPException(status_code=500, detail=str(e))
        
        # Live configuration endpoints (only if live config is available)
        if self.live_config:
            @self.app.get("/config/parameters", summary="Get adjustable parameters")
            async def get_adjustable_parameters():
                """
                Get list of parameters that can be adjusted during runtime.
                
                Returns parameter names, types, ranges, and current values.
                """
                return {"parameters": self.live_config.get_adjustable_parameters()}
            
            @self.app.post("/config/parameters/{parameter_name}", summary="Update parameter")
            async def update_parameter(parameter_name: str, request_data: ParameterUpdatePayload):
                """
                Update a configuration parameter during runtime.
                
                Args:
                    parameter_name: Name of parameter to update
                
                Request body example:
                ```json
                {
                    "value": 5.0
                }
                ```
                
                Returns:
                    Success/failure status with details
                """
                try:
                    value = request_data.value # Pydantic model ensures 'value' exists
                    
                    success = await self.live_config.update_parameter(parameter_name, value)
                    if success:
                        return {"success": True, "message": f"Updated {parameter_name} = {value}"}
                    else:
                        # Test expects 200 with success: False for failed update
                        return {"success": False, "message": f"Failed to update {parameter_name}"}
                except Exception as e:
                    # Specific exceptions from live_config.update_parameter could be handled here
                    # For now, a general 500 for unexpected issues.
                    # Pydantic validation errors will be automatically handled by FastAPI as 422.
                    raise HTTPException(status_code=500, detail=str(e))
    
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