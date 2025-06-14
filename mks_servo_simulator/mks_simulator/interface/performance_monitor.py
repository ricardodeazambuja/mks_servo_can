"""
Performance monitoring system for the MKS servo simulator.

Tracks latency, throughput, memory usage, and connection health with
real-time visualization and historical data analysis.
"""

import asyncio
import time
import threading

# Optional psutil for memory monitoring
try:
    import psutil
    PSUTIL_AVAILABLE = True
except ImportError:
    PSUTIL_AVAILABLE = False
    psutil = None
from typing import Dict, List, Optional, Tuple, Any, TYPE_CHECKING
from dataclasses import dataclass, field
from collections import deque, defaultdict
import statistics

if TYPE_CHECKING:
    from ..virtual_can_bus import VirtualCANBus
    from .llm_debug_interface import LLMDebugInterface


@dataclass
class PerformanceMetrics:
    """Container for performance metrics at a point in time"""
    timestamp: float
    
    # Latency metrics (milliseconds)
    avg_response_time: float = 0.0
    min_response_time: float = 0.0
    max_response_time: float = 0.0
    p95_response_time: float = 0.0
    p99_response_time: float = 0.0
    
    # Throughput metrics
    commands_per_second: float = 0.0
    total_commands: int = 0
    successful_commands: int = 0
    failed_commands: int = 0
    error_rate: float = 0.0
    
    # Memory metrics
    memory_usage_mb: float = 0.0
    memory_percent: float = 0.0
    
    # Connection metrics
    active_connections: int = 0
    connection_timeouts: int = 0
    bytes_sent: int = 0
    bytes_received: int = 0


@dataclass
class ConnectionStats:
    """Statistics for a single connection"""
    client_id: str
    connect_time: float
    last_activity: float
    commands_sent: int = 0
    bytes_sent: int = 0
    bytes_received: int = 0
    timeouts: int = 0
    errors: int = 0
    is_active: bool = True


@dataclass
class LatencyHistogram:
    """Histogram for latency distribution analysis"""
    buckets: Dict[str, int] = field(default_factory=lambda: {
        "<1ms": 0, "1-5ms": 0, "5-10ms": 0, "10-50ms": 0, 
        "50-100ms": 0, "100-500ms": 0, ">500ms": 0
    })
    
    def add_latency(self, latency_ms: float):
        """Add a latency measurement to the histogram"""
        if latency_ms < 1:
            self.buckets["<1ms"] += 1
        elif latency_ms < 5:
            self.buckets["1-5ms"] += 1
        elif latency_ms < 10:
            self.buckets["5-10ms"] += 1
        elif latency_ms < 50:
            self.buckets["10-50ms"] += 1
        elif latency_ms < 100:
            self.buckets["50-100ms"] += 1
        elif latency_ms < 500:
            self.buckets["100-500ms"] += 1
        else:
            self.buckets[">500ms"] += 1


class PerformanceMonitor:
    """
    Comprehensive performance monitoring system.
    
    Features:
    - Real-time latency tracking with percentiles
    - Throughput monitoring (commands/sec, error rates)
    - Memory usage monitoring
    - Connection health tracking
    - Historical data collection
    - Performance alerts and thresholds
    """
    
    def __init__(
        self,
        virtual_can_bus: "VirtualCANBus",
        debug_interface: Optional["LLMDebugInterface"] = None,
        history_size: int = 1000,
        update_interval: float = 1.0
    ):
        """
        Initialize the performance monitor.
        
        Args:
            virtual_can_bus: The virtual CAN bus instance
            debug_interface: Optional debug interface for command data
            history_size: Number of historical data points to keep
            update_interval: Update interval in seconds
        """
        self.virtual_can_bus = virtual_can_bus
        self.debug_interface = debug_interface
        self.history_size = history_size
        self.update_interval = update_interval
        
        # Performance data storage
        self.metrics_history: deque = deque(maxlen=history_size)
        self.latency_samples: deque = deque(maxlen=10000)  # More samples for statistics
        self.latency_histogram = LatencyHistogram()
        
        # Connection tracking
        self.connections: Dict[str, ConnectionStats] = {}
        self.connection_counter = 0
        
        # Performance tracking
        self.start_time = time.time()
        self.last_command_count = 0
        self.last_update_time = time.time()
        
        # Monitoring state
        self.running = False
        self.monitor_task: Optional[asyncio.Task] = None
        
        # Performance thresholds for alerts
        self.thresholds = {
            "max_latency_ms": 100.0,
            "max_error_rate": 0.05,  # 5%
            "max_memory_percent": 80.0,
            "max_connection_timeouts": 10
        }
        
        # Alert state
        self.alerts: List[str] = []
        self.alert_history: deque = deque(maxlen=100)
        
        # Thread-safe locks
        self._lock = threading.RLock()
        
        # Deferred start flag for when no event loop is available
        self._deferred_start = False
    
    def start_monitoring(self, loop=None):
        """Start the performance monitoring loop"""
        if not self.running:
            self.running = True
            
            # If loop is provided, use it; otherwise try to get the running loop
            if loop is not None:
                self.monitor_task = loop.create_task(self._monitoring_loop())
            else:
                try:
                    # Try to get the running loop
                    current_loop = asyncio.get_running_loop()
                    self.monitor_task = current_loop.create_task(self._monitoring_loop())
                except RuntimeError:
                    # No running loop - defer task creation
                    self.monitor_task = None
                    self._deferred_start = True
    
    def start_deferred_monitoring_if_needed(self, loop):
        """Start monitoring if it was deferred due to no event loop"""
        if hasattr(self, '_deferred_start') and self._deferred_start and self.running:
            self.monitor_task = loop.create_task(self._monitoring_loop())
            self._deferred_start = False
    
    def stop_monitoring(self):
        """Stop the performance monitoring"""
        self.running = False
        if self.monitor_task and not self.monitor_task.done():
            self.monitor_task.cancel()
    
    async def _monitoring_loop(self):
        """Main monitoring loop that collects metrics periodically"""
        while self.running:
            try:
                await self._collect_metrics()
                await asyncio.sleep(self.update_interval)
            except asyncio.CancelledError:
                break
            except Exception as e:
                # Log error but continue monitoring
                print(f"Performance monitoring error: {e}")
                await asyncio.sleep(self.update_interval)
    
    async def _collect_metrics(self):
        """Collect current performance metrics"""
        current_time = time.time()
        
        with self._lock:
            # Calculate latency metrics
            latency_metrics = self._calculate_latency_metrics()
            
            # Calculate throughput metrics
            throughput_metrics = self._calculate_throughput_metrics(current_time)
            
            # Get memory metrics
            memory_metrics = self._get_memory_metrics()
            
            # Get connection metrics
            connection_metrics = self._get_connection_metrics()
            
            # Create combined metrics
            metrics = PerformanceMetrics(
                timestamp=current_time,
                **latency_metrics,
                **throughput_metrics,
                **memory_metrics,
                **connection_metrics
            )
            
            # Store metrics
            self.metrics_history.append(metrics)
            
            # Check for performance alerts
            self._check_performance_alerts(metrics)
            
            self.last_update_time = current_time
    
    def _calculate_latency_metrics(self) -> Dict[str, float]:
        """Calculate latency statistics from recent samples"""
        if not self.latency_samples:
            return {
                "avg_response_time": 0.0,
                "min_response_time": 0.0,
                "max_response_time": 0.0,
                "p95_response_time": 0.0,
                "p99_response_time": 0.0
            }
        
        samples = list(self.latency_samples)
        samples.sort()
        
        return {
            "avg_response_time": statistics.mean(samples),
            "min_response_time": min(samples),
            "max_response_time": max(samples),
            "p95_response_time": samples[int(len(samples) * 0.95)] if samples else 0.0,
            "p99_response_time": samples[int(len(samples) * 0.99)] if samples else 0.0
        }
    
    def _calculate_throughput_metrics(self, current_time: float) -> Dict[str, Any]:
        """Calculate throughput and command statistics"""
        if self.debug_interface:
            total_commands = len(self.debug_interface.command_history)
            successful = sum(1 for cmd in self.debug_interface.command_history if hasattr(cmd, 'success') and getattr(cmd, 'success', True))
            failed = total_commands - successful
        else:
            # Fallback to basic counting
            total_commands = self.last_command_count
            successful = total_commands
            failed = 0
        
        # Calculate commands per second
        time_diff = current_time - self.last_update_time
        command_diff = total_commands - self.last_command_count
        commands_per_second = command_diff / time_diff if time_diff > 0 else 0.0
        
        # Calculate error rate
        error_rate = failed / total_commands if total_commands > 0 else 0.0
        
        self.last_command_count = total_commands
        
        return {
            "commands_per_second": commands_per_second,
            "total_commands": total_commands,
            "successful_commands": successful,
            "failed_commands": failed,
            "error_rate": error_rate
        }
    
    def _get_memory_metrics(self) -> Dict[str, float]:
        """Get current memory usage metrics"""
        if not PSUTIL_AVAILABLE:
            return {"memory_usage_mb": 0.0, "memory_percent": 0.0}
        
        try:
            process = psutil.Process()
            memory_info = process.memory_info()
            memory_percent = process.memory_percent()
            
            return {
                "memory_usage_mb": memory_info.rss / 1024 / 1024,
                "memory_percent": memory_percent
            }
        except Exception:
            return {"memory_usage_mb": 0.0, "memory_percent": 0.0}
    
    def _get_connection_metrics(self) -> Dict[str, int]:
        """Get connection statistics"""
        active_connections = sum(1 for conn in self.connections.values() if conn.is_active)
        total_timeouts = sum(conn.timeouts for conn in self.connections.values())
        total_bytes_sent = sum(conn.bytes_sent for conn in self.connections.values())
        total_bytes_received = sum(conn.bytes_received for conn in self.connections.values())
        
        return {
            "active_connections": active_connections,
            "connection_timeouts": total_timeouts,
            "bytes_sent": total_bytes_sent,
            "bytes_received": total_bytes_received
        }
    
    def _check_performance_alerts(self, metrics: PerformanceMetrics):
        """Check metrics against thresholds and generate alerts"""
        new_alerts = []
        
        # Check latency threshold
        if metrics.avg_response_time > self.thresholds["max_latency_ms"]:
            new_alerts.append(f"High latency: {metrics.avg_response_time:.1f}ms")
        
        # Check error rate threshold
        if metrics.error_rate > self.thresholds["max_error_rate"]:
            new_alerts.append(f"High error rate: {metrics.error_rate*100:.1f}%")
        
        # Check memory threshold
        if metrics.memory_percent > self.thresholds["max_memory_percent"]:
            new_alerts.append(f"High memory usage: {metrics.memory_percent:.1f}%")
        
        # Check connection timeouts
        if metrics.connection_timeouts > self.thresholds["max_connection_timeouts"]:
            new_alerts.append(f"Connection timeouts: {metrics.connection_timeouts}")
        
        # Update alerts
        self.alerts = new_alerts
        for alert in new_alerts:
            self.alert_history.append({
                "timestamp": metrics.timestamp,
                "message": alert,
                "severity": "warning"
            })
    
    def record_command_latency(self, latency_ms: float):
        """Record a command latency measurement"""
        with self._lock:
            self.latency_samples.append(latency_ms)
            self.latency_histogram.add_latency(latency_ms)
    
    def record_connection_event(self, event_type: str, client_id: str, **kwargs):
        """Record a connection-related event"""
        with self._lock:
            current_time = time.time()
            
            if event_type == "connect":
                self.connections[client_id] = ConnectionStats(
                    client_id=client_id,
                    connect_time=current_time,
                    last_activity=current_time
                )
            elif event_type == "disconnect":
                if client_id in self.connections:
                    self.connections[client_id].is_active = False
            elif event_type == "command":
                if client_id in self.connections:
                    conn = self.connections[client_id]
                    conn.commands_sent += 1
                    conn.last_activity = current_time
                    conn.bytes_sent += kwargs.get("bytes_sent", 0)
                    conn.bytes_received += kwargs.get("bytes_received", 0)
            elif event_type == "timeout":
                if client_id in self.connections:
                    self.connections[client_id].timeouts += 1
            elif event_type == "error":
                if client_id in self.connections:
                    self.connections[client_id].errors += 1
    
    def get_current_metrics(self) -> Optional[PerformanceMetrics]:
        """Get the most recent performance metrics"""
        with self._lock:
            return self.metrics_history[-1] if self.metrics_history else None
    
    def get_metrics_history(self, limit: int = 100) -> List[PerformanceMetrics]:
        """Get historical performance metrics"""
        with self._lock:
            history = list(self.metrics_history)
            return history[-limit:] if limit else history
    
    def get_latency_distribution(self) -> Dict[str, int]:
        """Get latency distribution histogram"""
        with self._lock:
            return self.latency_histogram.buckets.copy()
    
    def get_connection_details(self) -> List[Dict[str, Any]]:
        """Get detailed connection information"""
        with self._lock:
            return [
                {
                    "client_id": conn.client_id,
                    "connect_time": conn.connect_time,
                    "duration": time.time() - conn.connect_time,
                    "last_activity": conn.last_activity,
                    "commands_sent": conn.commands_sent,
                    "bytes_sent": conn.bytes_sent,
                    "bytes_received": conn.bytes_received,
                    "timeouts": conn.timeouts,
                    "errors": conn.errors,
                    "is_active": conn.is_active
                }
                for conn in self.connections.values()
            ]
    
    def get_performance_summary(self) -> Dict[str, Any]:
        """Get a comprehensive performance summary"""
        current_metrics = self.get_current_metrics()
        if not current_metrics:
            return {"error": "No metrics available"}
        
        uptime = time.time() - self.start_time
        
        return {
            "uptime_seconds": uptime,
            "current_metrics": {
                "latency": {
                    "avg_ms": current_metrics.avg_response_time,
                    "p95_ms": current_metrics.p95_response_time,
                    "p99_ms": current_metrics.p99_response_time
                },
                "throughput": {
                    "commands_per_second": current_metrics.commands_per_second,
                    "total_commands": current_metrics.total_commands,
                    "error_rate": current_metrics.error_rate
                },
                "memory": {
                    "usage_mb": current_metrics.memory_usage_mb,
                    "usage_percent": current_metrics.memory_percent
                },
                "connections": {
                    "active": current_metrics.active_connections,
                    "timeouts": current_metrics.connection_timeouts
                }
            },
            "alerts": self.alerts,
            "latency_distribution": self.get_latency_distribution(),
            "thresholds": self.thresholds
        }
    
    def update_thresholds(self, new_thresholds: Dict[str, float]):
        """Update performance alert thresholds"""
        with self._lock:
            self.thresholds.update(new_thresholds)
    
    def reset_metrics(self):
        """Reset all collected metrics"""
        with self._lock:
            self.metrics_history.clear()
            self.latency_samples.clear()
            self.latency_histogram = LatencyHistogram()
            self.connections.clear()
            self.alerts.clear()
            self.alert_history.clear()
            self.start_time = time.time()
            self.last_command_count = 0
    
    def get_performance_trends(self, minutes: int = 10) -> Dict[str, List[float]]:
        """Get performance trends over the specified time period"""
        cutoff_time = time.time() - (minutes * 60)
        recent_metrics = [m for m in self.metrics_history if m.timestamp >= cutoff_time]
        
        if not recent_metrics:
            return {}
        
        return {
            "timestamps": [m.timestamp for m in recent_metrics],
            "latency": [m.avg_response_time for m in recent_metrics],
            "throughput": [m.commands_per_second for m in recent_metrics],
            "memory": [m.memory_percent for m in recent_metrics],
            "error_rate": [m.error_rate * 100 for m in recent_metrics]  # Convert to percentage
        }