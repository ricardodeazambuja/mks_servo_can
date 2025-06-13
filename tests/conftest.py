"""
Shared test fixtures for MKS Servo CAN simulator management.

This module provides reusable fixtures for starting and managing the simulator
across different test suites, eliminating the need for direct simulator imports
and ensuring consistent test environments.

## Key Features

- **Automated Simulator Management**: Fixtures automatically start and stop
  simulator processes for each test module.
- **Multiple Simulator Configurations**: Different fixtures for integration,
  compliance, and performance testing with appropriate configurations.
- **Proper Resource Cleanup**: Ensures simulator processes are terminated
  cleanly after tests complete.
- **Parallel Test Safety**: Uses different ports for different test types
  to avoid conflicts during concurrent test execution.

## Usage Patterns

### For Protocol Compliance Testing:
```python
@pytest.mark.compliance
class TestProtocolCompliance:
    @pytest.mark.asyncio
    async def test_command(self, compliance_api):
        result = await compliance_api.read_encoder_value_carry(can_id=1)
        assert isinstance(result, tuple)
```

### For Integration Testing:
```python
@pytest.mark.integration
class TestIntegration:
    @pytest.mark.asyncio
    async def test_feature(self, basic_api):
        await basic_api.enable_motor(can_id=1)
        # Test integration scenarios
```

### For Performance Testing:
```python
@pytest.mark.performance
class TestPerformance:
    @pytest.mark.asyncio
    async def test_throughput(self, performance_api):
        # Test performance scenarios with realistic latency
```

## Migration from Direct Imports

**Before (problematic pattern):**
```python
try:
    from mks_simulator.virtual_can_bus import VirtualCANBus
    from mks_simulator.motor_model import MotorModel
except ImportError:
    pytest.skip("Simulator not available for testing")
```

**After (using shared fixtures):**
```python
@pytest.mark.asyncio
async def test_feature(self, compliance_api):
    # Use compliance_api directly - simulator management is automatic
    result = await compliance_api.some_command(can_id=1)
```
"""

import pytest
import pytest_asyncio
import subprocess
import time
import asyncio
from typing import Optional, Dict, Any

from mks_servo_can import CANInterface, LowLevelAPI, exceptions


# Simulator configuration constants
SIMULATOR_HOST = "localhost"
DEFAULT_SIMULATOR_PORT = 6789
COMPLIANCE_SIMULATOR_PORT = 6790
PERFORMANCE_SIMULATOR_PORT = 6791
SIMULATOR_STARTUP_TIMEOUT = 5  # seconds
SIMULATOR_CMD = "mks-servo-simulator"


class SimulatorManager:
    """Manages simulator subprocess lifecycle for tests"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.process: Optional[subprocess.Popen] = None
        self.port = config.get("port", DEFAULT_SIMULATOR_PORT)
        self.num_motors = config.get("num_motors", 2)
        self.start_can_id = config.get("start_can_id", 1)
        self.log_level = config.get("log_level", "INFO")
        self.latency_ms = config.get("latency_ms", 1)
        
    def start(self) -> None:
        """Start the simulator subprocess"""
        # Check if simulator command exists
        try:
            subprocess.check_output(
                f"command -v {SIMULATOR_CMD}", shell=True, text=True
            ).strip()
        except (subprocess.CalledProcessError, FileNotFoundError):
            pytest.skip(
                f"'{SIMULATOR_CMD}' not found in PATH. Skipping simulator-dependent tests."
            )
            return
        
        cmd = [
            SIMULATOR_CMD,
            "--port", str(self.port),
            "--num-motors", str(self.num_motors),
            "--start-can-id", str(self.start_can_id),
            "--log-level", self.log_level,
        ]
        
        # Add latency parameter if supported
        if self.latency_ms is not None:
            cmd.extend(["--latency-ms", str(self.latency_ms)])
        
        print(f"\nStarting simulator: {' '.join(cmd)}")
        
        try:
            self.process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            
            print(f"Simulator starting with PID: {self.process.pid}")
            
            # Wait for simulator to initialize
            time.sleep(SIMULATOR_STARTUP_TIMEOUT)
            
            # Check if process is still running
            if self.process.poll() is not None:
                stdout, stderr = self.process.communicate()
                error_msg = (
                    f"Simulator failed to start or stay running. Exit code: {self.process.returncode}\n"
                    f"STDOUT: {stdout.decode(errors='replace')}\n"
                    f"STDERR: {stderr.decode(errors='replace')}"
                )
                pytest.fail(error_msg)
            
            print("Simulator started successfully")
            
        except Exception as e:
            pytest.fail(f"Failed to start simulator: {e}")
    
    def stop(self) -> None:
        """Stop the simulator subprocess"""
        if self.process and self.process.poll() is None:
            print(f"\nTerminating simulator process (PID: {self.process.pid})")
            self.process.terminate()
            
            try:
                self.process.communicate(timeout=5)
                print("Simulator terminated gracefully")
            except subprocess.TimeoutExpired:
                print("Simulator did not terminate gracefully, killing")
                self.process.kill()
                self.process.communicate()
                print("Simulator killed")
        elif self.process:
            print("Simulator process already terminated")
    
    def is_running(self) -> bool:
        """Check if simulator is still running"""
        return self.process is not None and self.process.poll() is None


@pytest.fixture(scope="module")
def basic_simulator():
    """
    Basic simulator fixture for integration tests.
    Provides 2 motors with minimal latency.
    """
    config = {
        "port": DEFAULT_SIMULATOR_PORT,
        "num_motors": 2,
        "start_can_id": 1,
        "log_level": "INFO",
        "latency_ms": 1
    }
    
    manager = SimulatorManager(config)
    manager.start()
    
    yield manager
    
    manager.stop()


@pytest.fixture(scope="module") 
def compliance_simulator():
    """
    Compliance simulator fixture for protocol testing.
    Provides 3 motors for comprehensive testing.
    """
    config = {
        "port": COMPLIANCE_SIMULATOR_PORT,
        "num_motors": 3,
        "start_can_id": 1,
        "log_level": "INFO",
        "latency_ms": 1
    }
    
    manager = SimulatorManager(config)
    manager.start()
    
    yield manager
    
    manager.stop()


@pytest.fixture(scope="module")
def performance_simulator():
    """
    Performance simulator fixture for load testing.
    Provides 4 motors with realistic latency.
    """
    config = {
        "port": PERFORMANCE_SIMULATOR_PORT,
        "num_motors": 4,
        "start_can_id": 1,
        "log_level": "WARNING",  # Reduce log noise for performance tests
        "latency_ms": 5
    }
    
    manager = SimulatorManager(config)
    manager.start()
    
    yield manager
    
    manager.stop()


@pytest_asyncio.fixture(scope="function")
async def basic_can_interface(basic_simulator: SimulatorManager):
    """
    Provides a CANInterface connected to the basic simulator.
    Function-scoped for clean state per test.
    """
    iface = CANInterface(
        use_simulator=True,
        simulator_host=SIMULATOR_HOST,
        simulator_port=basic_simulator.port,
    )
    
    try:
        await iface.connect()
        print(f"Connected to basic simulator on port {basic_simulator.port}")
        yield iface
    except exceptions.SimulatorError as e:
        pytest.fail(
            f"Failed to connect to basic simulator: {e}. "
            f"Ensure simulator is running on port {basic_simulator.port}"
        )
    finally:
        print("Disconnecting from basic simulator")
        await iface.disconnect()


@pytest_asyncio.fixture(scope="function")
async def compliance_can_interface(compliance_simulator: SimulatorManager):
    """
    Provides a CANInterface connected to the compliance simulator.
    Function-scoped for clean state per test.
    """
    iface = CANInterface(
        use_simulator=True,
        simulator_host=SIMULATOR_HOST,
        simulator_port=compliance_simulator.port,
    )
    
    try:
        await iface.connect()
        print(f"Connected to compliance simulator on port {compliance_simulator.port}")
        yield iface
    except exceptions.SimulatorError as e:
        pytest.fail(
            f"Failed to connect to compliance simulator: {e}. "
            f"Ensure simulator is running on port {compliance_simulator.port}"
        )
    finally:
        print("Disconnecting from compliance simulator")
        await iface.disconnect()


@pytest_asyncio.fixture(scope="function")
async def performance_can_interface(performance_simulator: SimulatorManager):
    """
    Provides a CANInterface connected to the performance simulator.
    Function-scoped for clean state per test.
    """
    iface = CANInterface(
        use_simulator=True,
        simulator_host=SIMULATOR_HOST,
        simulator_port=performance_simulator.port,
    )
    
    try:
        await iface.connect()
        print(f"Connected to performance simulator on port {performance_simulator.port}")
        yield iface
    except exceptions.SimulatorError as e:
        pytest.fail(
            f"Failed to connect to performance simulator: {e}. "
            f"Ensure simulator is running on port {performance_simulator.port}"
        )
    finally:
        print("Disconnecting from performance simulator")
        await iface.disconnect()


@pytest_asyncio.fixture(scope="function")
async def compliance_api(compliance_can_interface: CANInterface):
    """
    Provides a LowLevelAPI instance connected to the compliance simulator.
    Used for protocol compliance testing.
    """
    api = LowLevelAPI(compliance_can_interface)
    return api


@pytest_asyncio.fixture(scope="function") 
async def basic_api(basic_can_interface: CANInterface):
    """
    Provides a LowLevelAPI instance connected to the basic simulator.
    Used for integration testing.
    """
    api = LowLevelAPI(basic_can_interface)
    return api


@pytest_asyncio.fixture(scope="function")
async def performance_api(performance_can_interface: CANInterface):
    """
    Provides a LowLevelAPI instance connected to the performance simulator.
    Used for performance testing.
    """
    api = LowLevelAPI(performance_can_interface)
    return api


# Utility fixtures for common test patterns
@pytest.fixture
def simulator_health_check():
    """
    Utility fixture for checking simulator health before tests.
    Can be used as a dependency to ensure simulator is responsive.
    """
    def check_health(manager: SimulatorManager) -> bool:
        """Check if simulator is healthy and responsive"""
        if not manager.is_running():
            pytest.fail("Simulator process is not running")
        
        # Additional health checks could be added here
        # e.g., attempting a simple connection test
        return True
    
    return check_health


@pytest.fixture
def test_motors_config():
    """
    Provides test motor configuration data for compliance testing.
    """
    return {
        1: {"enabled": True, "encoder_position": 0x1000},
        2: {"enabled": True, "encoder_position": 0x2000}, 
        3: {"enabled": False, "encoder_position": 0x3000},
    }


# Marks for different test categories
pytest_mark_integration = pytest.mark.integration
pytest_mark_compliance = pytest.mark.compliance
pytest_mark_performance = pytest.mark.performance

# Skip markers for conditional tests
requires_simulator = pytest.mark.skipif(
    subprocess.run(
        f"command -v {SIMULATOR_CMD}", 
        shell=True, 
        capture_output=True
    ).returncode != 0,
    reason="mks-servo-simulator not available"
)