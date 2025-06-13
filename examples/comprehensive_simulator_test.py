#!/usr/bin/env python3
"""
Comprehensive Simulator Test Suite

This script performs comprehensive testing of the enhanced MKS servo simulator,
including all Phase 2 features: dashboard, configuration management, 
performance monitoring, and API integration.

Usage:
1. Start simulator: mks-servo-simulator --dashboard --debug-api --num-motors 3
2. Run this test: python comprehensive_simulator_test.py
3. Observe results and verify functionality

Requirements:
- mks-servo-simulator with all enhanced features
- mks-servo-can library
- requests library for API testing
"""

import asyncio
import requests
import json
import time
from typing import Dict, Any, List, Optional
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime


@dataclass
class TestResult:
    """Represents the result of a single test."""
    name: str
    passed: bool
    message: str
    duration_ms: float
    details: Optional[Dict[str, Any]] = None


class SimulatorTestSuite:
    """Comprehensive test suite for the enhanced simulator."""
    
    def __init__(self, api_base: str = "http://localhost:8765", sim_base: str = "localhost:6789"):
        self.api_base = api_base
        self.sim_base = sim_base
        self.results: List[TestResult] = []
        
    def log(self, message: str, level: str = "INFO"):
        """Log a message with timestamp."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        print(f"[{timestamp}] {level}: {message}")
    
    def time_test(self, test_func):
        """Decorator to time test execution."""
        def wrapper(*args, **kwargs):
            start_time = time.time()
            result = test_func(*args, **kwargs)
            duration = (time.time() - start_time) * 1000  # Convert to ms
            return result, duration
        return wrapper
    
    def test_api_connectivity(self) -> TestResult:
        """Test basic API connectivity."""
        start_time = time.time()
        try:
            response = requests.get(f"{self.api_base}/health", timeout=5)
            duration = (time.time() - start_time) * 1000
            
            if response.status_code == 200:
                data = response.json()
                return TestResult(
                    "API Connectivity",
                    True,
                    f"API healthy, uptime: {data.get('uptime', 'N/A')}s",
                    duration,
                    data
                )
            else:
                return TestResult(
                    "API Connectivity",
                    False,
                    f"API returned status {response.status_code}",
                    duration
                )
        except Exception as e:
            duration = (time.time() - start_time) * 1000
            return TestResult(
                "API Connectivity",
                False,
                f"API connection failed: {str(e)}",
                duration
            )
    
    def test_configuration_management(self) -> TestResult:
        """Test configuration management features."""
        start_time = time.time()
        try:
            # Test getting current config
            response = requests.get(f"{self.api_base}/config")
            if response.status_code != 200:
                raise Exception(f"Failed to get config: {response.status_code}")
            
            config = response.json()
            
            # Test listing profiles
            response = requests.get(f"{self.api_base}/config/profiles")
            if response.status_code != 200:
                raise Exception(f"Failed to list profiles: {response.status_code}")
            
            profiles = response.json()["profiles"]
            
            # Test getting templates
            response = requests.get(f"{self.api_base}/config/templates")
            if response.status_code != 200:
                raise Exception(f"Failed to get templates: {response.status_code}")
            
            templates = response.json()["templates"]
            
            duration = (time.time() - start_time) * 1000
            return TestResult(
                "Configuration Management",
                True,
                f"Config loaded, {len(profiles)} profiles, {len(templates)} templates",
                duration,
                {"config": config, "profiles": profiles, "templates": list(templates.keys())}
            )
            
        except Exception as e:
            duration = (time.time() - start_time) * 1000
            return TestResult(
                "Configuration Management",
                False,
                f"Configuration test failed: {str(e)}",
                duration
            )
    
    def test_live_parameter_adjustment(self) -> TestResult:
        """Test live parameter adjustment."""
        start_time = time.time()
        try:
            # Get adjustable parameters
            response = requests.get(f"{self.api_base}/config/parameters")
            if response.status_code != 200:
                raise Exception(f"Failed to get parameters: {response.status_code}")
            
            parameters = response.json()["parameters"]
            
            # Test updating latency (safe parameter to modify)
            original_latency = None
            test_latency = 4.5
            
            # Try to update latency
            update_data = {"value": test_latency}
            response = requests.post(
                f"{self.api_base}/config/parameters/latency_ms",
                json=update_data
            )
            
            if response.status_code != 200:
                raise Exception(f"Failed to update latency: {response.status_code}")
            
            update_result = response.json()
            if not update_result.get("success", False):
                raise Exception(f"Parameter update reported failure: {update_result.get('message', 'Unknown error')}")
            
            duration = (time.time() - start_time) * 1000
            return TestResult(
                "Live Parameter Adjustment",
                True,
                f"Successfully updated latency to {test_latency}ms",
                duration,
                {"parameters": list(parameters.keys()), "update_result": update_result}
            )
            
        except Exception as e:
            duration = (time.time() - start_time) * 1000
            return TestResult(
                "Live Parameter Adjustment",
                False,
                f"Parameter adjustment test failed: {str(e)}",
                duration
            )
    
    def test_command_injection(self) -> TestResult:
        """Test command injection functionality."""
        start_time = time.time()
        try:
            # Test getting templates
            response = requests.get(f"{self.api_base}/templates")
            if response.status_code != 200:
                raise Exception(f"Failed to get injection templates: {response.status_code}")
            
            templates = response.json()["templates"]
            
            # Test template command injection (safe enable command)
            injection_data = {
                "motor_id": 1,
                "template_name": "enable"
            }
            
            response = requests.post(
                f"{self.api_base}/inject_template",
                json=injection_data
            )
            
            if response.status_code != 200:
                raise Exception(f"Template injection failed: {response.status_code}")
            
            result = response.json()
            if not result.get("success", False):
                raise Exception(f"Template injection reported failure: {result.get('error_message', 'Unknown error')}")
            
            duration = (time.time() - start_time) * 1000
            return TestResult(
                "Command Injection",
                True,
                f"Successfully injected 'enable' template, execution time: {result.get('execution_time_ms', 'N/A')}ms",
                duration,
                {"templates": list(templates.keys()), "injection_result": result}
            )
            
        except Exception as e:
            duration = (time.time() - start_time) * 1000
            return TestResult(
                "Command Injection",
                False,
                f"Command injection test failed: {str(e)}",
                duration
            )
    
    def test_performance_monitoring(self) -> TestResult:
        """Test performance monitoring features."""
        start_time = time.time()
        try:
            # Get current performance metrics
            response = requests.get(f"{self.api_base}/performance")
            if response.status_code != 200:
                raise Exception(f"Failed to get performance metrics: {response.status_code}")
            
            metrics = response.json()
            
            # Check required performance fields
            required_fields = ["latency", "throughput", "memory", "connections"]
            missing_fields = [field for field in required_fields if field not in metrics]
            
            if missing_fields:
                raise Exception(f"Missing performance fields: {missing_fields}")
            
            # Get performance history
            response = requests.get(f"{self.api_base}/performance/history")
            if response.status_code != 200:
                raise Exception(f"Failed to get performance history: {response.status_code}")
            
            history = response.json()
            
            duration = (time.time() - start_time) * 1000
            latency_avg = metrics["latency"].get("average_ms", "N/A")
            throughput_cps = metrics["throughput"].get("commands_per_second", "N/A")
            
            return TestResult(
                "Performance Monitoring",
                True,
                f"Latency: {latency_avg}ms avg, Throughput: {throughput_cps} cmd/s",
                duration,
                {"metrics": metrics, "history_points": len(history.get("data_points", []))}
            )
            
        except Exception as e:
            duration = (time.time() - start_time) * 1000
            return TestResult(
                "Performance Monitoring",
                False,
                f"Performance monitoring test failed: {str(e)}",
                duration
            )
    
    async def test_library_integration(self) -> TestResult:
        """Test integration with the mks-servo-can library."""
        start_time = time.time()
        try:
            # Import the library
            from mks_servo_can import CANInterface, Axis, RotaryKinematics, const
            
            # Connect to simulator
            can_if = CANInterface(use_simulator=True, simulator_host='localhost', simulator_port=6789)
            await can_if.connect()
            
            # Create axis
            kin = RotaryKinematics(steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION)
            motor1 = Axis(can_if, motor_can_id=1, name="TestMotor", kinematics=kin)
            
            # Test basic communication
            await motor1.initialize()
            
            # Test getting position
            position = await motor1.get_current_position_user()
            
            # Cleanup
            await can_if.disconnect()
            
            duration = (time.time() - start_time) * 1000
            return TestResult(
                "Library Integration",
                True,
                f"Successfully connected and read position: {position:.2f} degrees",
                duration,
                {"position_degrees": position}
            )
            
        except Exception as e:
            duration = (time.time() - start_time) * 1000
            return TestResult(
                "Library Integration",
                False,
                f"Library integration test failed: {str(e)}",
                duration
            )
    
    def test_motor_templates(self) -> TestResult:
        """Test motor template application."""
        start_time = time.time()
        try:
            # Get available templates
            response = requests.get(f"{self.api_base}/config/templates")
            if response.status_code != 200:
                raise Exception(f"Failed to get templates: {response.status_code}")
            
            templates = response.json()["templates"]
            
            # Test applying a template
            template_data = {"motor_id": 1}
            response = requests.post(
                f"{self.api_base}/config/templates/servo42d/apply",
                json=template_data
            )
            
            if response.status_code != 200:
                raise Exception(f"Failed to apply template: {response.status_code}")
            
            result = response.json()
            if not result.get("success", False):
                raise Exception(f"Template application failed: {result.get('message', 'Unknown error')}")
            
            duration = (time.time() - start_time) * 1000
            return TestResult(
                "Motor Templates",
                True,
                f"Successfully applied servo42d template to motor 1",
                duration,
                {"available_templates": list(templates.keys()), "apply_result": result}
            )
            
        except Exception as e:
            duration = (time.time() - start_time) * 1000
            return TestResult(
                "Motor Templates",
                False,
                f"Motor template test failed: {str(e)}",
                duration
            )
    
    async def run_all_tests(self) -> Dict[str, Any]:
        """Run all tests and return comprehensive results."""
        self.log("Starting comprehensive simulator test suite...")
        
        # Define test methods
        test_methods = [
            ("Basic API", self.test_api_connectivity),
            ("Configuration Management", self.test_configuration_management),
            ("Live Parameters", self.test_live_parameter_adjustment),
            ("Command Injection", self.test_command_injection),
            ("Performance Monitoring", self.test_performance_monitoring),
            ("Motor Templates", self.test_motor_templates),
        ]
        
        # Run synchronous tests
        for test_name, test_method in test_methods:
            self.log(f"Running {test_name} test...")
            result = test_method()
            self.results.append(result)
            
            status = "PASS" if result.passed else "FAIL"
            self.log(f"{test_name}: {status} - {result.message} ({result.duration_ms:.1f}ms)")
        
        # Run asynchronous tests
        self.log("Running Library Integration test...")
        library_result = await self.test_library_integration()
        self.results.append(library_result)
        
        status = "PASS" if library_result.passed else "FAIL"
        self.log(f"Library Integration: {status} - {library_result.message} ({library_result.duration_ms:.1f}ms)")
        
        # Calculate summary
        total_tests = len(self.results)
        passed_tests = sum(1 for r in self.results if r.passed)
        failed_tests = total_tests - passed_tests
        total_duration = sum(r.duration_ms for r in self.results)
        
        summary = {
            "timestamp": datetime.now().isoformat(),
            "total_tests": total_tests,
            "passed": passed_tests,
            "failed": failed_tests,
            "success_rate": (passed_tests / total_tests) * 100 if total_tests > 0 else 0,
            "total_duration_ms": total_duration,
            "results": [
                {
                    "name": r.name,
                    "passed": r.passed,
                    "message": r.message,
                    "duration_ms": r.duration_ms,
                    "details": r.details
                }
                for r in self.results
            ]
        }
        
        return summary
    
    def print_summary(self, summary: Dict[str, Any]):
        """Print a formatted test summary."""
        print("\\n" + "=" * 70)
        print("🧪 COMPREHENSIVE SIMULATOR TEST RESULTS")
        print("=" * 70)
        
        print(f"📊 Test Summary:")
        print(f"   Total Tests: {summary['total_tests']}")
        print(f"   Passed: {summary['passed']} ✅")
        print(f"   Failed: {summary['failed']} ❌")
        print(f"   Success Rate: {summary['success_rate']:.1f}%")
        print(f"   Total Duration: {summary['total_duration_ms']:.1f}ms")
        
        print("\\n📋 Detailed Results:")
        for result in summary['results']:
            status_icon = "✅" if result['passed'] else "❌"
            print(f"   {status_icon} {result['name']}: {result['message']} ({result['duration_ms']:.1f}ms)")
        
        print("\\n" + "=" * 70)
        
        if summary['failed'] == 0:
            print("🎉 ALL TESTS PASSED! The simulator is working perfectly.")
        else:
            print(f"⚠️ {summary['failed']} test(s) failed. Check the details above.")
        
        print("=" * 70)


def check_simulator_running() -> bool:
    """Check if the simulator is running."""
    try:
        response = requests.get("http://localhost:8765/health", timeout=2)
        return response.status_code == 200
    except:
        return False


async def main():
    """Main function to run the test suite."""
    print("🚀 MKS Servo Simulator - Comprehensive Test Suite")
    print("=" * 55)
    
    # Check if simulator is running
    if not check_simulator_running():
        print("❌ Error: Simulator API not available!")
        print("\\nPlease start the simulator with all features enabled:")
        print("  mks-servo-simulator --dashboard --debug-api --num-motors 3")
        print("\\nThen run this test again.")
        return
    
    print("✅ Simulator API detected")
    
    # Run tests
    test_suite = SimulatorTestSuite()
    summary = await test_suite.run_all_tests()
    
    # Print results
    test_suite.print_summary(summary)
    
    # Save results to file
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"simulator_test_results_{timestamp}.json"
    
    with open(filename, 'w') as f:
        json.dump(summary, f, indent=2)
    
    print(f"\\n📄 Detailed results saved to: {filename}")


if __name__ == "__main__":
    asyncio.run(main())