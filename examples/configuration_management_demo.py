#!/usr/bin/env python3
"""
Configuration Management Demo for MKS Servo Simulator

This example demonstrates the new configuration management features:
- Creating and saving configuration profiles
- Loading configurations with different motor setups
- Using motor templates for quick configuration
- Live parameter adjustment during simulation

Requirements:
- mks-servo-simulator installed with enhanced features
- Run this script while the simulator is running with --debug-api flag
"""

import asyncio
import json
import requests
from typing import Dict, Any
import time

# Simulator API base URL
API_BASE = "http://localhost:8765"


class ConfigurationDemo:
    """Demonstrates configuration management features."""
    
    def __init__(self, api_base: str = API_BASE):
        self.api_base = api_base
        
    def check_api_availability(self) -> bool:
        """Check if the simulator API is available."""
        try:
            response = requests.get(f"{self.api_base}/health", timeout=2)
            return response.status_code == 200
        except requests.RequestException:
            return False
    
    def get_current_config(self) -> Dict[str, Any]:
        """Get current simulator configuration."""
        response = requests.get(f"{self.api_base}/config")
        response.raise_for_status()
        return response.json()
    
    def list_profiles(self) -> list:
        """List available configuration profiles."""
        response = requests.get(f"{self.api_base}/config/profiles")
        response.raise_for_status()
        return response.json()["profiles"]
    
    def save_profile(self, profile_name: str) -> bool:
        """Save current configuration as a profile."""
        response = requests.post(f"{self.api_base}/config/profiles/{profile_name}/save")
        response.raise_for_status()
        return response.json()["success"]
    
    def load_profile(self, profile_name: str) -> bool:
        """Load a configuration profile."""
        response = requests.post(f"{self.api_base}/config/profiles/{profile_name}/load")
        response.raise_for_status()
        return response.json()["success"]
    
    def get_motor_templates(self) -> Dict[str, Any]:
        """Get available motor templates."""
        response = requests.get(f"{self.api_base}/config/templates")
        response.raise_for_status()
        return response.json()["templates"]
    
    def apply_motor_template(self, motor_id: int, template_name: str) -> bool:
        """Apply a motor template to a specific motor."""
        data = {"motor_id": motor_id}
        response = requests.post(
            f"{self.api_base}/config/templates/{template_name}/apply",
            json=data
        )
        response.raise_for_status()
        return response.json()["success"]
    
    def get_adjustable_parameters(self) -> Dict[str, Any]:
        """Get list of parameters that can be adjusted live."""
        response = requests.get(f"{self.api_base}/config/parameters")
        response.raise_for_status()
        return response.json()["parameters"]
    
    def update_parameter(self, parameter_name: str, value: Any) -> bool:
        """Update a configuration parameter live."""
        data = {"value": value}
        response = requests.post(
            f"{self.api_base}/config/parameters/{parameter_name}",
            json=data
        )
        response.raise_for_status()
        return response.json()["success"]
    
    def demo_configuration_profiles(self):
        """Demonstrate configuration profile management."""
        print("=== Configuration Profile Management Demo ===")
        
        # Show current configuration
        print("\\n1. Current Configuration:")
        current_config = self.get_current_config()
        print(f"   Name: {current_config.get('name', 'N/A')}")
        print(f"   Motors: {current_config.get('num_motors', 0)}")
        print(f"   Host: {current_config.get('host', 'N/A')}")
        print(f"   Port: {current_config.get('port', 'N/A')}")
        print(f"   Latency: {current_config.get('latency_ms', 'N/A')}ms")
        
        # List existing profiles
        print("\\n2. Available Profiles:")
        profiles = self.list_profiles()
        if profiles:
            for profile in profiles:
                print(f"   - {profile}")
        else:
            print("   No profiles found")
        
        # Save current configuration as demo profile
        profile_name = f"demo_profile_{int(time.time())}"
        print(f"\\n3. Saving current configuration as '{profile_name}'...")
        success = self.save_profile(profile_name)
        print(f"   Success: {success}")
        
        # List profiles again to show the new one
        print("\\n4. Updated Profile List:")
        profiles = self.list_profiles()
        for profile in profiles:
            print(f"   - {profile}")
    
    def demo_motor_templates(self):
        """Demonstrate motor template usage."""
        print("\\n=== Motor Template Management Demo ===")
        
        # Show available templates
        print("\\n1. Available Motor Templates:")
        templates = self.get_motor_templates()
        for name, template in templates.items():
            print(f"   {name}:")
            print(f"     Type: {template.get('motor_type', 'N/A')}")
            print(f"     Steps/Rev: {template.get('steps_per_rev', 'N/A')}")
            print(f"     Max Current: {template.get('max_current', 'N/A')}mA")
            print(f"     Max Speed: {template.get('max_speed', 'N/A')} steps/s")
            print(f"     Description: {template.get('description', 'N/A')}")
            print()
        
        # Apply templates to motors (if they exist)
        current_config = self.get_current_config()
        if current_config.get("num_motors", 0) > 0:
            print("2. Applying templates to motors...")
            
            # Apply servo42d template to first motor
            print("   Applying 'servo42d' template to motor 1...")
            success = self.apply_motor_template(1, "servo42d")
            print(f"   Success: {success}")
            
            # Apply high_precision template to second motor if it exists
            if current_config.get("num_motors", 0) > 1:
                print("   Applying 'high_precision' template to motor 2...")
                success = self.apply_motor_template(2, "high_precision")
                print(f"   Success: {success}")
        else:
            print("2. No motors available to apply templates")
    
    def demo_live_parameters(self):
        """Demonstrate live parameter adjustment."""
        print("\\n=== Live Parameter Adjustment Demo ===")
        
        # Show adjustable parameters
        print("\\n1. Available Adjustable Parameters:")
        parameters = self.get_adjustable_parameters()
        for param_name, param_info in parameters.items():
            current = param_info.get('current', 'N/A')
            param_type = param_info.get('type', 'unknown')
            description = param_info.get('description', 'No description')
            print(f"   {param_name} ({param_type}): {description}")
            print(f"     Current value: {current}")
            if 'min' in param_info and 'max' in param_info:
                print(f"     Range: {param_info['min']} - {param_info['max']}")
            print()
        
        # Demonstrate parameter updates
        print("2. Adjusting parameters...")
        
        # Adjust CAN bus latency
        print("   Updating latency_ms to 3.5ms...")
        success = self.update_parameter("latency_ms", 3.5)
        print(f"   Success: {success}")
        
        # Adjust refresh rate
        print("   Updating refresh_rate to 150ms...")
        success = self.update_parameter("refresh_rate", 150)
        print(f"   Success: {success}")
        
        # Show updated configuration
        print("\\n3. Updated Configuration:")
        updated_config = self.get_current_config()
        print(f"   Latency: {updated_config.get('latency_ms', 'N/A')}ms")
        
    def run_full_demo(self):
        """Run the complete configuration management demonstration."""
        print("MKS Servo Simulator - Configuration Management Demo")
        print("=" * 55)
        
        # Check API availability
        if not self.check_api_availability():
            print("❌ Error: Simulator API not available!")
            print("   Please start the simulator with: mks-servo-simulator --debug-api --dashboard")
            return
        
        print("✅ Simulator API is available")
        
        try:
            # Run demonstrations
            self.demo_configuration_profiles()
            self.demo_motor_templates()
            self.demo_live_parameters()
            
            print("\\n=== Demo Complete ===")
            print("✅ All configuration management features demonstrated successfully!")
            print("\\nNext steps:")
            print("- Try the interactive dashboard with: mks-servo-simulator --dashboard")
            print("- Explore keyboard controls: 'p' for profiles, 'l' for live parameters, 'm' for templates")
            print("- Check the API documentation at: http://localhost:8765/docs")
            
        except requests.RequestException as e:
            print(f"❌ API Error: {e}")
        except Exception as e:
            print(f"❌ Unexpected Error: {e}")


def main():
    """Main function to run the configuration demo."""
    demo = ConfigurationDemo()
    demo.run_full_demo()


if __name__ == "__main__":
    main()