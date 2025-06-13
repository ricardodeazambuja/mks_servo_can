#!/usr/bin/env python3
"""
Interactive Dashboard Tutorial for MKS Servo Simulator

This script provides a guided tutorial for using the Rich interactive dashboard.
It demonstrates all the key features and keyboard controls available.

Usage:
1. Run this script to see the tutorial
2. Then start the simulator with dashboard: mks-servo-simulator --dashboard --num-motors 3
3. Follow the on-screen instructions in the dashboard

Requirements:
- mks-servo-simulator installed with Rich support
"""

import time
import subprocess
import sys
from typing import List, Tuple


class DashboardTutorial:
    """Interactive tutorial for the Rich dashboard features."""
    
    def __init__(self):
        self.tutorial_steps = [
            ("Getting Started", [
                "Welcome to the MKS Servo Simulator Interactive Dashboard!",
                "This tutorial will guide you through all the features.",
                "",
                "To start the dashboard, run:",
                "  mks-servo-simulator --dashboard --num-motors 3",
                "",
                "The dashboard will show real-time motor status and system information."
            ]),
            
            ("Basic Navigation", [
                "Once the dashboard is running, you can navigate with these keys:",
                "",
                "🔼 ↑ (Up Arrow)    - Select previous motor",
                "🔽 ↓ (Down Arrow)  - Select next motor", 
                "⏎ Enter          - Show detailed motor information",
                "❓ h              - Show/hide help panel",
                "⏸️ Space          - Pause/resume dashboard updates",
                "❌ q              - Quit the simulator",
                "",
                "Try selecting different motors with the arrow keys!"
            ]),
            
            ("Motor Controls", [
                "You can directly control motors from the dashboard:",
                "",
                "🔘 e  - Enable/disable the selected motor",
                "🛑 s  - Stop the selected motor immediately", 
                "0️⃣ z  - Zero the position of selected motor",
                "",
                "Motor status will update in real-time as you make changes.",
                "Look for color-coded status indicators:",
                "  🟢 Green = Enabled/Good",
                "  🔴 Red = Disabled/Error",
                "  🟡 Yellow = Warning/Transition"
            ]),
            
            ("Command Injection", [
                "Advanced users can inject commands directly:",
                "",
                "🔧 i  - Interactive command injection mode",
                "📝 c  - Text-based command mode",
                "📋 t  - Show available command templates",
                "🧪 x  - Run test scenarios",
                "",
                "Command injection allows you to:",
                "- Send raw hex commands to motors",
                "- Use pre-defined templates (enable, disable, move, etc.)",
                "- Run automated test sequences",
                "- Validate command responses"
            ]),
            
            ("Performance Monitoring", [
                "The dashboard includes real-time performance monitoring:",
                "",
                "📊 Performance Panel shows:",
                "  - Command latency (average, P95, P99)",
                "  - Throughput (commands per second)",
                "  - Memory usage and connection health",
                "  - Error rates and success statistics",
                "",
                "⚡ Performance Controls:",
                "  + (Plus)   - Increase refresh rate (faster updates)",
                "  - (Minus)  - Decrease refresh rate (slower updates)",
                "",
                "Monitor these metrics to ensure optimal performance!"
            ]),
            
            ("Configuration Management", [
                "NEW: Advanced configuration management features:",
                "",
                "📁 p  - Configuration profiles menu",
                "⚙️ l  - Live parameter adjustment",
                "🏷️ m  - Motor templates menu", 
                "💾 k  - Save current configuration",
                "",
                "Configuration features:",
                "- Save/load different simulator setups",
                "- Apply motor templates (SERVO42D, SERVO57D, etc.)",
                "- Adjust parameters live without restart",
                "- Manage multiple configuration profiles"
            ]),
            
            ("Command Mode Examples", [
                "When in command mode (press 'c'), try these commands:",
                "",
                "Basic Commands:",
                "  motor 1           - Select motor 1",
                "  enable            - Enable selected motor",
                "  disable           - Disable selected motor",
                "  stop              - Stop selected motor",
                "  zero              - Zero motor position",
                "",
                "Configuration Commands:",
                "  param latency_ms 5.0        - Set CAN latency to 5ms",
                "  template 1 servo42d         - Apply SERVO42D template to motor 1",
                "  profile my_setup            - Load 'my_setup' profile",
                "  save test_config            - Save current config as 'test_config'"
            ]),
            
            ("HTTP API Integration", [
                "The dashboard works alongside the HTTP API:",
                "",
                "Start with API enabled:",
                "  mks-servo-simulator --dashboard --debug-api",
                "",
                "API Endpoints (http://localhost:8765/):",
                "  /docs                    - Interactive API documentation",
                "  /status                  - Get simulator status",
                "  /config                  - Configuration management",
                "  /performance             - Performance metrics",
                "  /inject                  - Command injection",
                "",
                "Use the API for programmatic control while monitoring",
                "the dashboard for real-time visual feedback!"
            ]),
            
            ("Tips and Best Practices", [
                "💡 Pro Tips for using the dashboard effectively:",
                "",
                "1. Start with basic navigation (arrow keys, enter)",
                "2. Use 'h' frequently to see available commands",
                "3. Monitor the performance panel for system health",
                "4. Save useful configurations as profiles",
                "5. Use command injection for testing and debugging",
                "",
                "Troubleshooting:",
                "- If dashboard is unresponsive, press 'space' to unpause",
                "- Check the log panel for error messages",
                "- Use 'q' to quit gracefully if needed",
                "- Restart simulator if performance degrades"
            ]),
            
            ("Next Steps", [
                "🎉 Congratulations! You've completed the dashboard tutorial.",
                "",
                "Ready to explore more? Try these:",
                "",
                "1. Advanced Examples:",
                "   - Run configuration_management_demo.py",
                "   - Try the HTTP API examples",
                "   - Explore motor control with the library",
                "",
                "2. Real-world Usage:",
                "   - Connect to physical hardware",
                "   - Build custom control applications",
                "   - Integrate with your automation systems",
                "",
                "3. Documentation:",
                "   - Check docs/ for detailed guides",
                "   - Read the API reference",
                "   - Explore example scripts in examples/",
                "",
                "Happy simulating! 🚀"
            ])
        ]
    
    def print_step(self, title: str, content: List[str]):
        """Print a tutorial step with formatting."""
        print("\\n" + "=" * 60)
        print(f"📚 {title}")
        print("=" * 60)
        
        for line in content:
            if line.startswith("  "):
                # Indented content
                print(f"    {line[2:]}")
            elif line == "":
                # Empty line
                print()
            else:
                # Regular content
                print(f"  {line}")
        
        print("\\n" + "-" * 60)
    
    def run_tutorial(self):
        """Run the complete dashboard tutorial."""
        print("🎯 MKS Servo Simulator - Interactive Dashboard Tutorial")
        print("=" * 70)
        print("  This tutorial will teach you how to use the Rich interactive dashboard")
        print("  with all its advanced features and keyboard controls.")
        print("=" * 70)
        
        try:
            for i, (title, content) in enumerate(self.tutorial_steps, 1):
                self.print_step(f"Step {i}: {title}", content)
                
                if i < len(self.tutorial_steps):
                    input("Press Enter to continue to the next step...")
                
            print("\\n" + "🎉" * 20)
            print("Tutorial Complete! Start the dashboard and begin exploring.")
            print("🎉" * 20)
            
        except KeyboardInterrupt:
            print("\\n\\n⚠️ Tutorial interrupted by user.")
            print("You can restart the tutorial anytime by running this script again.")
    
    def check_dependencies(self) -> bool:
        """Check if required dependencies are available."""
        try:
            # Check if simulator is installed
            result = subprocess.run(['mks-servo-simulator', '--help'], 
                                  capture_output=True, text=True, timeout=5)
            return result.returncode == 0
        except (subprocess.TimeoutExpired, FileNotFoundError):
            return False
    
    def show_quick_start(self):
        """Show quick start commands."""
        print("\\n🚀 Quick Start Commands:")
        print("=" * 40)
        print("  # Basic dashboard")
        print("  mks-servo-simulator --dashboard --num-motors 2")
        print()
        print("  # Dashboard with API")
        print("  mks-servo-simulator --dashboard --debug-api --num-motors 3")
        print()
        print("  # Full-featured setup")
        print("  mks-servo-simulator --dashboard --debug-api --json-output --num-motors 2")
        print("=" * 40)


def main():
    """Main function to run the dashboard tutorial."""
    tutorial = DashboardTutorial()
    
    print("Checking dependencies...")
    if not tutorial.check_dependencies():
        print("❌ Error: mks-servo-simulator not found!")
        print("Please install the simulator first:")
        print("  cd mks_servo_simulator && pip install -e .")
        return
    
    print("✅ Dependencies OK")
    
    # Ask user what they want to do
    print("\\nWhat would you like to do?")
    print("1. Run complete tutorial")
    print("2. Show quick start commands only")
    print("3. Exit")
    
    choice = input("\\nEnter your choice (1-3): ").strip()
    
    if choice == "1":
        tutorial.run_tutorial()
        tutorial.show_quick_start()
    elif choice == "2":
        tutorial.show_quick_start()
    elif choice == "3":
        print("Goodbye!")
    else:
        print("Invalid choice. Please run the script again.")


if __name__ == "__main__":
    main()