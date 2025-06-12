"""
Simulator interface modules for different output modes and debugging support.

This package provides various interfaces for the MKS servo simulator:
- LLM-friendly debugging interface
- Rich console dashboard
- HTTP API for programmatic access
- Structured output modes
"""

from .llm_debug_interface import LLMDebugInterface
from .http_debug_server import DebugHTTPServer
from .rich_dashboard import RichDashboard

__all__ = ["LLMDebugInterface", "DebugHTTPServer", "RichDashboard"]