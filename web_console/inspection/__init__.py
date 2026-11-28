"""Inspection mission module: structured patrol tasks from an inspection backend.

The console hosts the module so the same code runs on ROS 1 (Go2, Dobot Lumo)
and ROS 2 (SW01) through the existing adapters, and the inspection backend only
needs the console's single HTTP port.
"""

from web_console.inspection.config import InspectionConfig
from web_console.inspection.manager import InspectionManager
from web_console.inspection.schema import TaskSchemaError

__all__ = ["InspectionConfig", "InspectionManager", "TaskSchemaError"]
