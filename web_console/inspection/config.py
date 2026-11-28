"""Inspection module configuration: defaults, YAML file, environment, CLI."""

import os
from dataclasses import dataclass, replace
from typing import Dict, Optional

import yaml

from web_console.inspection.schema import TaskSchemaError

_TRUE = ("1", "true", "yes", "on")
_FALSE = ("0", "false", "no", "off")

_FILE_KEYS = {
    "enabled", "storage_dir", "points_file", "token", "callbacks_enabled",
    "default_callback_url", "max_queue", "max_tasks_kept", "report_progress",
    "callback_hosts",
}


def _as_bool(value, field: str) -> bool:
    if isinstance(value, bool):
        return value
    text = str(value or "").strip().lower()
    if text in _TRUE:
        return True
    if text in _FALSE:
        return False
    raise TaskSchemaError("{} must be a boolean".format(field))


@dataclass(frozen=True)
class InspectionConfig:
    """Defaults match the CLI: the module is on unless a flag/file/env disables it."""

    enabled: bool = True
    storage_dir: str = "~/ons2_inspection"
    points_file: str = ""
    token: str = ""
    callbacks_enabled: bool = True
    default_callback_url: str = ""
    max_queue: int = 32
    max_tasks_kept: int = 200
    report_progress: bool = True
    callback_hosts: tuple = ()

    @classmethod
    def from_mapping(cls, raw: Dict) -> "InspectionConfig":
        unknown = sorted(set(raw) - _FILE_KEYS)
        if unknown:
            raise TaskSchemaError("inspection config has unknown key(s): {}".format(", ".join(unknown)))
        values = {}
        if "enabled" in raw:
            values["enabled"] = _as_bool(raw["enabled"], "enabled")
        if "callbacks_enabled" in raw:
            values["callbacks_enabled"] = _as_bool(raw["callbacks_enabled"], "callbacks_enabled")
        if "report_progress" in raw:
            values["report_progress"] = _as_bool(raw["report_progress"], "report_progress")
        for key in ("storage_dir", "points_file", "token", "default_callback_url"):
            if key in raw:
                values[key] = str(raw[key] or "").strip()
        for key, minimum, maximum in (("max_queue", 1, 500), ("max_tasks_kept", 10, 5000)):
            if key in raw:
                try:
                    number = int(raw[key])
                except (TypeError, ValueError):
                    raise TaskSchemaError("{} must be an integer".format(key))
                if number < minimum or number > maximum:
                    raise TaskSchemaError("{} must be between {} and {}".format(key, minimum, maximum))
                values[key] = number
        if "callback_hosts" in raw:
            hosts = raw["callback_hosts"] or []
            if isinstance(hosts, str):
                hosts = [hosts]
            if not isinstance(hosts, list):
                raise TaskSchemaError("callback_hosts must be a list of host names")
            values["callback_hosts"] = tuple(str(host).strip() for host in hosts if str(host).strip())
        return cls(**values)

    @classmethod
    def load_file(cls, path: str) -> "InspectionConfig":
        expanded = os.path.expanduser(os.path.expandvars(path or ""))
        if not expanded:
            return cls()
        if not os.path.isfile(expanded):
            raise TaskSchemaError("inspection config not found: {}".format(expanded))
        try:
            with open(expanded, "r", encoding="utf-8") as handle:
                document = yaml.safe_load(handle) or {}
        except (OSError, yaml.YAMLError) as exc:
            raise TaskSchemaError("cannot read inspection config {}: {}".format(expanded, exc))
        if not isinstance(document, dict):
            raise TaskSchemaError("inspection config {} must be a mapping".format(expanded))
        section = document.get("inspection", document)
        if not isinstance(section, dict):
            raise TaskSchemaError("'inspection' section must be a mapping")
        return cls.from_mapping(section)

    def with_env(self, env: Optional[Dict[str, str]] = None) -> "InspectionConfig":
        env = os.environ if env is None else env
        values = {}
        if env.get("ONS2_INSPECTION_ENABLE"):
            values["enabled"] = _as_bool(env["ONS2_INSPECTION_ENABLE"], "ONS2_INSPECTION_ENABLE")
        if env.get("ONS2_INSPECTION_DIR"):
            values["storage_dir"] = env["ONS2_INSPECTION_DIR"]
        if env.get("ONS2_INSPECTION_POINTS"):
            values["points_file"] = env["ONS2_INSPECTION_POINTS"]
        if env.get("ONS2_INSPECTION_TOKEN"):
            values["token"] = env["ONS2_INSPECTION_TOKEN"]
        if env.get("ONS2_INSPECTION_CALLBACK"):
            values["default_callback_url"] = env["ONS2_INSPECTION_CALLBACK"]
        if env.get("ONS2_INSPECTION_CALLBACKS"):
            values["callbacks_enabled"] = _as_bool(env["ONS2_INSPECTION_CALLBACKS"], "ONS2_INSPECTION_CALLBACKS")
        return replace(self, **values)

    def with_cli(self, args) -> "InspectionConfig":
        """CLI flags win; argparse defaults are None so untouched flags keep file/env values."""
        values = {}
        for key in ("inspection_storage_dir", "inspection_points", "inspection_token",
                    "inspection_callback"):
            target = {"inspection_storage_dir": "storage_dir",
                      "inspection_points": "points_file",
                      "inspection_token": "token",
                      "inspection_callback": "default_callback_url"}[key]
            value = getattr(args, key, None)
            if value is not None:
                values[target] = str(value).strip()
        enable = getattr(args, "inspection", None)
        if enable is not None:
            values["enabled"] = bool(enable)
        callbacks = getattr(args, "inspection_callbacks", None)
        if callbacks is not None:
            values["callbacks_enabled"] = bool(callbacks)
        hosts = getattr(args, "inspection_callback_hosts", None)
        if hosts:
            values["callback_hosts"] = tuple(hosts)
        return replace(self, **values)

    @classmethod
    def resolve(cls, args=None) -> "InspectionConfig":
        config = cls()
        if args is not None and getattr(args, "inspection_config", None):
            config = cls.load_file(args.inspection_config)
        config = config.with_env()
        if args is not None:
            config = config.with_cli(args)
        return config

    def storage_path(self) -> str:
        return os.path.abspath(os.path.expanduser(os.path.expandvars(self.storage_dir)))

    def points_path(self) -> str:
        return os.path.abspath(os.path.expanduser(os.path.expandvars(self.points_file))) if self.points_file else ""
