"""Named point library used to resolve inspection targets by point id.

Maps differ per building, so a point may carry the map it was surveyed on. When
several maps define the same id the task's own `map` field disambiguates.
"""

import os
from typing import Dict, List, Optional

import yaml

from web_console.inspection.schema import TaskSchemaError

SCHEMA_VERSION = 1


def _coerce_point(raw, index: int, fallback_id: str = "") -> Dict:
    where = "points[{}]".format(index)
    if not isinstance(raw, dict):
        raise TaskSchemaError("{} must be an object".format(where))
    point_id = str(raw.get("id") or fallback_id or "").strip()
    if not point_id:
        raise TaskSchemaError("{} needs an id".format(where))
    try:
        x = float(raw["x"])
        y = float(raw["y"])
        yaw = float(raw.get("yaw", 0.0))
    except (KeyError, TypeError, ValueError):
        raise TaskSchemaError("{} ({}) needs numeric x/y (and optional yaw)".format(where, point_id))
    return {
        "id": point_id,
        "name": str(raw.get("name") or point_id),
        "map": str(raw.get("map") or "").strip(),
        "floor": str(raw.get("floor") or "").strip(),
        "x": x,
        "y": y,
        "yaw": yaw,
    }


class PointLibrary:
    """In-memory point index; empty when no points file is configured."""

    def __init__(self, points: Optional[List[Dict]] = None, path: str = ""):
        self.path = path
        self._by_id: Dict[str, List[Dict]] = {}
        for point in points or []:
            self._by_id.setdefault(point["id"], []).append(point)

    @classmethod
    def load(cls, path: str) -> "PointLibrary":
        expanded = os.path.expanduser(os.path.expandvars(path or ""))
        if not expanded or not os.path.isfile(expanded):
            return cls(path=expanded)
        try:
            with open(expanded, "r", encoding="utf-8") as handle:
                document = yaml.safe_load(handle) or {}
        except (OSError, yaml.YAMLError) as exc:
            raise TaskSchemaError("cannot read point library {}: {}".format(expanded, exc))

        if not isinstance(document, dict):
            raise TaskSchemaError("point library {} must be a mapping".format(expanded))
        version = document.get("schema_version", SCHEMA_VERSION)
        if version not in (SCHEMA_VERSION, str(SCHEMA_VERSION)):
            raise TaskSchemaError("unsupported point library schema_version {} in {}".format(
                version, expanded))

        raw_points = document.get("points") or []
        points = []
        if isinstance(raw_points, dict):
            for index, (point_id, raw) in enumerate(raw_points.items()):
                entry = dict(raw or {})
                entry.setdefault("id", point_id)
                points.append(_coerce_point(entry, index, point_id))
        elif isinstance(raw_points, list):
            points = [_coerce_point(raw, index) for index, raw in enumerate(raw_points)]
        else:
            raise TaskSchemaError("point library 'points' must be a list or mapping")
        return cls(points=points, path=expanded)

    def resolve(self, point_id: str, map_name: str = "") -> Dict:
        candidates = self._by_id.get(point_id) or []
        if not candidates:
            known = ", ".join(sorted(self._by_id)) or "none"
            raise TaskSchemaError("unknown point_id '{}' (known: {})".format(point_id, known))
        if len(candidates) > 1:
            if map_name:
                scoped = [point for point in candidates if point["map"] in (map_name, "")]
                candidates = scoped or candidates
            if len(candidates) > 1:
                raise TaskSchemaError(
                    "point_id '{}' is defined on several maps ({}); set the task 'map' field".format(
                        point_id, ", ".join(sorted({point["map"] or "<any>" for point in candidates}))))
        return dict(candidates[0])

    def list(self) -> List[Dict]:
        return [dict(point) for point in sorted(
            (point for points in self._by_id.values() for point in points),
            key=lambda point: (point["map"], point["id"]))]

    def status(self) -> Dict:
        return {
            "path": self.path,
            "loaded": bool(self.path and os.path.isfile(self.path)),
            "count": sum(len(points) for points in self._by_id.values()),
        }
