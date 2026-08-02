"""Validated hub-to-display estimate contract."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Mapping


class EstimateValidationError(ValueError):
    """Raised when an estimate cannot safely be rendered."""


def _number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise EstimateValidationError(f"{field} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise EstimateValidationError(f"{field} must be finite")
    return result


def _probability(value: Any, field: str) -> float:
    result = _number(value, field)
    if not 0.0 <= result <= 1.0:
        raise EstimateValidationError(f"{field} must be between 0 and 1")
    return result


def _pair(value: Any, field: str) -> tuple[float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise EstimateValidationError(f"{field} must contain two numbers")
    return (_number(value[0], f"{field}[0]"), _number(value[1], f"{field}[1]"))


def _node_ids(value: Any, field: str) -> tuple[int, ...]:
    if not isinstance(value, (list, tuple)):
        raise EstimateValidationError(f"{field} must be an array")
    result: list[int] = []
    for index, node_id in enumerate(value):
        if isinstance(node_id, bool) or not isinstance(node_id, int) or node_id < 0:
            raise EstimateValidationError(f"{field}[{index}] must be a non-negative integer")
        result.append(node_id)
    if len(result) != len(set(result)):
        raise EstimateValidationError(f"{field} must not contain duplicates")
    return tuple(result)


@dataclass(frozen=True, slots=True)
class Estimate:
    """One localization result in the fixed wall coordinate frame."""

    t_us: int
    site: str
    present: float
    position_m: tuple[float, float]
    covariance: tuple[tuple[float, float], tuple[float, float]]
    height_m: float | None
    nodes_online: tuple[int, ...]
    nodes_expected: tuple[int, ...]
    quality: float
    model: str

    @classmethod
    def from_mapping(cls, payload: Mapping[str, Any]) -> "Estimate":
        if not isinstance(payload, Mapping):
            raise EstimateValidationError("estimate must be an object")

        required = {
            "t_us",
            "site",
            "present",
            "position_m",
            "covariance",
            "height_m",
            "nodes_online",
            "nodes_expected",
            "quality",
            "model",
        }
        missing = required.difference(payload)
        if missing:
            raise EstimateValidationError(f"missing fields: {', '.join(sorted(missing))}")

        t_us = payload["t_us"]
        if isinstance(t_us, bool) or not isinstance(t_us, int) or t_us < 0:
            raise EstimateValidationError("t_us must be a non-negative integer")

        site = payload["site"]
        model = payload["model"]
        if not isinstance(site, str) or not site.strip():
            raise EstimateValidationError("site must be a non-empty string")
        if not isinstance(model, str) or not model.strip():
            raise EstimateValidationError("model must be a non-empty string")

        covariance_raw = payload["covariance"]
        if not isinstance(covariance_raw, (list, tuple)) or len(covariance_raw) != 2:
            raise EstimateValidationError("covariance must be a 2x2 matrix")
        row_0 = _pair(covariance_raw[0], "covariance[0]")
        row_1 = _pair(covariance_raw[1], "covariance[1]")
        covariance = (row_0, row_1)
        if row_0[0] < 0.0 or row_1[1] < 0.0:
            raise EstimateValidationError("covariance diagonal must be non-negative")
        if not math.isclose(row_0[1], row_1[0], rel_tol=1e-6, abs_tol=1e-9):
            raise EstimateValidationError("covariance must be symmetric")
        if row_0[0] * row_1[1] - row_0[1] * row_1[0] < -1e-9:
            raise EstimateValidationError("covariance must be positive semidefinite")

        height_raw = payload["height_m"]
        height_m = None if height_raw is None else _number(height_raw, "height_m")

        return cls(
            t_us=t_us,
            site=site,
            present=_probability(payload["present"], "present"),
            position_m=_pair(payload["position_m"], "position_m"),
            covariance=covariance,
            height_m=height_m,
            nodes_online=_node_ids(payload["nodes_online"], "nodes_online"),
            nodes_expected=_node_ids(payload["nodes_expected"], "nodes_expected"),
            quality=_probability(payload["quality"], "quality"),
            model=model,
        )

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-serializable representation."""
        return asdict(self)
