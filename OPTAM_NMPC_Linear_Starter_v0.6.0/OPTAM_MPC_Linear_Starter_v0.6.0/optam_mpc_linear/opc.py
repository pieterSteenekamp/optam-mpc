"""Local supervised OPC UA twin protocol; not a live-plant tag driver."""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import tomllib
from urllib.parse import urlparse

from .definition import ConfigurationError

PROTOCOL = "optam-linear-twin/1"
NODE_KEYS = ("sample_node", "command_node", "status_node", "telemetry_node", "stop_node")


def load_opc(path):
    with Path(path).open("rb") as stream:
        raw = tomllib.load(stream)
    cfg = raw.get("opc", {})
    errors = []
    if not isinstance(cfg, dict):
        raise ConfigurationError(["Use [opc] table."])
    endpoint = urlparse(str(cfg.get("endpoint", "")))
    if endpoint.scheme != "opc.tcp" or endpoint.hostname != "127.0.0.1":
        errors.append("OPC demo endpoint must use opc.tcp://127.0.0.1:PORT/path (loopback only).")
    try:
        if not endpoint.port:
            errors.append("OPC endpoint requires an explicit port.")
    except ValueError:
        errors.append("Invalid OPC port.")
    if endpoint.username or endpoint.password:
        errors.append("Do not put credentials in the endpoint.")
    for key in ("namespace_uri", *NODE_KEYS):
        if not isinstance(cfg.get(key), str) or not cfg[key].strip():
            errors.append(f"[opc].{key}: non-empty text required.")
    if len({cfg.get(k) for k in NODE_KEYS}) != len(NODE_KEYS):
        errors.append("OPC node identifiers must be distinct.")
    for key in ("poll_seconds", "request_timeout_seconds", "startup_timeout_seconds",
                "command_timeout_seconds", "stale_timeout_seconds", "speed"):
        value = cfg.get(key)
        if type(value) not in (float, int) or not math.isfinite(value) or value <= 0:
            errors.append(f"[opc].{key}: positive finite number required.")
    if not errors:
        if cfg["stale_timeout_seconds"] <= cfg["command_timeout_seconds"]:
            errors.append("stale_timeout_seconds must exceed command_timeout_seconds.")
        if cfg["command_timeout_seconds"] <= cfg["poll_seconds"] * 2:
            errors.append("command_timeout_seconds must exceed twice poll_seconds.")
    if errors:
        raise ConfigurationError(errors)
    return cfg


def fingerprint(definition):
    return hashlib.sha256(json.dumps(definition.raw, sort_keys=True,
                                     separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def encode(value):
    return json.dumps(value, separators=(",", ":"), allow_nan=False)


def decode(value):
    if not isinstance(value, str) or len(value) > 1_000_000:
        raise ValueError("Invalid OPC frame type or size.")
    value = json.loads(value, parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
    if not isinstance(value, dict):
        raise ValueError("OPC frame must be an object.")
    return value


def numbers(values, keys, label):
    if not isinstance(values, dict) or set(values) != set(keys):
        raise ValueError(f"{label}: signal IDs differ from configured IDs.")
    if not all(type(x) in (int, float) and math.isfinite(x) for x in values.values()):
        raise ValueError(f"{label}: finite numeric values required.")
    return values


def validate_sample(frame, definition, expected_sample=None, run_id=None):
    if frame.get("protocol") != PROTOCOL or frame.get("application_hash") != fingerprint(definition):
        raise ValueError("Protocol/application mismatch: use identical application.toml on both sides.")
    if not isinstance(frame.get("run_id"), str) or not frame["run_id"]:
        raise ValueError("Missing run identifier.")
    if run_id is not None and frame["run_id"] != run_id:
        raise ValueError("Twin restarted: restart controller explicitly.")
    if type(frame.get("sample")) is not int or frame["sample"] < 0:
        raise ValueError("Invalid sample sequence.")
    if expected_sample is not None and frame["sample"] != expected_sample:
        raise ValueError("Missing or out-of-order sample.")
    if frame.get("sample_time_seconds") != definition.sample_time_seconds:
        raise ValueError("Sample time mismatch.")
    numbers(frame.get("cvs"), [x.id for x in definition.primary_cvs], "CVs")
    numbers(frame.get("dvs"), [x.id for x in definition.sources if x.kind == "dv"], "DVs")
    numbers(frame.get("mvs"), [x.id for x in definition.sources if x.kind == "mv"], "MVs")
    if "targets" in frame:
        numbers(frame["targets"], [x.id for x in definition.primary_cvs if x.target is not None], "Targets")


def validate_command(command, frame, definition, owner=None):
    if (type(command.get("sample")) is not int or command.get("run_id") != frame["run_id"]
            or command.get("sample") != frame["sample"]):
        raise ValueError("Stale or out-of-order command.")
    if command.get("application_hash") != frame["application_hash"]:
        raise ValueError("Command application mismatch.")
    client_id = command.get("controller_id")
    if not isinstance(client_id, str) or not client_id:
        raise ValueError("Missing controller identifier.")
    if owner is not None and owner != client_id:
        raise ValueError("Second controller detected. Only one controller is allowed.")
    if command.get("status") != "ok":
        raise ValueError("Controller fault: " + str(command.get("message", "unspecified")))
    values = numbers(command.get("mvs"), frame["mvs"], "MV command")
    solve_time = command.get("solver_seconds")
    if type(solve_time) not in (int, float) or not math.isfinite(solve_time) or solve_time < 0:
        raise ValueError("Invalid solver time.")
    for cfg in definition.raw["mvs"]:
        value, previous = values[cfg["id"]], frame["mvs"][cfg["id"]]
        eps = 1e-6
        if not cfg["lower"] - eps <= value <= cfg["upper"] + eps:
            raise ValueError(f"MV {cfg['id']}: position limit exceeded.")
        if not -cfg["maximum_decrease_per_sample"] - eps <= value - previous <= cfg["maximum_increase_per_sample"] + eps:
            raise ValueError(f"MV {cfg['id']}: move limit exceeded.")
    return values


async def read_frame(node, maximum_age=None):
    data = await node.read_data_value()
    data.StatusCode.check()
    if maximum_age is not None:
        stamp = data.SourceTimestamp
        if stamp is None:
            raise ValueError("Missing OPC source timestamp.")
        if stamp.tzinfo is None:
            stamp = stamp.replace(tzinfo=timezone.utc)
        age = (datetime.now(timezone.utc) - stamp).total_seconds()
        if age > maximum_age or age < -2:
            raise ValueError("Stale OPC sample or invalid source timestamp.")
    return decode(data.Value.Value)


async def client_nodes(client, cfg):
    from asyncua import ua
    ns = await client.get_namespace_index(cfg["namespace_uri"])
    return {key: client.get_node(ua.NodeId(cfg[key], ns)) for key in NODE_KEYS}
