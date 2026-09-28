"""Machine-readable release and input identity for engineering reports."""

from __future__ import annotations

import hashlib
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path
import platform


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def report_provenance(application_file: Path, simulation_file: Path) -> dict:
    package_root = Path(__file__).resolve().parents[1]
    version_file = package_root / "VERSION"
    dependencies = {}
    for name in ("casadi", "numpy", "matplotlib", "opcua"):
        try:
            dependencies[name] = version(name)
        except PackageNotFoundError:
            dependencies[name] = "not installed"
    files = {
        "application": file_sha256(application_file),
        "simulation": file_sha256(simulation_file),
    }
    try:
        import tomllib
        raw = tomllib.loads(application_file.read_text(encoding="utf-8"))
        model_name = raw.get("model", {}).get("file")
        if model_name:
            model_file = application_file.parent / model_name
            files["model"] = file_sha256(model_file)
    except (OSError, ValueError, TypeError):
        pass
    return {
        "package_version": (
            version_file.read_text(encoding="utf-8").strip()
            if version_file.is_file() else "unversioned-runtime-copy"
        ),
        "python_version": platform.python_version(),
        "dependencies": dependencies,
        "sha256": files,
    }
