"""Build deterministic user releases from one tested canonical source tree."""

from __future__ import annotations

import hashlib
from pathlib import Path
import shutil
import zipfile


ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / "build"
DIST = ROOT / "dist"
PRODUCTS = {
    "linear": ("OPTAM_MPC_Linear_Starter_v0.6.0", "0.6.0"),
    "nonlinear": ("OPTAM_MPC_Nonlinear_Starter_v0.2.0", "0.2.0"),
}
EXCLUDED_PARTS = {"__pycache__", ".venv", "results", "opc_results"}


def excluded(path: Path) -> bool:
    return bool(set(path.parts) & EXCLUDED_PARTS) or path.suffix == ".pyc"


def copy_product(kind: str, destination: Path) -> None:
    source = ROOT / "product_sources" / kind
    shutil.copytree(
        source,
        destination,
        ignore=shutil.ignore_patterns("optam_mpc", "optam_mpc_linear", "__pycache__", "*.pyc"),
    )
    shutil.copytree(ROOT / "src" / "optam_mpc", destination / "optam_mpc")
    shutil.copytree(ROOT / "src" / "optam_mpc_linear", destination / "optam_mpc_linear")


def write_manifest(root: Path) -> None:
    entries = []
    for path in sorted(root.rglob("*")):
        if path.is_file() and path.name != "MANIFEST.sha256" and not excluded(path.relative_to(root)):
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            entries.append(f"{digest}  {path.relative_to(root).as_posix()}")
    (root / "MANIFEST.sha256").write_text("\n".join(entries) + "\n", encoding="utf-8")


def deterministic_zip(root: Path, archive: Path) -> None:
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as stream:
        for path in sorted(root.rglob("*")):
            relative = path.relative_to(root.parent)
            if path.is_file() and not excluded(relative):
                info = zipfile.ZipInfo(relative.as_posix(), (2026, 1, 1, 0, 0, 0))
                info.compress_type = zipfile.ZIP_DEFLATED
                info.external_attr = 0o100644 << 16
                stream.writestr(info, path.read_bytes())


def main() -> None:
    shutil.rmtree(BUILD, ignore_errors=True)
    shutil.rmtree(DIST, ignore_errors=True)
    BUILD.mkdir()
    DIST.mkdir()
    for kind, (folder_name, version) in PRODUCTS.items():
        destination = BUILD / folder_name
        copy_product(kind, destination)
        (destination / "VERSION").write_text(version + "\n", encoding="utf-8")
        write_manifest(destination)
        deterministic_zip(destination, DIST / f"{folder_name}.zip")
    source_name = "OPTAM_MPC_Canonical_Source_v0.6.0"
    source_destination = BUILD / source_name
    source_destination.mkdir()
    for filename in ("README.md", "REVIEW_RESOLUTION.md", "VERSION"):
        shutil.copy2(ROOT / filename, source_destination / filename)
    for dirname in ("src", "tests", "release_builder"):
        shutil.copytree(ROOT / dirname, source_destination / dirname,
                        ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    product_destination = source_destination / "product_sources"
    product_destination.mkdir()
    for kind in PRODUCTS:
        shutil.copytree(
            ROOT / "product_sources" / kind,
            product_destination / kind,
            ignore=shutil.ignore_patterns("optam_mpc", "optam_mpc_linear", "__pycache__", "*.pyc"),
        )
    write_manifest(source_destination)
    deterministic_zip(source_destination, DIST / f"{source_name}.zip")
    print(f"Built {len(PRODUCTS) + 1} releases in {DIST}")


if __name__ == "__main__":
    main()
