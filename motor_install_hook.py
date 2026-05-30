"""Install hook: fetch or verify prebuilt libmotor_arm / libmotor_chassis before build_py/develop."""
import hashlib
import json
import os
import platform
import re
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import Request, urlopen

_ROOT = Path(__file__).resolve().parent
_SHA256_MARKER = "AGXMOTOR_CTRL_LIBMOTOR_SHA256"
_VARIANTS = ("arm", "chassis")


def _read_version() -> str:
    text = (_ROOT / "agx_motor_ctrl" / "version.py").read_text(encoding="utf-8")
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("__version__"):
            return line.split("=", 1)[1].strip().strip('"').strip("'")
    raise RuntimeError("cannot read __version__")


def _host_platform_arch() -> Tuple[str, str]:
    pl = sys.platform
    if pl == "linux":
        p = "linux"
    elif pl == "darwin":
        p = "darwin"
    elif pl == "win32":
        p = "windows"
    else:
        p = pl.replace(" ", "_")
    m = platform.machine().lower()
    if m in ("aarch64", "arm64"):
        a = "aarch64"
    elif m in ("x86_64", "amd64"):
        a = "x86_64"
    elif m in ("i386", "i686"):
        a = "x86"
    else:
        a = m.replace(" ", "_")
    return p, a


def _dest_filename(variant: str) -> str:
    if sys.platform == "win32":
        return f"motor_{variant}.dll"
    if sys.platform == "darwin":
        return f"libmotor_{variant}.dylib"
    return f"libmotor_{variant}.so"


def _release_asset_name(variant: str, plat: str, arch: str) -> str:
    ext = {"win32": "dll", "darwin": "dylib"}.get(sys.platform, "so")
    return f"libmotor-{variant}-{plat}-{arch}.{ext}"


def _download_url(repo: str, version: str, asset_name: str) -> str:
    return f"https://github.com/{repo}/releases/download/v{version}/{asset_name}"


def _sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _fetch_release_body(repo: str, tag: str) -> str:
    api = f"https://api.github.com/repos/{repo}/releases/tags/{quote(tag)}"
    try:
        req = Request(api)
        req.add_header("Accept", "application/vnd.github+json")
        req.add_header("User-Agent", "agx_motor_ctrl-setup")
        with urlopen(req, timeout=30) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        body = data.get("body")
        return body if isinstance(body, str) else ""
    except Exception:
        return ""


def _parse_sha256_map(body: str) -> Dict[str, Any]:
    if not body:
        return {}
    m = re.search(
        rf"<!--\s*{_SHA256_MARKER}\s*-->\s*```(?:json)?\s*([\s\S]*?)\s*```",
        body,
    )
    if not m:
        return {}
    try:
        out = json.loads(m.group(1).strip())
        return out if isinstance(out, dict) else {}
    except ValueError:
        return {}


def _http_get_bytes(url: str) -> bytes:
    with urlopen(url, timeout=120) as resp:
        return resp.read()


def _raise_network_error(cause: Optional[BaseException] = None) -> None:
    raise RuntimeError(
        "Cannot install agx_motor_ctrl: failed to download libmotor binaries. "
        "Set AGXMOTOR_SKIP_DOWNLOAD=1 and place libmotor_arm / libmotor_chassis under "
        "agx_motor_ctrl/motor/, or set AGXMOTOR_ARM_LIB / AGXMOTOR_CHASSIS_LIB at runtime."
    ) from cause


def _raise_404(version: str, asset: str, plat: str, arch: str, cause: Optional[BaseException] = None) -> None:
    raise RuntimeError(
        f"Cannot install agx_motor_ctrl: asset '{asset}' not found in release v{version} ({plat}/{arch})."
    ) from cause


def _download_bytes(url: str, version: str, plat: str, arch: str, asset: str) -> bytes:
    try:
        data = _http_get_bytes(url)
    except HTTPError as e:
        if getattr(e, "code", None) == 404:
            _raise_404(version, asset, plat, arch, e)
        _raise_network_error(e)
    except (URLError, OSError) as e:
        _raise_network_error(e)
    if not data:
        raise RuntimeError(f"Downloaded {asset} is empty.")
    return data


def _ensure_variant(
    variant: str,
    plat: str,
    arch: str,
    version: str,
    repo: str,
    sha_map: Dict[str, Any],
    dest_dir: Path,
) -> None:
    dest_file = dest_dir / _dest_filename(variant)
    asset = _release_asset_name(variant, plat, arch)
    url = _download_url(repo, version, asset)
    has_local = dest_file.is_file() and dest_file.stat().st_size > 0

    expected = sha_map.get(asset)
    if isinstance(expected, str):
        expected = expected.strip().lower()
    else:
        expected = None

    if expected and has_local and _sha256_file(dest_file).lower() == expected:
        print(f"agx_motor_ctrl: local {dest_file.name} matches release metadata, skip download.")
        return

    if expected or not has_local:
        print(f"agx_motor_ctrl: downloading {asset}: {url}")
        data = _download_bytes(url, version, plat, arch, asset)
        dest_dir.mkdir(parents=True, exist_ok=True)
        dest_file.write_bytes(data)
        print(f"agx_motor_ctrl: wrote {dest_file} ({len(data)} bytes).")
        return

    print(f"agx_motor_ctrl: using local {dest_file.name}")


def download_motor_if_needed() -> None:
    if os.environ.get("AGXMOTOR_SKIP_DOWNLOAD", "").strip():
        print("agx_motor_ctrl: AGXMOTOR_SKIP_DOWNLOAD set, skip libmotor download.")
        return

    plat, arch = _host_platform_arch()
    dest_dir = _ROOT / "agx_motor_ctrl" / "motor"
    version = _read_version()
    repo = os.environ.get("AGXMOTOR_GITHUB_REPO", "kehuanjack/agx_motor_ctrl").strip()
    tag = f"v{version}"
    sha_map = _parse_sha256_map(_fetch_release_body(repo, tag))

    for variant in _VARIANTS:
        _ensure_variant(variant, plat, arch, version, repo, sha_map, dest_dir)


def get_cmdclass() -> Dict[str, Any]:
    from setuptools.command.build_py import build_py as _build_py
    from setuptools.command.develop import develop as _develop

    class _MotorBuildPy(_build_py):
        def run(self):
            download_motor_if_needed()
            super().run()

    class _MotorDevelop(_develop):
        def run(self):
            download_motor_if_needed()
            super().run()

    return {"build_py": _MotorBuildPy, "develop": _MotorDevelop}
