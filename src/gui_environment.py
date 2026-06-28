from __future__ import annotations

import importlib
import os
import ctypes
import sys
from pathlib import Path
from typing import Iterable


def _import_or_message(module_name: str, label: str | None = None) -> str | None:
    try:
        importlib.import_module(module_name)
    except Exception as exc:  # pragma: no cover - environment dependent
        name = label or module_name
        return f"{name} import failed: {exc}"
    return None


def _has_active_macos_console_session() -> bool:
    """
    Return True when the current process appears to be attached to a macOS
    console session.

    Qt's Cocoa platform plugin aborts early when launched from a detached shell
    without a live desktop session, so we probe the CoreGraphics session state
    before constructing QApplication.
    """
    if sys.platform != "darwin":
        return True

    try:
        coregraphics = ctypes.CDLL("/System/Library/Frameworks/CoreGraphics.framework/CoreGraphics")
    except OSError:
        return True

    coregraphics.CGSessionCopyCurrentDictionary.restype = ctypes.c_void_p
    session = coregraphics.CGSessionCopyCurrentDictionary()
    return bool(session)


def qt_plugin_paths() -> tuple[Path, Path]:
    from PySide6 import QtCore

    plugin_root = Path(QtCore.QLibraryInfo.path(QtCore.QLibraryInfo.LibraryPath.PluginsPath))
    platform_root = plugin_root / "platforms"
    return plugin_root, platform_root


def configure_qt_plugin_paths() -> tuple[Path, Path]:
    plugin_root, platform_root = qt_plugin_paths()
    os.environ["QT_PLUGIN_PATH"] = str(plugin_root)
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = str(platform_root)

    from PySide6 import QtCore

    QtCore.QCoreApplication.setLibraryPaths([str(plugin_root)])
    return plugin_root, platform_root


def collect_runtime_issues() -> list[str]:
    issues: list[str] = []

    if sys.platform == "darwin" and not _has_active_macos_console_session():
        issues.append(
            "No active macOS desktop session was detected. Launch the app from an interactive "
            "desktop terminal, not a detached shell."
        )

    pyside6_ok = True
    for module_name, label in (
        ("numpy", "numpy"),
        ("PySide6", "PySide6"),
        ("pyqtgraph", "pyqtgraph"),
        ("OpenGL", "PyOpenGL"),
        ("OpenGL.GL", "OpenGL.GL"),
        ("pyqtgraph.opengl", "pyqtgraph.opengl"),
    ):
        message = _import_or_message(module_name, label)
        if message:
            issues.append(message)
            if module_name == "PySide6":
                pyside6_ok = False

    if pyside6_ok:
        try:
            plugin_root, platform_root = qt_plugin_paths()
        except Exception as exc:  # pragma: no cover - environment dependent
            issues.append(f"Qt plugin paths could not be resolved from PySide6: {exc}")
        else:
            if not plugin_root.is_dir():
                issues.append(f"Qt plugin root does not exist: {plugin_root}")
            if not platform_root.is_dir():
                issues.append(f"Qt platform plugin directory does not exist: {platform_root}")
            cocoa = platform_root / "libqcocoa.dylib"
            if not cocoa.is_file():
                issues.append(f"Qt Cocoa platform plugin is missing: {cocoa}")

    return issues


def format_runtime_issues(issues: Iterable[str]) -> str:
    issues = list(issues)
    if not issues:
        return "Environment check passed."
    return "Environment check failed:\n" + "\n".join(f"- {issue}" for issue in issues)
