#!/usr/bin/env python3
"""RTT-SearchAgent — ADB + ROS 2 Remote Control GUI.

Mission-control interface for multi-drone WiFi RTT experiments.
Tabs:
  1. Mission Control — discover ROS 2 devices, start/stop experiments
  2. Experiment Config — pre-flight ADB configuration
  3. NTRIP — RTK correction setup
  4. Ground Truth — register known AP positions
  5. Multi-OSA — launch centralised fusion node
  6. AP Estimates — live AP position estimates from phones & fusion
  7. Map — interactive OSM / satellite map with estimated AP positions
    8. Logs — pull/list experiment data
"""

import csv
import json
import os
import re
import shutil
import shlex
import signal
import subprocess
import sys
import tempfile
import threading
import time
from datetime import datetime
from pathlib import Path


_BOOTSTRAP_REPO_ROOT = Path(__file__).resolve().parents[1]
_BOOTSTRAP_VENV = _BOOTSTRAP_REPO_ROOT / ".venv"


def _preferred_gui_python() -> Path | None:
    """Return the workspace virtualenv interpreter when available."""
    venv_python = _BOOTSTRAP_VENV / "bin" / "python"
    return venv_python if venv_python.exists() else None


def _running_in_workspace_venv() -> bool:
    """Return True when the current interpreter already belongs to the repo venv."""
    try:
        return Path(sys.prefix).resolve() == _BOOTSTRAP_VENV.resolve()
    except Exception:
        return False


def _maybe_reexec_in_workspace_venv():
    """Re-exec the GUI under the repo virtualenv to avoid mixed Tk stacks."""
    if __name__ != "__main__":
        return
    if os.environ.get("RTT_GUI_VENV_BOOTSTRAPPED") == "1":
        return

    preferred = _preferred_gui_python()
    if preferred is None:
        return

    if _running_in_workspace_venv():
        return

    env = os.environ.copy()
    env["RTT_GUI_VENV_BOOTSTRAPPED"] = "1"
    env.setdefault("PYTHONNOUSERSITE", "1")
    os.execve(
        str(preferred),
        [str(preferred), str(Path(__file__).resolve()), *sys.argv[1:]],
        env,
    )


_maybe_reexec_in_workspace_venv()

import tkinter as tk
from tkinter import ttk, messagebox, filedialog

try:
    from tkintermapview import TkinterMapView
    _HAS_MAP = True
except ImportError:
    _HAS_MAP = False

PKG = "com.jbravo.osa_ftm"
DATA_DIR = f"/sdcard/Android/data/{PKG}/files"
CONFIG_FILE = Path.home() / ".config" / "rtt_searchagent.json"
MOBILE_RMW_IMPLEMENTATION = os.environ.get(
    "RTT_GUI_RMW_IMPLEMENTATION",
    os.environ.get("MOBILE_RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
)
_REPO_ROOT = _BOOTSTRAP_REPO_ROOT
USER_DATA_ROOT = Path(os.environ.get(
    "RTT_GUI_DATA_ROOT",
    Path.home() / ".local" / "share" / "rtt-searchagent",
)).expanduser()
GROUND_TRUTH_CSV = USER_DATA_ROOT / "reference" / "ground_truth.csv"
FUSION_LOGS_DIR = USER_DATA_ROOT / "fusion_logs"
MULTI_OSA_SCRIPT = _REPO_ROOT / "runtime" / "online" / "networked-OSA.py"


# ── Tooltip helper ──

class ToolTip:
    """Lightweight tooltip that appears on hover over a Tkinter widget."""

    def __init__(self, widget: tk.Widget, text: str, delay: int = 400):
        self.widget = widget
        self.text = text
        self.delay = delay
        self._tip_window: tk.Toplevel | None = None
        self._after_id: str | None = None
        widget.bind("<Enter>", self._schedule, add="+")
        widget.bind("<Leave>", self._cancel, add="+")
        widget.bind("<ButtonPress>", self._cancel, add="+")

    def _schedule(self, _event=None):
        self._cancel()
        self._after_id = self.widget.after(self.delay, self._show)

    def _cancel(self, _event=None):
        if self._after_id:
            self.widget.after_cancel(self._after_id)
            self._after_id = None
        self._hide()

    def _show(self):
        if self._tip_window:
            return
        x = self.widget.winfo_rootx() + 20
        y = self.widget.winfo_rooty() + self.widget.winfo_height() + 4
        tw = tk.Toplevel(self.widget)
        tw.wm_overrideredirect(True)
        tw.wm_geometry(f"+{x}+{y}")
        label = tk.Label(
            tw, text=self.text, justify="left", relief="solid", borderwidth=1,
            background="#ffffcc", foreground="#333333",
            font=("", 9), wraplength=350, padx=6, pady=4,
        )
        label.pack()
        self._tip_window = tw

    def _hide(self):
        if self._tip_window:
            self._tip_window.destroy()
            self._tip_window = None

# Known UAV labels for convenience (namespace → friendly label)
KNOWN_UAVS: dict[str, str] = {
    "pixel_7_pro_da383195": "UAV (dron)",
    "pixel_7_pro_c16cd3a6": "HUMAN (a pie)",
    #"pixel_7_pro_7da00a9c": "UAV 2 (negro)",
}


# ────────────────────────────────────────────────────────────────────────────
#  User-facing guides (shown at the top of every tab and in the About tab).
#  Written in English so first-time users can operate the whole pipeline
#  without prior knowledge of ROS 2, ADB or WiFi-RTT internals.
# ────────────────────────────────────────────────────────────────────────────

GUIDE_MISSION = (
    "What is this tab for?\n"
    "    Starting and stopping a mission across all drones at once.\n"
    "\n"
    "Typical workflow:\n"
    "    1. Power the drones (or the hand-held Android phones) and make sure\n"
    "       they are on the same ZeroTier VPN as this laptop.\n"
    "    2. Enter each phone's VPN IP and press 'Connect' under\n"
    "       'ADB over Network' (the very first time, cable a phone and press\n"
    "       'Enable TCP mode' once).\n"
    "    3. Press 'Discover UAVs (ROS 2)'. Every phone that is running the\n"
    "       RTT-SearchAgent Android app will appear in the list below.\n"
    "    4. Use the big green 'START ALL' to begin the RTT scan on every\n"
    "       drone, and the red 'STOP ALL' when you want to land.\n"
    "    5. The 'Pre-flight Checklist' boxes should all turn green (✅)\n"
    "       before you start the mission."
)

GUIDE_EXPERIMENT = (
    "What is this tab for?\n"
    "    Sending pre-flight configuration (altitude mode, RTT settings,\n"
    "    log label, etc.) to one or every drone.\n"
    "\n"
    "Typical workflow:\n"
    "    1. Fill in an experiment 'Label' — this is what will tag the logs.\n"
    "    2. Pick 'Altitude mode = auto' unless you know you need manual AGL.\n"
    "    3. Press 'Send to ALL' so every phone gets the same configuration.\n"
    "    4. Go back to 'Mission Control' and press START ALL."
)

GUIDE_NTRIP = (
    "What is this tab for?\n"
    "    Turning the cheap phone GNSS into centimetre-grade RTK by\n"
    "    streaming RTCM corrections from an NTRIP caster.\n"
    "\n"
    "Typical workflow:\n"
    "    1. Enter the caster details (host, port, mountpoint, user,\n"
    "       password). The free 'caster.centipede.fr' works in most of\n"
    "       Europe.\n"
    "    2. Press 'Send to ALL devices' to push the config, then\n"
    "       'NTRIP Connect'.\n"
    "    3. Watch 'Phone Status': Fix should read 'RTK FIX', HDOP < 1 m,\n"
    "       and 'RTCM bytes' should be growing.\n"
    "    4. If Fix is stuck on '3D', wait 30–60 s outdoors or check the\n"
    "       caster mountpoint."
)

GUIDE_GT = (
    "What is this tab for?\n"
    "    Registering the TRUE (surveyed) lat/lon/alt of each WiFi AP so\n"
    "    the GUI can compute live position errors and paint red 'GT pins'\n"
    "    on the map.\n"
    "\n"
    "Typical workflow:\n"
    "    1. For every victim AP, enter its BSSID (MAC of the radio you\n"
    "       want to find) and its surveyed coordinates.\n"
    "    2. Press 'Add / Update' — the row appears in the table.\n"
    "    3. Press 'Save CSV' to persist to tools/ground_truth.csv. That\n"
    "       file is auto-loaded on every GUI start."
)

GUIDE_MULTI_OSA = (
    "What is this tab for?\n"
    "    Launching the CENTRAL fusion node. Every drone computes its own\n"
    "    AP estimate; this node fuses them to produce one globally-best\n"
    "    estimate per victim.\n"
    "\n"
    "Typical workflow:\n"
    "    1. Make sure at least one drone is discovered (Mission tab).\n"
    "    2. Press 'Start Multi-OSA'. The log panel below should show the\n"
    "       node subscribing to each drone namespace.\n"
    "    3. Move to the 'Map' tab to watch the fused (green) estimates\n"
    "       converge while the drones explore.\n"
    "    4. Press 'Stop Multi-OSA' before closing the GUI."
)

GUIDE_ESTIMATES = (
    "What is this tab for?\n"
    "    A live table of every AP estimate published by the drones and by\n"
    "    the Multi-OSA fusion node, with instantaneous position error vs\n"
    "    ground truth.\n"
    "\n"
    "How to read it:\n"
    "    •  Source 'Multi-OSA' → fused estimate (best).\n"
    "    •  Source 'OSA: <phone>' → that drone's own estimate.\n"
    "    •  Err(m) column is green when < 15 m, orange when > 15 m.\n"
    "    •  Use the check-boxes to hide/show individual drones or the\n"
    "       fusion layer."
)

GUIDE_MAP = (
    "What is this tab for?\n"
    "    A live interactive map (OSM or satellite imagery) that shows:\n"
    "      •  🎯 Red pins  = Ground-Truth AP positions.\n"
    "      •  ⭐ Green pin = Multi-OSA fused estimate.\n"
    "      •  ⭐ Other colours = per-drone estimates and trails.\n"
    "\n"
    "Typical workflow:\n"
    "    1. Pick 'Satellite' for outdoor missions, 'OpenStreetMap' for\n"
    "       indoor / schematic views.\n"
    "    2. 'Max zoom' caps how far you can zoom in for the active layer.\n"
    "    3. Type 'lat, lon' in 'Go to' + Enter to jump anywhere.\n"
    "    4. 'Clear markers' removes only estimates — GT pins stay.\n"
    "    5. 'Clear trails' wipes the drone polylines.\n"
    "\n"
    "If the map is blank:\n"
    "    •  Check the laptop has internet access (tiles are streamed).\n"
    "    •  Install the map backend:  pip install tkintermapview"
)

GUIDE_LOGS = (
    "What is this tab for?\n"
    "    Pulling experiment logs (JSONL + rosbags) from every connected\n"
    "    phone into the laptop, and listing / deleting the on-phone\n"
    "    recordings.\n"
    "\n"
    "Typical workflow:\n"
    "    1. Select the ADB device (or '(any)' for all).\n"
    "    2. Press 'List remote logs' to see what's available.\n"
    "    3. Press 'Pull ALL' to copy them to the configurable local\n"
    "       'Destination' shown in this tab.\n"
    "    4. Use 'Clear remote logs' once you have a confirmed local copy."
)

GUIDE_ABOUT = (
    "RTT-SearchAgent — Mission Control\n"
    "A Tkinter GUI to command a swarm of Android-based WiFi-RTT drones\n"
    "that cooperatively locate WiFi victims after a disaster.\n"
    "\n"
    "Basic operating steps (end-to-end):\n"
    "    1. Power the drones and make sure every phone is on the\n"
    "       ZeroTier VPN shared with this laptop.\n"
    "    2. 'Mission Control' → ADB-over-network → Connect to every\n"
    "       phone's VPN IP.\n"
    "    3. 'Mission Control' → 'Discover UAVs (ROS 2)'.\n"
    "    4. 'Experiment Config' → set a label → 'Send to ALL'.\n"
    "    5. 'NTRIP' → fill caster details → 'Send to ALL' → 'Connect'.\n"
    "       Wait for RTK FIX on every phone.\n"
    "    6. 'Ground Truth' → enter / load the true AP coordinates.\n"
    "    7. 'Multi-OSA' → 'Start Multi-OSA' so estimates get fused.\n"
    "    8. 'Mission Control' → 'START ALL'.\n"
    "    9. Watch the mission live in 'AP Estimates' and 'Map'.\n"
    "   10. When the victims are found (errors < 10 m are typical),\n"
    "       'Mission Control' → 'STOP ALL'.\n"
    "   11. 'Multi-OSA' → 'Stop Multi-OSA'.\n"
    "   12. 'Logs' → 'Pull ALL' to archive the mission.\n"
    "\n"
    "Colour code across all tabs:\n"
    "    🎯 Red      = Ground truth (known AP position).\n"
    "    ⭐ Green   = Multi-OSA fused estimate.\n"
    "    ⭐ Other   = Per-drone (per-OSA) estimate / trail.\n"
    "\n"
    "Troubleshooting quick-reference:\n"
    "    •  'adb: device not found'  → check VPN, re-press 'Connect'.\n"
    "    •  'No UAVs discovered'     → phones are on a different ROS 2\n"
    "       domain or the Android app is not running.\n"
    "    •  Map tab is empty         → pip install tkintermapview and\n"
    "       make sure the laptop has internet for the map tiles.\n"
    "    •  'RTCM bytes = 0'         → wrong NTRIP mountpoint or\n"
    "       credentials.\n"
    "    •  Huge position errors     → no RTK fix, or AP BSSID mismatch\n"
    "       between Ground Truth tab and the real AP."
)


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Approximate 2D distance in metres between two lat/lon points."""
    import math
    R = 6_371_000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(dlon / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))



# ── Shell helpers ──

def _mobile_ros_env() -> dict[str, str]:
    """Environment for ROS 2 CLI calls targeting Android rcljava nodes."""
    env = os.environ.copy()
    if MOBILE_RMW_IMPLEMENTATION:
        env["RMW_IMPLEMENTATION"] = MOBILE_RMW_IMPLEMENTATION
        if MOBILE_RMW_IMPLEMENTATION.startswith("rmw_fastrtps"):
            env.pop("CYCLONEDDS_URI", None)
    return env


def run_cmd(args: list[str], timeout: int = 15, env: dict[str, str] | None = None) -> str:
    """Run a command and return combined stdout+stderr."""
    try:
        r = subprocess.run(args, capture_output=True, text=True, timeout=timeout, env=env)
        out = r.stdout.strip()
        if r.stderr.strip():
            out += "\n" + r.stderr.strip()
        if r.returncode != 0:
            command = " ".join(args)
            out = f"ERROR: command failed (rc={r.returncode}): {command}" + (
                f"\n{out}" if out else ""
            )
        return out
    except FileNotFoundError:
        return f"ERROR: '{args[0]}' not found in PATH"
    except subprocess.TimeoutExpired:
        return f"ERROR: command timed out ({timeout}s)"


def find_adb() -> str | None:
    """Find adb in PATH, ANDROID_HOME, or common Android SDK locations."""
    direct = shutil.which("adb")
    if direct:
        return direct

    candidates = []
    for env_name in ("ANDROID_HOME", "ANDROID_SDK_ROOT"):
        sdk_root = os.environ.get(env_name)
        if sdk_root:
            candidates.append(Path(sdk_root) / "platform-tools" / "adb")
    candidates.extend(
        [
            Path.home() / "Android" / "Sdk" / "platform-tools" / "adb",
            Path.home() / "Android" / "SDK" / "platform-tools" / "adb",
            Path("/opt/android-sdk/platform-tools/adb"),
            Path("/usr/lib/android-sdk/platform-tools/adb"),
        ]
    )
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate)
    return None


def adb(cmd: str, serial: str | None = None, timeout: int = 15) -> str:
    """Run an adb command, optionally targeting a device serial."""
    adb_bin = find_adb()
    if not adb_bin:
        return (
            "ERROR: 'adb' not found. Install Android platform-tools or set "
            "ANDROID_HOME."
        )
    parts = [adb_bin]
    if serial:
        parts += ["-s", serial]
    parts += shlex.split(cmd)
    return run_cmd(parts, timeout=timeout)


def _adb_failed(out: str) -> bool:
    """Return True when adb output indicates the command did not reach a device."""
    low = (out or "").lower()
    return any(
        needle in low
        for needle in (
            "error:", "failed", "more than one device", "no devices",
            "device not found", "device offline", "unauthorized",
        )
    )


def adb_start_app(serial: str | None = None) -> str:
    """Bring MainActivity to the foreground so dynamic ADB receivers exist."""
    return adb(f"shell am start -n {PKG}/.MainActivity", serial=serial, timeout=10)


def adb_force_stop_app(serial: str | None = None) -> str:
    """Stop the app process on one phone."""
    return adb(f"shell am force-stop {PKG}", serial=serial, timeout=10)


def adb_serial_usable(serial: str, timeout: int = 4) -> bool:
    """Return True only if a listed ADB serial actually executes shell commands."""
    probe = "__rtt_adb_ok__"
    out = adb(f"shell echo {probe}", serial=serial, timeout=timeout)
    return probe in out and not _adb_failed(out)


def broadcast(
    action: str,
    extras: dict,
    serial: str | None = None,
    ensure_app: bool = True,
) -> str:
    """Send an ADB broadcast with string extras.

    The app registers these receivers dynamically in MainActivity, so the GUI
    first starts the app on the target phone. The package flag keeps the
    broadcast explicit and avoids Android background broadcast filtering.
    """
    if ensure_app:
        start_result = adb_start_app(serial=serial)
        if _adb_failed(start_result):
            return start_result
        time.sleep(0.6)
    cmd = f"shell am broadcast -a {PKG}.{action} -p {PKG}"
    for k, v in extras.items():
        if v == "":
            continue
        cmd += f" --es {k} {shlex.quote(str(v))}"
    return adb(cmd, serial=serial)


def ros2_call(namespace: str, service: str, timeout: int = 10) -> str:
    """Call a ROS 2 Trigger service (may fail with rcljava)."""
    full = f"/{namespace}/{service}"
    return run_cmd(
        ["ros2", "service", "call", full, "std_srvs/srv/Trigger", "{}"],
        timeout=timeout,
        env=_mobile_ros_env(),
    )


def ros2_topic_cmd(namespace: str, command: str, timeout: int = 5) -> str:
    """Send a command via the ftm/command topic (reliable fallback).

    Supported commands: start, stop, wls, huber, trim, sigmaclip.
    """
    topic = f"/{namespace}/ftm/command"
    data = '{"data": "' + command + '"}'
    return run_cmd(
        [
            "ros2", "topic", "pub",
            "--once",
            "--wait-matching-subscriptions", "1",
            topic,
            "std_msgs/msg/String",
            data,
        ],
        timeout=timeout,
        env=_mobile_ros_env(),
    )


def ros2_topic_publisher_count(topic: str, timeout: int = 4) -> int:
    """Return the ROS 2 publisher count for a topic, or 0 on errors."""
    out = run_cmd(
        ["ros2", "topic", "info", topic],
        timeout=timeout,
        env=_mobile_ros_env(),
    )
    if out.startswith("ERROR"):
        return 0
    m = re.search(r"Publisher count:\s*(\d+)", out)
    return int(m.group(1)) if m else 0


def ros2_discover_namespaces() -> list[str]:
    """Discover RTT-SearchAgent namespaces from ROS 2 services or topics.

    Android rcljava services can be flaky across DDS/RMW boundaries, while the
    app always creates the ``ftm/command`` subscriber and publishes telemetry
    topics.  Falling back to topic discovery keeps the GUI usable even when
    ``ros2 service list`` does not expose the phone nodes.
    """
    control_namespaces: set[str] = set()
    telemetry_namespaces: set[str] = set()

    service_out = run_cmd(["ros2", "service", "list"], timeout=8, env=_mobile_ros_env())
    if not service_out.startswith("ERROR"):
        # Match /<namespace>/ftm/start_experiment
        service_pattern = re.compile(r"^/([^/]+)/ftm/start_experiment$", re.MULTILINE)
        control_namespaces.update(service_pattern.findall(service_out))

    topic_out = run_cmd(["ros2", "topic", "list"], timeout=8, env=_mobile_ros_env())
    if not topic_out.startswith("ERROR"):
        control_topic_pattern = re.compile(r"^/([^/]+)/ftm/command$", re.MULTILINE)
        telemetry_topic_pattern = re.compile(
            r"^/([^/]+)/(?:ftm_rtt|phone/location|ftm/anchor/[^/]+/(?:estimate|state))$",
            re.MULTILINE,
        )
        control_namespaces.update(control_topic_pattern.findall(topic_out))
        telemetry_namespaces.update(telemetry_topic_pattern.findall(topic_out))

    # Prefer namespaces that are actually controllable. Stale ROS discovery can
    # otherwise show the same phone twice after agent_id/namespace changes.
    if control_namespaces:
        return sorted(control_namespaces)

    # If no control topic is visible, keep only telemetry topics with a real
    # publisher. ``ros2 topic list`` can include subscription-only topics left
    # by peer subscriptions or ROS CLI probes; those are not active UAVs.
    active_telemetry = {
        ns for ns in telemetry_namespaces
        if ros2_topic_publisher_count(f"/{ns}/phone/location") > 0
        or ros2_topic_publisher_count(f"/{ns}/ftm_rtt") > 0
    }
    return sorted(active_telemetry)


def ros2_echo_once(topic: str, timeout: int = 3) -> str:
    """Echo one message from a ROS 2 topic."""
    return run_cmd(
        ["ros2", "topic", "echo", "--once", topic],
        timeout=timeout,
        env=_mobile_ros_env(),
    )


def _parse_float64_multiarray(raw: str) -> list[float] | None:
    """Parse data values from a ros2-echoed Float64MultiArray.

    Handles both YAML block sequence (one '- value' per line) and
    flow sequence ('data: [v1, v2, ...]') formats.
    """
    # Try flow-style first: data: [1.0, 2.0, ...]
    m = re.search(r"data:\s*\[([^\]]+)\]", raw)
    if m:
        try:
            return [float(v.strip()) for v in m.group(1).split(",")]
        except ValueError:
            pass
    # Try block-style: data:\n- 1.0\n- 2.0\n...
    m = re.search(r"data:\s*\n((?:\s*-\s*[\d.eE+\-]+\n?)+)", raw)
    if m:
        try:
            return [float(v.strip().lstrip("- ")) for v in m.group(1).strip().splitlines()]
        except ValueError:
            pass
    return None


def ros2_list_topics(pattern: str = "") -> list[str]:
    """List ROS 2 topics, optionally filtered by regex pattern."""
    out = run_cmd(["ros2", "topic", "list"], timeout=8, env=_mobile_ros_env())
    if out.startswith("ERROR"):
        return []
    topics = [t.strip() for t in out.splitlines() if t.strip()]
    if pattern:
        rx = re.compile(pattern)
        topics = [t for t in topics if rx.search(t)]
    return topics


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("RTT-SearchAgent — Mission Control")
        self.resizable(True, True)
        self.minsize(750, 550)

        style = ttk.Style(self)
        style.configure("Header.TLabel", font=("", 11, "bold"))
        style.configure("Status.TLabel", foreground="gray")
        style.configure("Running.TLabel", foreground="green", font=("", 10, "bold"))
        style.configure("Stopped.TLabel", foreground="gray", font=("", 10))
        style.configure("BigStart.TButton", foreground="green")
        style.configure("BigStop.TButton", foreground="red")
        style.configure("OsaTelemetry.TLabel", font=("Courier", 8))
        style.configure("OsaAgent.TLabel", font=("", 9, "bold"), foreground="#0d47a1")
        style.configure("OsaOk.TLabel", font=("", 9, "bold"), foreground="#2e7d32")
        style.configure("OsaWarn.TLabel", font=("", 9, "bold"), foreground="#e65100")
        style.configure("OsaError.TLabel", font=("", 9, "bold"), foreground="#c62828")

        main = ttk.Frame(self, padding=10)
        main.pack(fill="both", expand=True)

        # ── Status bar ──
        self.status_var = tk.StringVar(value="Ready")
        ttk.Label(main, textvariable=self.status_var, style="Status.TLabel").pack(
            anchor="w"
        )
        ttk.Separator(main).pack(fill="x", pady=4)

        # Track all ADB serial combos for bulk refresh
        self._all_serial_combos: list[ttk.Combobox] = []

        # ── Notebook with tabs ──
        nb = ttk.Notebook(main)
        nb.pack(fill="both", expand=True)
        self._nb = nb  # kept so tabs can switch programmatically

        # ===== Tab 0: About / Quick start =====
        self._build_about_tab(nb)

        # ===== Tab 1: Mission Control =====
        self._build_mission_tab(nb)

        # ===== Tab 2: Experiment Config =====
        self._build_experiment_tab(nb)

        # ===== Tab 3: NTRIP =====
        self._build_ntrip_tab(nb)

        # ===== Tab 4: Ground Truth =====
        self._build_gt_tab(nb)

        # ===== Tab 5: Multi-OSA Fusion =====
        self._build_multi_osa_tab(nb)

        # ===== Tab 6: AP Estimates =====
        self._build_estimates_tab(nb)

        # ===== Tab 7: Map =====
        self._build_map_tab(nb)

        # ===== Tab 8: Logs =====
        self._build_logs_tab(nb)

        # ── Output area (collapsible) ──
        ttk.Separator(main).pack(fill="x", pady=4)

        output_header = ttk.Frame(main)
        output_header.pack(fill="x")
        ttk.Label(output_header, text="Output:", style="Header.TLabel").pack(side="left")
        self._output_visible = True
        self._output_toggle_btn = ttk.Button(
            output_header, text="▼ Hide", width=8,
            command=self._toggle_output,
        )
        self._output_toggle_btn.pack(side="right")

        self._output_frame = ttk.Frame(main)
        self._output_frame.pack(fill="both", expand=True)
        self.output = tk.Text(self._output_frame, height=10, width=90, font=("Courier", 9))
        self.output.pack(fill="both", expand=True)

        # Right-click context menu for output
        self._output_menu = tk.Menu(self.output, tearoff=0)
        self._output_menu.add_command(label="Copy", accelerator="Ctrl+C",
                                      command=self._copy_output)
        self._output_menu.add_command(label="Select All", accelerator="Ctrl+A",
                                      command=lambda: self.output.tag_add("sel", "1.0", "end"))
        self._output_menu.add_separator()
        self._output_menu.add_command(label="Clear",
                                      command=lambda: self.output.delete("1.0", "end"))
        self.output.bind("<Button-3>",
                         lambda e: self._output_menu.tk_popup(e.x_root, e.y_root))
        self.output.bind("<Control-c>", lambda e: self._copy_output())
        self.output.bind("<Control-a>",
                         lambda e: (self.output.tag_add("sel", "1.0", "end"), "break")[1])

        # Discovered devices: {namespace: {"var": StringVar, "running": bool}}
        self.devices: dict[str, dict] = {}

        # Multi-OSA fusion subprocess
        self._multi_osa_proc: subprocess.Popen | None = None
        self._multi_osa_log_thread: threading.Thread | None = None

        # AP Estimates polling
        self._est_polling = False
        self._est_poll_thread: threading.Thread | None = None

        # Map markers: {(source_tag, bssid): marker_object}
        self._map_markers: dict[tuple[str, str], object] = {}
        # _build_map_tab() creates the widget. Do not overwrite it afterwards.
        if not hasattr(self, "_map_widget"):
            self._map_widget: object | None = None
        if not hasattr(self, "_map_tab"):
            self._map_tab: ttk.Frame | None = None
        self._ground_truth_positions: dict[str, tuple[float, float, float]] = {}  # bssid → (lat, lon, alt)
        # Populate GT from tools/ground_truth.csv on the next event-loop tick
        # (the Map widget is created inside _build_map_tab below; at this
        # point it already exists because _build_* ran earlier in __init__).
        self.after(200, self._autoload_ground_truth)

        # Per-OSA trajectory trails on the map
        # {namespace: {"points": list[(lat, lon)], "path": path_obj, "color": str}}
        self._osa_trails: dict[str, dict] = {}
        self._osa_trail_colors = [
            "#1565c0", "#ef6c00", "#2e7d32", "#6a1b9a",
            "#c62828", "#00838f", "#558b2f", "#4527a0",
        ]
        self._osa_trail_max = 300  # last N fixes kept per OSA
        # Stable colour assignment per namespace for live markers and trails.
        self._ns_color_index: dict[str, int] = {}
        self._live_estimate_rows: list[tuple] = []
        self._live_source_vars: dict[str, tk.BooleanVar] = {}

        # Per-OSA live telemetry poller
        self._telemetry_polling = False
        self._telemetry_thread: threading.Thread | None = None
        # {ns: {"agent_id", "coop", "fix", "fix_age_ms", "rtt_rate", "updated"}}
        self._osa_telemetry: dict[str, dict] = {}

        # Clean up subprocesses on window close
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        # Auto-reconnect saved ADB-over-TCP device
        cfg = self._load_config()
        saved_ip = cfg.get("adb_ip", "")
        saved_port = cfg.get("adb_port", "5555")
        if saved_ip:
            self.adb_net_ip.set(saved_ip)
            self.adb_net_port.set(str(saved_port))
            self._run_in_thread(self._adb_connect_tcp)

    # ──────────────────────────────────────────────────────────────
    #  Tab builders
    # ──────────────────────────────────────────────────────────────

    def _build_mission_tab(self, nb: ttk.Notebook):
        tab = self._new_tab_with_guide(nb, "Mission Control", GUIDE_MISSION, padding=8)

        # ── ADB over Network ──
        net_frame = ttk.LabelFrame(tab, text="ADB over Network (ZeroTier VPN)", padding=6)
        net_frame.pack(fill="x", pady=(0, 8))

        ip_row = ttk.Frame(net_frame)
        ip_row.pack(fill="x")
        ttk.Label(ip_row, text="Phone IP:").pack(side="left")
        self.adb_net_ip = tk.StringVar()
        ttk.Entry(ip_row, textvariable=self.adb_net_ip, width=18).pack(side="left", padx=4)
        ttk.Label(ip_row, text="Port:").pack(side="left")
        self.adb_net_port = tk.StringVar(value="5555")
        ttk.Entry(ip_row, textvariable=self.adb_net_port, width=6).pack(side="left", padx=4)

        self.adb_net_status = tk.StringVar(value="Not connected")
        ttk.Label(ip_row, textvariable=self.adb_net_status,
                  foreground="gray").pack(side="right", padx=8)

        btn_row = ttk.Frame(net_frame)
        btn_row.pack(fill="x", pady=(4, 0))
        ttk.Button(btn_row, text="Connect",
                   command=lambda: self._run_in_thread(self._adb_connect_tcp)).pack(side="left")
        ttk.Button(btn_row, text="Disconnect",
                   command=lambda: self._run_in_thread(self._adb_disconnect_tcp)).pack(side="left", padx=4)
        ttk.Button(btn_row, text="Enable TCP mode (USB)",
                   command=lambda: self._run_in_thread(self._adb_enable_tcpip)).pack(side="left", padx=8)

        ttk.Label(btn_row, text="⚡ First 'Enable TCP mode' using a cable; second 'Connect' always.",
                  foreground="gray", font=("", 8)).pack(side="left", padx=8)

        # Top row: discover + global actions
        top = ttk.Frame(tab)
        top.pack(fill="x", pady=(0, 8))

        ttk.Button(top, text="Discover UAVs (ROS 2)",
                   command=self.discover_uavs).pack(side="left")
        ttk.Button(top, text="Check ADB devices",
                   command=self.check_device).pack(side="left", padx=8)

        ttk.Separator(top, orient="vertical").pack(side="left", fill="y", padx=8)

        start_all = ttk.Button(
            top, text="START ALL",
            command=self.start_all_experiments,
            style="BigStart.TButton",
        )
        start_all.pack(side="left", padx=4)

        stop_all = ttk.Button(
            top, text="STOP ALL",
            command=self.stop_all_experiments,
            style="BigStop.TButton",
        )
        stop_all.pack(side="left", padx=4)

        # ── ADB-first control plane ──
        adb_frame = ttk.LabelFrame(
            tab,
            text="ADB devices (always configurable; independent of ROS/RMW)",
            padding=6,
        )
        adb_frame.pack(fill="x", pady=(0, 8))
        self.adb_devices_var = tk.StringVar(value="Not refreshed yet")
        ttk.Label(adb_frame, textvariable=self.adb_devices_var, foreground="gray").pack(
            side="left", padx=(0, 8)
        )
        ttk.Button(adb_frame, text="Refresh ADB list",
                   command=self._refresh_adb_serials).pack(side="left", padx=2)
        ttk.Button(adb_frame, text="Start app on ALL",
                   command=lambda: self._run_in_thread(self._adb_start_all_apps)).pack(side="left", padx=2)
        ttk.Button(adb_frame, text="Force-stop app on ALL",
                   command=lambda: self._run_in_thread(self._adb_force_stop_all_apps)).pack(side="left", padx=2)
        ttk.Button(adb_frame, text="START via ADB ALL",
                   command=lambda: self._run_in_thread(self._adb_remote_all, "start")).pack(side="left", padx=(10, 2))
        ttk.Button(adb_frame, text="STOP via ADB ALL",
                   command=lambda: self._run_in_thread(self._adb_remote_all, "stop")).pack(side="left", padx=2)

        # ── Pre-flight Checklist ──
        check_frame = ttk.LabelFrame(tab, text="Pre-flight Checklist", padding=6)
        check_frame.pack(fill="x", pady=(0, 8))

        self._check_vars: dict[str, tk.StringVar] = {}
        checks = [
            ("adb", "ADB connected"),
            ("uavs", "UAVs discovered"),
            ("config", "Experiment config sent"),
            ("gt", "Ground truth loaded"),
            ("ntrip", "NTRIP / RTK active"),
            ("fusion", "Multi-OSA running"),
        ]
        for i, (key, label) in enumerate(checks):
            col = (i % 3) * 2
            row = i // 3
            var = tk.StringVar(value="⬜")
            self._check_vars[key] = var
            lbl = ttk.Label(check_frame, textvariable=var, font=("", 12))
            lbl.grid(row=row, column=col, sticky="w", padx=(4, 0))
            ttk.Label(check_frame, text=label, font=("", 9)).grid(
                row=row, column=col + 1, sticky="w", padx=(2, 16))

        ttk.Separator(tab).pack(fill="x", pady=4)

        # Device list (populated by discover)
        ttk.Label(tab, text="Discovered UAVs:", style="Header.TLabel").pack(anchor="w")
        self.device_frame = ttk.Frame(tab)
        self.device_frame.pack(fill="both", expand=True)

        # Placeholder
        self.no_devices_label = ttk.Label(
            self.device_frame,
            text='No UAVs discovered yet. Click "Discover UAVs (ROS 2)" to scan.',
            foreground="gray",
        )
        self.no_devices_label.pack(pady=20)

    def _build_experiment_tab(self, nb: ttk.Notebook):
        exp = self._new_tab_with_guide(nb, "Experiment Config", GUIDE_EXPERIMENT, padding=8)

        ttk.Label(exp, text="Pre-flight Config (ADB broadcast)",
                  style="Header.TLabel").grid(row=0, column=0, columnspan=3, sticky="w")

        self.exp_label = self._labeled_entry(exp, "Label:", 1,
            tooltip="Etiqueta del experimento (se guarda en los logs).")

        # Altitude mode selector (auto = no AGL seed; manual_agl = legacy)
        ttk.Label(exp, text="Altitude mode:").grid(row=2, column=0, sticky="w", pady=2)
        self.exp_alt_mode = ttk.Combobox(
            exp, values=["auto", "manual_agl"], state="readonly", width=14
        )
        self.exp_alt_mode.set("auto")
        self.exp_alt_mode.grid(row=2, column=1, sticky="w", pady=2)
        ToolTip(self.exp_alt_mode,
                "auto  (recomendado): no se envía semilla AGL; el solver 3D\n"
                "  recupera la Z del anchor a partir de la diversidad vertical\n"
                "  de las trayectorias de uno o varios drones.\n"
                "manual_agl: se envía AGL y se usa como prior de Z (legacy).")
        self.exp_alt_mode.bind("<<ComboboxSelected>>", lambda _e: self._on_altitude_mode_change())

        self.exp_agl = self._labeled_entry(exp, "AGL (m):", 3, default="15.0",
            tooltip="Altura del dron sobre el suelo (AGL). Sólo se usa en modo manual_agl.")
        # Keep a reference to the AGL row widgets so we can hide/show them.
        self._agl_row_widgets = [w for w in exp.grid_slaves(row=3)]
        self.exp_bias = self._labeled_entry(exp, "Range bias (m):", 4, default="0.0",
            tooltip="Sesgo sistemático RTT a restar del rango (m). +2.5 típico.")
        self.exp_notes = self._labeled_entry(exp, "Notes:", 5,
            tooltip="Notas libres (se guardan en experiment.jsonl).")

        ttk.Label(exp, text="Filter mode:").grid(row=6, column=0, sticky="w", pady=2)
        self.filter_modes = [
            "BASELINE", "NLOS", "EMPIRICAL_R", "UKF", "EMA", "ADAPTIVE_R",
        ]
        self.exp_filter = ttk.Combobox(
            exp, values=self.filter_modes, state="readonly", width=18
        )
        self.exp_filter.set("BASELINE")
        self.exp_filter.grid(row=6, column=1, sticky="ew", pady=2)
        ToolTip(self.exp_filter,
                "BASELINE: IEKF3D + Median (recomendado)\n"
                "NLOS: IEKF3D + detector NLOS (infla σ en multipath)\n"
                "EMPIRICAL_R: IEKF3D + modelo R empírico (dist+RSSI)\n"
                "UKF: UKF3D sin sesgo (3 estados)\n"
                "EMA: IEKF3D + suavizado EMA en salida\n"
                "ADAPTIVE_R: IEKF3D + R adaptativo (innovación)")

        # ADB serial selector
        ttk.Label(exp, text="ADB device:").grid(row=7, column=0, sticky="w", pady=2)
        self.adb_serial_var = tk.StringVar(value="(any)")
        self.adb_serial_combo = ttk.Combobox(
            exp, textvariable=self.adb_serial_var, state="readonly", width=28,
            values=["(any)"],
        )
        self.adb_serial_combo.grid(row=7, column=1, sticky="ew", pady=2)
        self._all_serial_combos.append(self.adb_serial_combo)
        ttk.Button(exp, text="Refresh", command=self._refresh_adb_serials).grid(
            row=7, column=2, sticky="w", padx=4
        )

        btn_frame_exp = ttk.Frame(exp)
        btn_frame_exp.grid(row=8, column=0, columnspan=3, sticky="ew", pady=(8, 4))
        ttk.Button(
            btn_frame_exp, text="Send Experiment Config", command=self.send_experiment
        ).pack(side="left", expand=True, fill="x", padx=(0, 4))
        ttk.Button(
            btn_frame_exp, text="Send to ALL devices", command=self.send_experiment_all
        ).pack(side="left", expand=True, fill="x", padx=(4, 0))

        # ── Runtime tuning ──
        ttk.Separator(exp).grid(row=9, column=0, columnspan=3, sticky="ew", pady=8)
        ttk.Label(exp, text="Runtime Tuning (no restart needed)",
                  style="Header.TLabel").grid(row=10, column=0, columnspan=3, sticky="w")

        ttk.Label(exp, text="GPS↔RTT sync window (ms):").grid(
            row=11, column=0, sticky="w", pady=2
        )
        self.sync_window_values = ["120", "300", "600", "1200", "2000", "5000"]
        self.sync_window_combo = ttk.Combobox(
            exp, values=self.sync_window_values, width=10
        )
        self.sync_window_combo.set("1200")
        self.sync_window_combo.grid(row=11, column=1, sticky="w", pady=2)
        ttk.Button(
            exp, text="Apply", command=self._send_sync_window
        ).grid(row=11, column=2, sticky="w", padx=4)

        # ── Cooperative (N-OSA) ──
        ttk.Separator(exp).grid(row=12, column=0, columnspan=4, sticky="ew", pady=8)
        ttk.Label(exp, text="Cooperative (N-OSA)",
                  style="Header.TLabel").grid(row=13, column=0, columnspan=4, sticky="w")
        ttk.Label(exp,
                  text="Identity per OSA and ring-topology ranging staggering.",
                  foreground="gray", font=("", 8)).grid(
                      row=14, column=0, columnspan=4, sticky="w", pady=(0, 4))

        self.coop_agent_id = tk.StringVar(value="")
        self.coop_peer_id = tk.StringVar(value="")
        self.coop_period = tk.StringVar(value="")
        self.coop_phase = tk.StringVar(value="")

        ttk.Label(exp, text="agent_id:").grid(row=15, column=0, sticky="w", pady=2)
        e_agent = ttk.Entry(exp, textvariable=self.coop_agent_id, width=16)
        e_agent.grid(row=15, column=1, sticky="w", pady=2)
        ToolTip(e_agent,
                "Identity for this OSA (e.g. osa1). Becomes the ROS 2 "
                "namespace of the phone. Restart the app after changing.")

        ttk.Label(exp, text="peer_agent_id:").grid(row=15, column=2, sticky="e", padx=(8, 2))
        e_peer = ttk.Entry(exp, textvariable=self.coop_peer_id, width=12)
        e_peer.grid(row=15, column=3, sticky="w", pady=2)
        ToolTip(e_peer, "Peer OSA identity. Empty = independent.")

        ttk.Label(exp, text="cooperative_mode:").grid(row=16, column=0, sticky="w", pady=2)
        self.coop_mode = ttk.Combobox(
            exp, values=["", "independent", "fused_offboard", "fused_onboard"],
            state="readonly", width=14)
        self.coop_mode.set("")
        self.coop_mode.grid(row=16, column=1, sticky="w", pady=2)
        ToolTip(self.coop_mode,
                "Blank: don't change.\n"
                "independent: single OSA.\n"
                "fused_offboard: publish for external fusion (recommended).\n"
                "fused_onboard: experimental peer RTT injection.")

        ttk.Label(exp, text="ranging_period_ms:").grid(row=17, column=0, sticky="w", pady=2)
        ttk.Entry(exp, textvariable=self.coop_period, width=8).grid(
            row=17, column=1, sticky="w", pady=2)
        ttk.Label(exp, text="phase_ms:").grid(row=17, column=2, sticky="e", padx=(8, 2))
        ttk.Entry(exp, textvariable=self.coop_phase, width=8).grid(
            row=17, column=3, sticky="w", pady=2)

        wiz = ttk.LabelFrame(exp, text="Auto-configure N OSAs (ring)", padding=6)
        wiz.grid(row=18, column=0, columnspan=4, sticky="ew", pady=(8, 0))

        self.wiz_n = tk.StringVar(value="2")
        self.wiz_prefix = tk.StringVar(value="osa")
        self.wiz_period = tk.StringVar(value="240")

        ttk.Label(wiz, text="N:").grid(row=0, column=0, sticky="e")
        ttk.Entry(wiz, textvariable=self.wiz_n, width=4).grid(row=0, column=1, padx=(2, 8))
        ttk.Label(wiz, text="prefix:").grid(row=0, column=2, sticky="e")
        ttk.Entry(wiz, textvariable=self.wiz_prefix, width=8).grid(row=0, column=3, padx=(2, 8))
        ttk.Label(wiz, text="period ms:").grid(row=0, column=4, sticky="e")
        ttk.Entry(wiz, textvariable=self.wiz_period, width=6).grid(row=0, column=5, padx=(2, 8))

        ttk.Button(wiz, text="Configure N OSAs",
                   command=lambda: self._run_in_thread(self._auto_configure_n_osas)
                   ).grid(row=1, column=0, columnspan=3, sticky="ew", pady=(6, 0))
        ttk.Button(wiz, text="Preview assignment",
                   command=self._preview_n_osa_assignment
                   ).grid(row=1, column=3, columnspan=3, sticky="ew", pady=(6, 0))

        ttk.Label(wiz,
                  text="⚠ agent_id requires app restart on each phone to rename ROS topics.",
                  foreground="#c62828", font=("", 8)).grid(
                      row=2, column=0, columnspan=6, sticky="w", pady=(4, 0))

        # Initial visibility of the AGL row depends on default altitude mode.
        self._on_altitude_mode_change()

    def _on_altitude_mode_change(self):
        """Show/hide the AGL entry depending on the selected altitude mode."""
        mode = (self.exp_alt_mode.get() or "auto").strip().lower()
        show = (mode == "manual_agl")
        for w in getattr(self, "_agl_row_widgets", []) or []:
            try:
                if show:
                    w.grid()
                else:
                    w.grid_remove()
            except Exception:
                pass

    def _build_ntrip_tab(self, nb: ttk.Notebook):
        ntrip = self._new_tab_with_guide(nb, "NTRIP", GUIDE_NTRIP, padding=8)

        # ── Phone Status (top) ──
        status_frame = ttk.LabelFrame(ntrip, text="Phone Status", padding=6)
        status_frame.grid(row=0, column=0, columnspan=3, sticky="ew", pady=(0, 8))

        self.phone_ntrip_status = tk.StringVar(value="—")
        self.phone_fix_label = tk.StringVar(value="—")
        self.phone_sats = tk.StringVar(value="—")
        self.phone_hdop = tk.StringVar(value="—")
        self.phone_rtcm = tk.StringVar(value="—")
        self.phone_ext_gnss = tk.StringVar(value="—")
        self.phone_sync_window = tk.StringVar(value="—")
        self.phone_weighting = tk.StringVar(value="—")
        self.phone_ranging = tk.StringVar(value="—")

        labels_and_vars = [
            ("NTRIP:", self.phone_ntrip_status),
            ("Fix:", self.phone_fix_label),
            ("Sats:", self.phone_sats),
            ("HDOP:", self.phone_hdop),
            ("RTCM bytes:", self.phone_rtcm),
            ("Ext. GNSS:", self.phone_ext_gnss),
            ("Sync window:", self.phone_sync_window),
            ("Weighting:", self.phone_weighting),
            ("Ranging:", self.phone_ranging),
        ]
        for i, (lbl_text, var) in enumerate(labels_and_vars):
            col = (i % 3) * 2
            row = i // 3
            ttk.Label(status_frame, text=lbl_text).grid(row=row, column=col, sticky="w", padx=(4, 2))
            lbl = ttk.Label(status_frame, textvariable=var, font=("", 9, "bold"))
            lbl.grid(row=row, column=col + 1, sticky="w", padx=(0, 12))

        btn_frame_status = ttk.Frame(status_frame)
        btn_frame_status.grid(row=3, column=0, columnspan=6, sticky="ew", pady=(6, 0))
        ttk.Button(btn_frame_status, text="Refresh from phone",
                   command=lambda: self._run_in_thread(self._query_phone_status)).pack(side="left")
        ttk.Button(btn_frame_status, text="NTRIP Connect",
                   command=lambda: self._run_in_thread(self._ntrip_connect)).pack(side="left", padx=8)
        ttk.Button(btn_frame_status, text="NTRIP Disconnect",
                   command=lambda: self._run_in_thread(self._ntrip_disconnect)).pack(side="left")

        # ── NTRIP Config (bottom) ──
        ttk.Label(ntrip, text="NTRIP RTK Config", style="Header.TLabel").grid(
            row=1, column=0, columnspan=2, sticky="w"
        )
        self.ntrip_host = self._labeled_entry(
            ntrip, "Host:", 2, default="caster.centipede.fr"
        )
        self.ntrip_port = self._labeled_entry(ntrip, "Port:", 3, default="2101")
        self.ntrip_mount = self._labeled_entry(ntrip, "Mountpoint:", 4)
        self.ntrip_user = self._labeled_entry(ntrip, "User:", 5)
        self.ntrip_pass = self._labeled_entry(ntrip, "Password:", 6, show="*")

        # ADB device selector
        ttk.Label(ntrip, text="ADB device:").grid(row=7, column=0, sticky="w", pady=2)
        self.ntrip_serial_var = tk.StringVar(value="(any)")
        self.ntrip_serial_combo = ttk.Combobox(
            ntrip, textvariable=self.ntrip_serial_var, state="readonly", width=28,
            values=["(any)"],
        )
        self.ntrip_serial_combo.grid(row=7, column=1, sticky="ew", pady=2)
        self._all_serial_combos.append(self.ntrip_serial_combo)
        ttk.Button(ntrip, text="Refresh", command=self._refresh_adb_serials).grid(
            row=7, column=2, sticky="w", padx=4
        )

        btn_frame_ntrip = ttk.Frame(ntrip)
        btn_frame_ntrip.grid(row=8, column=0, columnspan=3, sticky="ew", pady=(8, 4))
        ttk.Button(
            btn_frame_ntrip, text="Send NTRIP Config", command=self.send_ntrip
        ).pack(side="left", expand=True, fill="x", padx=(0, 4))
        ttk.Button(
            btn_frame_ntrip, text="Send to ALL devices", command=self.send_ntrip_all
        ).pack(side="left", expand=True, fill="x", padx=(4, 0))

    def _build_gt_tab(self, nb: ttk.Notebook):
        gt = self._new_tab_with_guide(nb, "Ground Truth", GUIDE_GT, padding=8)

        ttk.Label(gt, text="Register AP Ground Truth", style="Header.TLabel").grid(
            row=0, column=0, columnspan=2, sticky="w"
        )
        self.gt_bssid = self._labeled_entry(
            gt, "BSSID:", 1, default="aa:bb:cc:dd:ee:ff"
        )
        self.gt_lat = self._labeled_entry(gt, "Latitude:", 2)
        self.gt_lon = self._labeled_entry(gt, "Longitude:", 3)
        self.gt_alt = self._labeled_entry(gt, "Altitude:", 4, default="0.0")
        self.gt_desc = self._labeled_entry(gt, "Description:", 5)

        # ADB device selector
        ttk.Label(gt, text="ADB device:").grid(row=6, column=0, sticky="w", pady=2)
        self.gt_serial_var = tk.StringVar(value="(any)")
        self.gt_serial_combo = ttk.Combobox(
            gt, textvariable=self.gt_serial_var, state="readonly", width=28,
            values=["(any)"],
        )
        self.gt_serial_combo.grid(row=6, column=1, sticky="ew", pady=2)
        self._all_serial_combos.append(self.gt_serial_combo)
        ttk.Button(gt, text="Refresh", command=self._refresh_adb_serials).grid(
            row=6, column=2, sticky="w", padx=4
        )

        btn_frame_gt = ttk.Frame(gt)
        btn_frame_gt.grid(row=7, column=0, columnspan=3, sticky="ew", pady=(8, 4))
        ttk.Button(
            btn_frame_gt, text="Send Ground Truth", command=self.send_gt
        ).pack(side="left", expand=True, fill="x", padx=(0, 4))
        ttk.Button(
            btn_frame_gt, text="Send to ALL devices", command=self.send_gt_all
        ).pack(side="left", expand=True, fill="x", padx=(4, 0))

        ttk.Separator(gt).grid(row=8, column=0, columnspan=3, sticky="ew", pady=8)
        ttk.Label(gt, text="Or upload CSV file:").grid(row=9, column=0, sticky="w")
        ttk.Button(gt, text="Push ground_truth.csv …", command=self.push_gt_csv).grid(
            row=9, column=1, sticky="e"
        )

    def _build_multi_osa_tab(self, nb: ttk.Notebook):
        tab = self._new_tab_with_guide(nb, "Multi-OSA", GUIDE_MULTI_OSA, padding=8)

        ttk.Label(tab, text="Multi-OSA Centralised Fusion",
                  style="Header.TLabel").pack(anchor="w")
        ttk.Label(tab, text="Launch networked-OSA.py as a ROS 2 node for multi-drone multilateration.",
                  foreground="gray", font=("", 9)).pack(anchor="w", pady=(0, 8))

        # ── Parameters ──
        params = ttk.LabelFrame(tab, text="Parameters", padding=6)
        params.pack(fill="x", pady=(0, 8))

        self.mosa_agl = self._labeled_entry(params, "AGL hint (m):", 0, default="10.0",
            tooltip="Altura AGL aprox. del dron (m). Usado para estimar z del AP.")
        self.mosa_bias = self._labeled_entry(params, "Range bias (m):", 1, default="0.0",
            tooltip="Sesgo RTT a restar de cada distancia (m). +2.5 m típico en Pixel 7 Pro.")
        self.mosa_sigma = self._labeled_entry(params, "σ_H threshold (m):", 2, default="2.5",
            tooltip="Publicar estimación solo cuando σ_H < este valor (m).")
        self.mosa_min_samples = self._labeled_entry(params, "Min samples:", 3, default="6",
            tooltip="Mínimo de observaciones RTT para inicializar Gauss-Newton 3D.")
        self.mosa_gnss_acc = self._labeled_entry(params, "GNSS acc max (m):", 4, default="5.0",
            tooltip="Descartar fixes GNSS con accuracy > este valor (m).\n"
                    "5 m ≈ RTK de buena calidad; subir a 15 si se vuela sin RTK\n"
                    "(a costa de ENU origin peor y multilateración más ruidosa).")
        self.mosa_z_prior_sigma = self._labeled_entry(params, "z-prior σ (m):", 5, default="5.0",
            tooltip="σ del prior de altitud z (m). Menor = prior más fuerte.\n"
                    "5 m para APs en superficie, 10+ para enterrados.")
        self.mosa_log_dir = self._labeled_entry(params, "Log directory:", 6,
                                                default=str(FUSION_LOGS_DIR))

        browse_btn = ttk.Button(params, text="Browse…",
                                command=self._browse_mosa_log_dir)
        browse_btn.grid(row=6, column=2, sticky="w", padx=4)

        # Divergence-reset controls. When enabled the fusion node resets
        # an anchor's EKF back to Gauss-Newton bootstrap if either a
        # sustained rejection streak or a large mean NIS is observed —
        # cheap defence against a track that gets stuck on a bad local
        # optimum.
        self.mosa_reset_enabled = tk.BooleanVar(value=True)
        reset_row = ttk.Frame(params)
        reset_row.grid(row=7, column=0, columnspan=3, sticky="w", pady=(6, 0))
        ttk.Checkbutton(
            reset_row, text="Reset EKF on divergence",
            variable=self.mosa_reset_enabled,
        ).pack(side="left")
        ttk.Label(reset_row, text="reject streak:",
                  foreground="gray").pack(side="left", padx=(12, 2))
        self.mosa_reset_streak = tk.StringVar(value="8")
        ttk.Entry(reset_row, textvariable=self.mosa_reset_streak,
                  width=4).pack(side="left")
        ttk.Label(reset_row, text="mean NIS:",
                  foreground="gray").pack(side="left", padx=(12, 2))
        self.mosa_reset_nis = tk.StringVar(value="6.0")
        ttk.Entry(reset_row, textvariable=self.mosa_reset_nis,
                  width=5).pack(side="left")

        # ── Controls ──
        ctrl = ttk.Frame(tab)
        ctrl.pack(fill="x", pady=8)

        self.mosa_start_btn = ttk.Button(
            ctrl, text="START Multi-OSA",
            command=lambda: self._run_in_thread(self._start_multi_osa),
            style="BigStart.TButton",
        )
        self.mosa_start_btn.pack(side="left", padx=4)

        self.mosa_stop_btn = ttk.Button(
            ctrl, text="STOP Multi-OSA",
            command=lambda: self._run_in_thread(self._stop_multi_osa),
            style="BigStop.TButton",
            state="disabled",
        )
        self.mosa_stop_btn.pack(side="left", padx=4)

        self.mosa_status_var = tk.StringVar(value="STOPPED")
        self.mosa_status_lbl = ttk.Label(ctrl, textvariable=self.mosa_status_var,
                                         style="Stopped.TLabel")
        self.mosa_status_lbl.pack(side="left", padx=12)

        self.mosa_pid_var = tk.StringVar(value="")
        ttk.Label(ctrl, textvariable=self.mosa_pid_var,
                  foreground="gray", font=("", 8)).pack(side="left", padx=8)

        # ── Live output ──
        ttk.Separator(tab).pack(fill="x", pady=4)
        ttk.Label(tab, text="Fusion node output:", style="Header.TLabel").pack(anchor="w")

        mosa_out_frame = ttk.Frame(tab)
        mosa_out_frame.pack(fill="both", expand=True)

        self.mosa_output = tk.Text(mosa_out_frame, height=12, width=90,
                                   font=("Courier", 8), state="disabled", wrap="word")
        mosa_scroll = ttk.Scrollbar(mosa_out_frame, orient="vertical",
                                     command=self.mosa_output.yview)
        self.mosa_output.configure(yscrollcommand=mosa_scroll.set)
        self.mosa_output.pack(side="left", fill="both", expand=True)
        mosa_scroll.pack(side="right", fill="y")

    def _build_estimates_tab(self, nb: ttk.Notebook):
        tab = self._new_tab_with_guide(nb, "AP Estimates", GUIDE_ESTIMATES, padding=8)

        ttk.Label(tab, text="Live AP Position Estimates",
                  style="Header.TLabel").pack(anchor="w")

        # ── Controls ──
        ctrl = ttk.Frame(tab)
        ctrl.pack(fill="x", pady=(4, 8))

        self.est_poll_btn = ttk.Button(
            ctrl, text="Start polling",
            command=self._toggle_est_polling,
        )
        self.est_poll_btn.pack(side="left", padx=4)

        ttk.Button(ctrl, text="Refresh now",
                   command=lambda: self._run_in_thread(self._poll_estimates_once)).pack(
            side="left", padx=4)

        self.est_interval_var = tk.StringVar(value="3")
        ttk.Label(ctrl, text="Interval (s):").pack(side="left", padx=(12, 2))
        ttk.Entry(ctrl, textvariable=self.est_interval_var, width=4).pack(side="left")

        self.est_status_var = tk.StringVar(value="")
        ttk.Label(ctrl, textvariable=self.est_status_var,
                  foreground="gray", font=("", 8)).pack(side="left", padx=12)

        src_frame = ttk.LabelFrame(tab, text="Live source filters", padding=4)
        src_frame.pack(fill="x", pady=(0, 8))
        src_ctrl = ttk.Frame(src_frame)
        src_ctrl.pack(fill="x")
        ttk.Button(
            src_ctrl, text="All",
            command=lambda: self._set_live_source_visibility(True),
        ).pack(side="left")
        ttk.Button(
            src_ctrl, text="None",
            command=lambda: self._set_live_source_visibility(False),
        ).pack(side="left", padx=4)
        self._live_sources_summary = tk.StringVar(value="No live sources yet.")
        ttk.Label(src_ctrl, textvariable=self._live_sources_summary,
                  foreground="gray").pack(side="left", padx=10)
        self._live_sources_checks = ttk.Frame(src_frame)
        self._live_sources_checks.pack(fill="x", pady=(4, 0))

        # ── Treeview table ──
        columns = ("source", "bssid", "lat", "lon", "alt", "sigma_h", "n_acc", "err_2d", "updated")
        self.est_tree = ttk.Treeview(tab, columns=columns, show="headings", height=14)

        col_widths = {
            "source": 130, "bssid": 140, "lat": 100, "lon": 100,
            "alt": 65, "sigma_h": 62, "n_acc": 55, "err_2d": 70, "updated": 70,
        }
        col_headers = {
            "source": "Source", "bssid": "BSSID", "lat": "Latitude",
            "lon": "Longitude", "alt": "Alt (m)", "sigma_h": "σ_H (m)",
            "n_acc": "n_acc", "err_2d": "Err 2D (m)", "updated": "Updated",
        }
        for c in columns:
            self.est_tree.heading(c, text=col_headers[c])
            self.est_tree.column(c, width=col_widths[c], minwidth=50)

        tree_scroll = ttk.Scrollbar(tab, orient="vertical", command=self.est_tree.yview)
        self.est_tree.configure(yscrollcommand=tree_scroll.set)
        self.est_tree.pack(side="left", fill="both", expand=True)
        tree_scroll.pack(side="right", fill="y")

        # Tag colours
        self.est_tree.tag_configure("fusion", background="#e8f5e9")
        self.est_tree.tag_configure("phone", background="#e3f2fd")
        self.est_tree.tag_configure("good_err", foreground="#2e7d32")
        self.est_tree.tag_configure("warn_err", foreground="#e65100")

    # ──────────────────────────────────────────────────────────────
    #  Tab 7: Interactive Map
    # ──────────────────────────────────────────────────────────────

    # Tile server URLs
    _TILE_OSM = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
    _TILE_SAT = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
    _TILE_OSM_DE = "https://tile.openstreetmap.de/{z}/{x}/{y}.png"
    _TILE_OPENTOPO = "https://tile.opentopomap.org/{z}/{x}/{y}.png"
    _TILE_CARTO_VOYAGER = "https://basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}.png"
    _TILE_CARTO_LIGHT = "https://basemaps.cartocdn.com/light_all/{z}/{x}/{y}.png"
    _TILE_ESRI_TOPO = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Topo_Map/MapServer/tile/{z}/{y}/{x}"

    _MAP_TILE_PROVIDERS = {
        "Satellite": {
            "url": _TILE_SAT,
            "min_cap": 20,
            "max_cap": 24,
            "default_cap": 23,
        },
        "OpenStreetMap": {
            "url": _TILE_OSM,
            "min_cap": 18,
            "max_cap": 22,
            "default_cap": 20,
        },
        "OpenStreetMap DE": {
            "url": _TILE_OSM_DE,
            "min_cap": 18,
            "max_cap": 22,
            "default_cap": 20,
        },
        "OpenTopoMap": {
            "url": _TILE_OPENTOPO,
            "min_cap": 17,
            "max_cap": 20,
            "default_cap": 19,
        },
        "Carto Voyager": {
            "url": _TILE_CARTO_VOYAGER,
            "min_cap": 18,
            "max_cap": 21,
            "default_cap": 20,
        },
        "Carto Light": {
            "url": _TILE_CARTO_LIGHT,
            "min_cap": 18,
            "max_cap": 21,
            "default_cap": 20,
        },
        "Esri Topo": {
            "url": _TILE_ESRI_TOPO,
            "min_cap": 18,
            "max_cap": 22,
            "default_cap": 20,
        },
    }

    def _build_map_tab(self, nb: ttk.Notebook):
        tab = self._new_tab_with_guide(nb, "Map", GUIDE_MAP, padding=4)
        self._map_tab = tab

        if not _HAS_MAP:
            ttk.Label(
                tab,
                text=(
                    "Map backend not installed.\n\n"
                    "To enable the interactive map tab, activate the project\n"
                    "virtualenv and run:\n\n"
                    "    pip install tkintermapview\n\n"
                    "Then restart this GUI."
                ),
                foreground="#c62828", font=("", 11),
                justify="left",
            ).pack(expand=True, padx=20, pady=20)
            return

        # ── Top toolbar ──
        toolbar = ttk.Frame(tab)
        toolbar.pack(fill="x", pady=(0, 4))

        ttk.Label(toolbar, text="Map", style="Header.TLabel").pack(side="left")

        # Tile layer selector
        self._tile_var = tk.StringVar(value="Satellite")
        ttk.Label(toolbar, text="  Layer:").pack(side="left", padx=(12, 2))
        tile_combo = ttk.Combobox(
            toolbar, textvariable=self._tile_var,
            values=list(self._MAP_TILE_PROVIDERS.keys()), state="readonly", width=18,
        )
        tile_combo.pack(side="left")
        tile_combo.bind("<<ComboboxSelected>>", self._on_tile_change)
        self._map_tile_combo = tile_combo
        ToolTip(
            tile_combo,
            "Switch tile providers when one of them renders blank or is slow. "
            "The map keeps the current view when you change provider.")

        self._map_zoom_var = tk.StringVar(value="6")
        ttk.Label(toolbar, text="  Zoom:").pack(side="left", padx=(12, 2))
        self._map_zoom_spin = ttk.Spinbox(
            toolbar, textvariable=self._map_zoom_var,
            from_=1, to=24, width=4,
        )
        self._map_zoom_spin.pack(side="left")
        self._map_zoom_spin.bind("<Return>", self._on_map_zoom_change)
        ttk.Button(
            toolbar, text="Set", command=self._on_map_zoom_change,
        ).pack(side="left", padx=2)

        self._map_zoom_cap_var = tk.StringVar(value="23")
        ttk.Label(toolbar, text="  Max zoom:").pack(side="left", padx=(12, 2))
        self._map_zoom_cap_spin = ttk.Spinbox(
            toolbar, textvariable=self._map_zoom_cap_var,
            values=self._map_zoom_cap_options("Satellite"), width=4,
        )
        self._map_zoom_cap_spin.pack(side="left")
        self._map_zoom_cap_spin.bind("<Return>", self._on_map_zoom_cap_change)
        self._map_zoom_cap_spin.bind("<FocusOut>", self._on_map_zoom_cap_change)
        ToolTip(
            self._map_zoom_cap_spin,
            "Upper zoom limit for the active tile layer. Use it to stay on the "
            "highest zoom that still keeps the terrain imagery visible.")
        ttk.Button(
            toolbar, text="Max", command=self._map_zoom_to_cap,
        ).pack(side="left", padx=2)

        ttk.Button(
            toolbar, text="Clear markers",
            command=self._map_clear_markers,
        ).pack(side="left", padx=12)

        ttk.Button(
            toolbar, text="Clear trails",
            command=self.clear_osa_trails,
        ).pack(side="left", padx=4)

        ttk.Button(
            toolbar, text="Refresh from estimates",
            command=lambda: self._run_in_thread(self._map_refresh_from_estimates),
        ).pack(side="left", padx=4)

        # ── Go-to coordinate ──
        ttk.Label(toolbar, text="  Go to:").pack(side="left", padx=(12, 2))
        self._map_goto_var = tk.StringVar(value="")
        goto_entry = ttk.Entry(toolbar, textvariable=self._map_goto_var, width=28)
        goto_entry.pack(side="left")
        goto_entry.bind("<Return>", lambda _e: self._map_goto())
        ttk.Button(toolbar, text="Go", command=self._map_goto).pack(side="left", padx=2)

        # ── Map widget ──
        self._map_widget = TkinterMapView(tab, corner_radius=0)
        self._map_widget.pack(fill="both", expand=True)

        # Default: satellite layer, zoom on Spain as a sensible starting point.
        self._map_sync_zoom_cap_ui("Satellite")
        self._map_apply_tile_server(preserve_view=False)
        self._map_widget.set_position(40.4168, -3.7038)  # Madrid
        self._map_set_zoom(6)

    def _map_provider_config(self, layer: str | None = None) -> dict:
        layer = layer or self._tile_var.get() or "Satellite"
        return self._MAP_TILE_PROVIDERS.get(layer, self._MAP_TILE_PROVIDERS["Satellite"])

    def _map_zoom_cap_options(self, layer: str | None = None) -> list[str]:
        cfg = self._map_provider_config(layer)
        return [str(v) for v in range(cfg["min_cap"], cfg["max_cap"] + 1)]

    def _map_zoom_cap(self, layer: str | None = None) -> int:
        cfg = self._map_provider_config(layer)
        try:
            requested = int(self._map_zoom_cap_var.get())
        except (TypeError, ValueError):
            requested = cfg["default_cap"]
        return max(cfg["min_cap"], min(cfg["max_cap"], requested))

    def _map_sync_zoom_cap_ui(self, layer: str | None = None):
        cfg = self._map_provider_config(layer)
        options = self._map_zoom_cap_options(layer)
        if hasattr(self, "_map_zoom_cap_spin"):
            self._map_zoom_cap_spin.configure(values=options)
        try:
            current = int(self._map_zoom_cap_var.get())
        except (TypeError, ValueError):
            current = cfg["default_cap"]
        low = int(options[0])
        high = int(options[-1])
        self._map_zoom_cap_var.set(str(max(low, min(high, current))))

    def _map_sync_zoom_ui(self, zoom: int | float | None = None):
        if not hasattr(self, "_map_zoom_var"):
            return
        if zoom is None:
            if self._map_widget is not None:
                zoom = getattr(self._map_widget, "zoom", self._map_zoom_cap())
            else:
                zoom = self._map_zoom_cap()
        try:
            value = int(round(float(zoom)))
        except (TypeError, ValueError):
            value = self._map_zoom_cap()
        self._map_zoom_var.set(str(max(1, min(self._map_zoom_cap(), value))))

    def _map_apply_tile_server(self, preserve_view: bool = True):
        if self._map_widget is None:
            return
        layer = self._tile_var.get() or "Satellite"
        self._map_sync_zoom_cap_ui(layer)
        cap = self._map_zoom_cap(layer)
        tile_server = self._map_provider_config(layer)["url"]

        if preserve_view:
            pos = self._map_widget.get_position()
            zoom = min(getattr(self._map_widget, "zoom", cap), cap)
        else:
            zoom = min(getattr(self._map_widget, "zoom", cap), cap)

        try:
            self._map_widget.set_tile_server(tile_server, max_zoom=cap)
        except Exception as exc:
            self.log(f"Map provider '{layer}' failed: {exc}")
            return

        if preserve_view:
            self._map_widget.set_position(*pos)
            self._map_widget.set_zoom(zoom)
        self._map_sync_zoom_ui(zoom)

    def _map_set_zoom(self, zoom: int | float):
        if self._map_widget is None:
            return
        try:
            target = int(round(float(zoom)))
        except (TypeError, ValueError):
            target = self._map_zoom_cap()
        target = max(1, min(target, self._map_zoom_cap()))
        self._map_widget.set_zoom(target)
        self._map_sync_zoom_ui(target)

    def _on_map_zoom_change(self, _event=None):
        self._map_set_zoom(self._map_zoom_var.get())

    def _on_map_zoom_cap_change(self, _event=None):
        self._map_apply_tile_server(preserve_view=True)
        self._map_sync_zoom_ui()

    def _map_zoom_to_cap(self):
        self._map_set_zoom(self._map_zoom_cap())

    def _on_tile_change(self, _event=None):
        if self._map_widget is None:
            return
        self._map_apply_tile_server(preserve_view=True)

    def _map_goto(self):
        """Centre the map on the coordinates typed in the 'Go to' entry.
        Accepts 'lat, lon' or 'lat lon'."""
        if self._map_widget is None:
            return
        raw = self._map_goto_var.get().strip()
        if not raw:
            return
        parts = re.split(r"[,\s]+", raw)
        try:
            lat, lon = float(parts[0]), float(parts[1])
            self._map_widget.set_position(lat, lon)
            self._map_set_zoom(17)
        except (ValueError, IndexError):
            self.log(f"Invalid coordinates: {raw}")

    def _map_clear_markers(self):
        if self._map_widget is None:
            return
        for marker in self._map_markers.values():
            marker.delete()
        self._map_markers.clear()
        # Re-add ground truth markers
        for bssid, (lat, lon, alt) in self._ground_truth_positions.items():
            self._map_add_gt_marker(bssid, lat, lon, alt)

    def _map_add_gt_marker(self, bssid: str, lat: float, lon: float, alt: float):
        """Add (or update) a red ground-truth marker on the map.

        GT markers are deliberately styled very differently from estimate
        markers so the two cannot be confused visually:
          * RED colour (#c62828) for both dot and outline
          * Text prefix "🎯 GT — " plus a reference line with "TRUE"
        """
        if self._map_widget is None:
            return
        key = ("gt", bssid)
        label = f"🎯 GT — {bssid}\nTRUE position (alt={alt:.1f} m)"
        if key in self._map_markers:
            self._map_markers[key].set_position(lat, lon)
            self._map_markers[key].set_text(label)
        else:
            m = self._map_widget.set_marker(
                lat, lon, text=label,
                marker_color_circle="#c62828",
                marker_color_outside="#ffffff",
                text_color="#c62828",
                font=("TkDefaultFont", 10, "bold"),
            )
            self._map_markers[key] = m
        # Centre map on GT if first marker
        if len(self._map_markers) == 1:
            self._map_widget.set_position(lat, lon)
            self._map_set_zoom(19)

    def _autoload_ground_truth(self):
        """Populate ground-truth positions from the configured user data directory.

        Idempotent: called on startup so the Map tab has GT pins to compare
        against estimates.
        """
        gt_path = GROUND_TRUTH_CSV
        if not gt_path.exists():
            return
        try:
            with gt_path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.reader(
                    line for line in handle if not line.lstrip().startswith("#")
                ))
        except Exception as exc:
            self.log(f"Could not auto-load GT from {gt_path}: {exc}")
            return
        for row in rows:
            if len(row) < 3:
                continue
            try:
                bssid = row[0].strip().lower()
                lat = float(row[1]); lon = float(row[2])
                alt = float(row[3]) if len(row) > 3 and row[3].strip() else 0.0
            except (ValueError, TypeError):
                continue
            if not bssid or bssid == "bssid":
                continue
            self._ground_truth_positions[bssid] = (lat, lon, alt)
            self._map_add_gt_marker(bssid, lat, lon, alt)
        self.log(
            f"Auto-loaded {len(self._ground_truth_positions)} GT anchor(s) "
            f"from {gt_path}.")

    def _map_update_markers(self, rows: list[tuple], force_center: bool = False):
        """Update map markers from estimate rows.

        Each row: (source, bssid, lat_str, lon_str, alt_str, sigma_h, n_acc, updated, tag)
        The tag is one of ``"fusion"`` (Multi-OSA best online estimate)
        or ``"phone"`` (per-drone estimate). Each phone namespace gets its
        own colour from the trail palette so the Map tab can show the
        fusion estimate alongside each drone's own estimate without
        visual confusion. GT markers are never removed here.
        If *force_center* is True, always pan to the first valid estimate.
        """
        if self._map_widget is None:
            return

        current_keys: set[tuple[str, ...]] = set()
        first_pos = None

        for r in rows:
            source, bssid = r[0], r[1]
            lat_s, lon_s = r[2], r[3]
            if not lat_s or not lon_s:
                continue
            try:
                lat, lon = float(lat_s), float(lon_s)
            except ValueError:
                continue
            if lat == 0.0 and lon == 0.0:
                continue

            tag = r[8] if len(r) > 8 else "phone"
            sigma = r[5] if len(r) > 5 else ""
            n_acc = r[6] if len(r) > 6 else ""

            # Resolve namespace and colour.
            # Expected source formats:
            #   "OSA: <friendly>"           → per-drone estimate
            #   "OSA: <friendly> (in progress)"
            #   "Multi-OSA"                 → best online fusion
            #   "Multi-OSA (Nd)"            → fusion with N contributing drones
            if tag == "fusion":
                colour = "#2e7d32"   # fusion = green
                key: tuple[str, ...] = ("fusion", bssid)
                src_short = "FUSION"
            else:
                ns_key = r[9] if len(r) > 9 and r[9] else source.replace("OSA: ", "").strip()
                ns_key = str(ns_key).replace(" (in progress)", "")
                colour = self._color_for_ns(ns_key)
                key = ("phone", ns_key, bssid)
                src_short = f"OSA[{ns_key}]"

            current_keys.add(key)

            label = f"⭐ {src_short}\n{bssid}"
            if sigma:
                label += f"\nσ_H={sigma}m"
            if n_acc:
                label += f"  n={n_acc}"

            if key in self._map_markers:
                self._map_markers[key].set_position(lat, lon)
                self._map_markers[key].set_text(label)
            else:
                m = self._map_widget.set_marker(
                    lat, lon, text=label,
                    marker_color_circle=colour,
                    marker_color_outside="#ffffff",
                    text_color=colour,
                )
                self._map_markers[key] = m

            if first_pos is None:
                first_pos = (lat, lon)

        # Remove stale estimate markers but always preserve GT pins and
        # any other non-estimate marker categories.
        managed_prefixes = {"fusion", "phone"}
        stale = {
            k for k in self._map_markers.keys()
            if k and k[0] in managed_prefixes and k not in current_keys
        }
        for key in stale:
            try:
                self._map_markers[key].delete()
            except Exception:
                pass
            del self._map_markers[key]

        # Auto-centre on first AP
        if first_pos:
            cur = self._map_widget.get_position()
            at_default = abs(cur[0] - 40.4168) < 0.01 and abs(cur[1] - (-3.7038)) < 0.01
            if force_center or at_default:
                self._map_widget.set_position(*first_pos)
                self._map_set_zoom(17)

    def _map_refresh_from_estimates(self):
        """Pull estimates and update map markers, centering the view."""
        self._map_force_center = True
        self._poll_estimates_once()

    def _ns_debug_label(self, ns: str) -> str:
        friendly = KNOWN_UAVS.get(ns, ns)
        if friendly != ns:
            return f"{friendly} [{ns}]"
        return ns

    def _live_row_source_key(self, row: tuple) -> str:
        if len(row) > 9 and row[9]:
            return str(row[9])
        tag = row[8] if len(row) > 8 else ""
        if tag == "fusion":
            return "fusion"
        source = row[0].replace("OSA: ", "").replace(" (in progress)", "").strip()
        return source or "unknown"

    def _is_live_source_visible(self, source_key: str) -> bool:
        var = self._live_source_vars.get(source_key)
        return True if var is None else bool(var.get())

    def _rebuild_source_filter_panel(
        self,
        host: ttk.Frame,
        specs: list[tuple[str, str]],
        vars_dict: dict[str, tk.BooleanVar],
        command,
        empty_text: str,
        summary_var: tk.StringVar,
        columns: int = 3,
    ):
        for child in host.winfo_children():
            child.destroy()
        if not specs:
            ttk.Label(host, text=empty_text, foreground="gray").grid(
                row=0, column=0, sticky="w")
            summary_var.set(empty_text)
            return

        visible = 0
        for idx, (key, label) in enumerate(specs):
            if key not in vars_dict:
                vars_dict[key] = tk.BooleanVar(value=True)
            if vars_dict[key].get():
                visible += 1
            cb = ttk.Checkbutton(
                host, text=label, variable=vars_dict[key], command=command,
            )
            cb.grid(row=idx // columns, column=idx % columns,
                    sticky="w", padx=(0, 12), pady=2)
        for col in range(columns):
            host.columnconfigure(col, weight=1)
        summary_var.set(f"{visible}/{len(specs)} visible")

    def _sync_live_source_controls(self, rows: list[tuple]):
        specs: list[tuple[str, str]] = []
        seen: set[str] = set()
        for row in rows:
            key = self._live_row_source_key(row)
            if key in seen:
                continue
            if key == "fusion":
                label = "Multi-OSA (online)"
            else:
                label = f"OSA: {self._ns_debug_label(key)}"
            specs.append((key, label))
            seen.add(key)
        if hasattr(self, "_live_sources_checks"):
            self._rebuild_source_filter_panel(
                self._live_sources_checks,
                specs,
                self._live_source_vars,
                self._on_live_source_visibility_change,
                empty_text="No live sources yet.",
                summary_var=self._live_sources_summary,
            )

    def _set_live_source_visibility(self, visible: bool):
        for var in self._live_source_vars.values():
            var.set(visible)
        self._on_live_source_visibility_change()

    def _on_live_source_visibility_change(self):
        self._refresh_trail_visibility()
        self._apply_live_source_filters()

    def _apply_live_source_filters(self, now_str: str | None = None):
        rows = list(self._live_estimate_rows)
        if hasattr(self, "_live_sources_checks"):
            self._sync_live_source_controls(rows)
        filtered = [
            row for row in rows
            if self._is_live_source_visible(self._live_row_source_key(row))
        ]

        if not hasattr(self, "est_tree"):
            return
        self.est_tree.delete(*self.est_tree.get_children())
        for r in filtered:
            tag = r[8] if len(r) > 8 else ""
            err_str = ""
            bssid_key = r[1].strip().lower()
            gt_pos = self._ground_truth_positions.get(bssid_key)
            if gt_pos and r[2] and r[3]:
                try:
                    est_lat, est_lon = float(r[2]), float(r[3])
                    gt_lat, gt_lon = gt_pos[0], gt_pos[1]
                    err_m = _haversine_m(est_lat, est_lon, gt_lat, gt_lon)
                    err_str = f"{err_m:.1f}"
                except ValueError:
                    pass
            vals = r[:7] + (err_str, r[7])
            tags = [tag]
            if err_str:
                tags.append("good_err" if float(err_str) < 15 else "warn_err")
            self.est_tree.insert("", "end", values=vals, tags=tuple(tags))

        stamp = now_str or datetime.now().strftime("%H:%M:%S")
        self.est_status_var.set(f"{len(filtered)}/{len(rows)} estimates @ {stamp}")
        force = getattr(self, "_map_force_center", False)
        self._map_force_center = False
        self._map_update_markers(filtered, force_center=force)

    def _refresh_trail_visibility(self):
        for ns in list(self._osa_trails):
            self._redraw_trail(ns)

    def _build_logs_tab(self, nb: ttk.Notebook):
        logs = self._new_tab_with_guide(nb, "Logs", GUIDE_LOGS, padding=8)

        ttk.Label(logs, text="Log Management", style="Header.TLabel").pack(anchor="w")

        # ADB device selector
        dev_frame = ttk.Frame(logs)
        dev_frame.pack(fill="x", pady=4)
        ttk.Label(dev_frame, text="ADB device:").pack(side="left")
        self.logs_serial_var = tk.StringVar(value="(any)")
        self.logs_serial_combo = ttk.Combobox(
            dev_frame, textvariable=self.logs_serial_var, state="readonly", width=28,
            values=["(any)"],
        )
        self.logs_serial_combo.pack(side="left", padx=4)
        self._all_serial_combos.append(self.logs_serial_combo)
        ttk.Button(dev_frame, text="Refresh", command=self._refresh_adb_serials).pack(
            side="left"
        )

        self.log_dest = tk.StringVar(value=str(Path.home() / "ftm_logs"))
        dest_frame = ttk.Frame(logs)
        dest_frame.pack(fill="x", pady=4)
        ttk.Label(dest_frame, text="Destination:").pack(side="left")
        ttk.Entry(dest_frame, textvariable=self.log_dest, width=40).pack(
            side="left", padx=4
        )
        ttk.Button(dest_frame, text="Browse…", command=self.browse_dest).pack(
            side="left"
        )

        btn_logs = ttk.Frame(logs)
        btn_logs.pack(fill="x", pady=4)
        ttk.Button(btn_logs, text="Pull logs", command=self.pull_logs).pack(
            side="left"
        )
        ttk.Button(btn_logs, text="Pull from ALL devices", command=self.pull_logs_all).pack(
            side="left", padx=8
        )
        ttk.Button(btn_logs, text="List remote logs", command=self.list_logs).pack(
            side="left", padx=8
        )
        ttk.Button(
            btn_logs, text="Delete remote logs", command=self.delete_logs
        ).pack(side="left", padx=8)

    # ──────────────────────────────────────────────────────────────
    #  Helpers
    # ──────────────────────────────────────────────────────────────

    def _copy_output(self):
        """Copy selected text (or everything) from the output widget."""
        try:
            text = self.output.get("sel.first", "sel.last")
        except tk.TclError:
            text = self.output.get("1.0", "end-1c")
        self.clipboard_clear()
        self.clipboard_append(text)

    def _toggle_output(self):
        """Show/hide the output text panel."""
        if self._output_visible:
            self._output_frame.pack_forget()
            self._output_toggle_btn.configure(text="▶ Show")
            self._output_visible = False
        else:
            self._output_frame.pack(fill="both", expand=True)
            self._output_toggle_btn.configure(text="▼ Hide")
            self._output_visible = True

    def _labeled_entry(self, parent, label, row, default="", show="", tooltip=""):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=2)
        var = tk.StringVar(value=default)
        kw: dict = {"textvariable": var, "width": 40}
        if show:
            kw["show"] = show
        entry = ttk.Entry(parent, **kw)
        entry.grid(row=row, column=1, sticky="ew", pady=2)
        parent.columnconfigure(1, weight=1)
        if tooltip:
            ToolTip(entry, tooltip)
        return var

    def log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S")
        self.output.insert("end", f"[{ts}] {msg}\n")
        self.output.see("end")
        self.status_var.set(msg[:120])

    def _thread_log(self, msg: str):
        """Log from either the Tk main thread or a worker thread."""
        if threading.current_thread() is threading.main_thread():
            self.log(msg)
        else:
            self.after(0, lambda m=msg: self.log(m))

    def _run_in_thread(self, fn, *args):
        """Run fn(*args) in a background thread to keep UI responsive."""
        def wrapper():
            try:
                fn(*args)
            except Exception as e:
                self.after(0, lambda: self.log(f"ERROR: {e}"))
        threading.Thread(target=wrapper, daemon=True).start()

    def _set_check(self, key: str, ok: bool):
        """Update a preflight checklist item (call from main thread or via after)."""
        var = self._check_vars.get(key)
        if var:
            var.set("✅" if ok else "⬜")

    def _adb_serial(self) -> str | None:
        """Return the selected ADB serial, or None for 'any'."""
        v = self.adb_serial_var.get()
        return None if v == "(any)" else v

    def _serials_from_selection(self, selected: str | None) -> list[str]:
        """Return selected serial, or every connected serial for '(any)'."""
        if selected:
            return [selected]
        serials = self._get_all_adb_serials()
        if not serials:
            self._thread_log("No ADB devices connected.")
        return serials

    def _single_serial_from_selection(self, selected: str | None) -> str | None:
        """Return one serial for readback operations that cannot target many."""
        if selected:
            return selected
        serials = self._get_all_adb_serials()
        if not serials:
            self._thread_log("No ADB devices connected.")
            return None
        if len(serials) > 1:
            self._thread_log(f"Select one ADB device first; connected: {', '.join(serials)}")
            return None
        return serials[0]

    def _broadcast_to_serials(self, action: str, extras: dict, serials: list[str]) -> bool:
        """Broadcast to explicit serials and log every result."""
        ok_any = False
        for s in serials:
            self._thread_log(f"ADB {action} → {s}: {extras}")
            result = broadcast(action, extras, serial=s)
            self._thread_log(f"[{s}] {result}")
            ok_any = ok_any or ("Broadcast completed" in result and not _adb_failed(result))
        return ok_any

    def _set_all_device_status(self, running: bool, ok: bool = True):
        """Update discovered ROS-device badges after an ADB bulk command."""
        for dev in self.devices.values():
            dev["var"].set("RUNNING" if running and ok else "STOPPED" if ok else "ERROR")
            dev["label"].configure(
                style="Running.TLabel" if running and ok else "Stopped.TLabel"
            )
            dev["running"] = running and ok

    def _broadcast_to_selection(self, action: str, extras: dict, selected: str | None) -> bool:
        """Broadcast to the selected phone, or to all phones when '(any)' is selected."""
        serials = self._serials_from_selection(selected)
        if not serials:
            return False
        return self._broadcast_to_serials(action, extras, serials)

    # ──────────────────────────────────────────────────────────────
    #  Mission Control actions
    # ──────────────────────────────────────────────────────────────

    def discover_uavs(self):
        """Discover ROS 2 namespaces, waking ADB-connected phones first."""
        self.log("Discovering UAVs via ROS 2 + ADB …")

        def _discover():
            # If phones are reachable through ADB-over-TCP, make sure the app is
            # foregrounded before ROS discovery. This starts the rcljava node on
            # phones that are connected by ADB but not yet visible in DDS.
            serials = self._get_all_adb_serials()
            for serial in serials:
                self._thread_log(f"Preparing ROS discovery on ADB phone: {serial}")
                result = adb_start_app(serial=serial)
                if result.strip():
                    self._thread_log(f"[{serial}] start app → {result}")
            if serials:
                import time
                time.sleep(1.0)
            namespaces = ros2_discover_namespaces()
            self.after(0, lambda: self._populate_devices(namespaces))

        self._run_in_thread(_discover)

    def _populate_devices(self, namespaces: list[str]):
        # Clear old widgets
        for w in self.device_frame.winfo_children():
            w.destroy()
        self.devices.clear()

        ros_namespaces = list(dict.fromkeys(namespaces))
        adb_meta = self._adb_devices_by_namespace(ros_namespaces)
        all_namespaces = ros_namespaces + [
            ns for ns in adb_meta.keys() if ns not in set(ros_namespaces)
        ]

        if not all_namespaces:
            ttk.Label(
                self.device_frame,
                text="No UAVs found via ROS 2 or ADB. Is ROS 2 running, or are phones connected by ADB?",
                foreground="red",
            ).pack(pady=20)
            self.log("No UAVs discovered via ROS 2 or ADB.")
            return

        vpn_ips_by_ns = {
            ns: meta.get("vpn_ip", "")
            for ns, meta in adb_meta.items()
            if meta.get("vpn_ip")
        }
        vpn_ips_by_ns.update({
            ns: ip for ns, ip in self._vpn_ips_by_namespace(ros_namespaces).items()
            if ns not in vpn_ips_by_ns
        })
        configured_vpn_ips = self._configured_vpn_ips()

        for ns in all_namespaces:
            friendly = KNOWN_UAVS.get(ns, ns)
            vpn_ip = vpn_ips_by_ns.get(ns, "")
            meta = adb_meta.get(ns, {})
            adb_serial = meta.get("serial", "")
            ros_visible = ns in ros_namespaces
            source = "ROS + ADB" if ros_visible and adb_serial else "ROS" if ros_visible else "ADB only"
            frame = ttk.LabelFrame(
                self.device_frame, text=f"{friendly}  [{ns}]", padding=6
            )
            frame.pack(fill="x", pady=4, padx=4)

            # Top row: status + buttons
            top = ttk.Frame(frame)
            top.pack(fill="x")

            status_var = tk.StringVar(value="IDLE")
            status_lbl = ttk.Label(top, textvariable=status_var, style="Stopped.TLabel")
            status_lbl.pack(side="left", padx=(0, 12))

            ip_text = f"VPN IP: {vpn_ip}" if vpn_ip else "VPN IP: unknown"
            if not vpn_ip and configured_vpn_ips:
                ip_text += f" (configured: {', '.join(configured_vpn_ips)})"
            ttk.Label(top, text=ip_text, foreground="gray").pack(side="left", padx=(0, 12))

            source_text = f"{source}"
            if adb_serial:
                source_text += f" · ADB: {adb_serial}"
            ttk.Label(top, text=source_text, foreground="#666666").pack(side="left", padx=(0, 12))

            ttk.Button(
                top, text="START",
                command=lambda n=ns: self._run_in_thread(self._start_one, n),
            ).pack(side="left", padx=2)

            ttk.Button(
                top, text="STOP",
                command=lambda n=ns: self._run_in_thread(self._stop_one, n),
            ).pack(side="left", padx=2)

            ttk.Button(
                top, text="Echo location",
                command=lambda n=ns: self._run_in_thread(self._echo_location, n),
            ).pack(side="left", padx=8)

            # Telemetry badges (updated by background poller)
            tele = ttk.Frame(frame)
            tele.pack(fill="x", pady=(4, 0))
            agent_var = tk.StringVar(value="agent=?")
            coop_var = tk.StringVar(value="mode=?")
            fix_var = tk.StringVar(value="fix=—")
            age_var = tk.StringVar(value="age=—")
            rate_var = tk.StringVar(value="rtt=—/s")
            pos_var = tk.StringVar(value="pos=—")

            agent_lbl = ttk.Label(tele, textvariable=agent_var, style="OsaAgent.TLabel")
            coop_lbl = ttk.Label(tele, textvariable=coop_var, style="OsaTelemetry.TLabel")
            fix_lbl = ttk.Label(tele, textvariable=fix_var, style="OsaTelemetry.TLabel")
            age_lbl = ttk.Label(tele, textvariable=age_var, style="OsaTelemetry.TLabel")
            rate_lbl = ttk.Label(tele, textvariable=rate_var, style="OsaTelemetry.TLabel")
            pos_lbl = ttk.Label(tele, textvariable=pos_var, style="OsaTelemetry.TLabel")

            for lbl in (agent_lbl, coop_lbl, fix_lbl, age_lbl, rate_lbl, pos_lbl):
                lbl.pack(side="left", padx=6)

            self.devices[ns] = {
                "var": status_var,
                "label": status_lbl,
                "running": False,
                "adb_serial": adb_serial,
                "ros_visible": ros_visible,
                # telemetry vars
                "agent_var": agent_var,
                "coop_var": coop_var,
                "fix_var": fix_var,
                "age_var": age_var,
                "rate_var": rate_var,
                "pos_var": pos_var,
                "fix_lbl": fix_lbl,
                "age_lbl": age_lbl,
                "rate_lbl": rate_lbl,
            }

        self.log(f"Discovered {len(all_namespaces)} UAV(s): {', '.join(all_namespaces)}")
        if adb_meta:
            self.log("ADB-backed UAVs: " + ", ".join(
                f"{ns}→{meta.get('serial', '?')}" for ns, meta in sorted(adb_meta.items())))
        if vpn_ips_by_ns:
            self.log("VPN IP mapping: " + ", ".join(
                f"{ns}={ip}" for ns, ip in sorted(vpn_ips_by_ns.items())))
        elif configured_vpn_ips:
            self.log("VPN IP mapping unknown; configured IPs: " + ", ".join(configured_vpn_ips))
        self._set_check("uavs", True)

        # Start the background telemetry poller (idempotent)
        self._start_telemetry_poller()

    def _adb_devices_by_namespace(self, ros_namespaces: list[str]) -> dict[str, dict[str, str]]:
        """Return ADB-reachable phones keyed by their best ROS namespace guess.

        ROS discovery can miss Android phones even when ADB-over-TCP is healthy.
        Merging ADB-reachable devices into the Mission Control list keeps every
        connected phone visible and controllable, while matching known namespace
        candidates prevents duplicate ROS/ADB cards for the same phone.
        """
        meta: dict[str, dict[str, str]] = {}
        used: set[str] = set()
        ros_set = set(ros_namespaces)
        self._adb_status_cache = {}
        for serial in self._get_all_adb_serials():
            status = self._adb_status_for_serial(serial, refresh=True)
            candidates = sorted(self._adb_namespace_candidates(serial))
            ns = next((c for c in candidates if c in ros_set and c not in used), "")
            if not ns:
                ns = next((c for c in candidates if c not in used), "")
            if not ns:
                base = self._sanitize_namespace_part(serial.rsplit(":", 1)[0].replace(".", "_"))
                ns = f"adb_{base or 'phone'}"
            if ns in used:
                suffix = self._sanitize_namespace_part(serial)[-8:] or str(len(used) + 1)
                ns = f"{ns}_{suffix}"
            used.add(ns)
            ips = self._vpn_ips_for_serial(serial)
            meta[ns] = {
                "serial": serial,
                "vpn_ip": ips[0] if ips else "",
                "ros_namespace": str(status.get("ros_namespace", "")),
                "agent_id": str(status.get("agent_id", "")),
            }
        return meta

    def _adb_status_for_serial(self, serial: str, refresh: bool = False) -> dict:
        """Return phone status.json via ADB when available.

        Newer app builds include the active ROS namespace and VPN IPs in this
        file. Older builds simply return fewer keys; callers still fall back to
        SharedPreferences and network-interface probing.
        """
        cache = getattr(self, "_adb_status_cache", {})
        if serial in cache and not refresh:
            return cache[serial]
        if refresh:
            result = broadcast("REMOTE_CMD", {"cmd": "get_status"}, serial=serial)
            if "Broadcast completed" not in result:
                self._thread_log(f"[{serial}] get_status broadcast failed: {result}")
                cache[serial] = {}
                self._adb_status_cache = cache
                return {}
            import time
            time.sleep(0.35)
        elif serial not in cache:
            return {}

        safe = self._sanitize_namespace_part(serial) or "phone"
        local = Path(tempfile.gettempdir()) / f"rtt_phone_status_{safe}.json"
        remote = f"{DATA_DIR}/status.json"
        pull_result = adb(
            f"pull {shlex.quote(remote)} {shlex.quote(str(local))}",
            serial=serial,
            timeout=6,
        )
        if "error" in pull_result.lower() or not local.exists():
            self._thread_log(f"[{serial}] status pull failed: {pull_result}")
            cache[serial] = {}
            self._adb_status_cache = cache
            return {}
        try:
            data = json.loads(local.read_text())
            if not isinstance(data, dict):
                data = {}
        except (json.JSONDecodeError, OSError) as exc:
            self._thread_log(f"[{serial}] status parse failed: {exc}")
            data = {}
        cache[serial] = data
        self._adb_status_cache = cache
        return data

    def _configured_vpn_ips(self) -> list[str]:
        """Return VPN IPs configured in the ADB-over-network box."""
        ips: list[str] = []
        seen: set[str] = set()
        for target in self._adb_tcp_targets():
            ip = target.rsplit(":", 1)[0]
            if ip and ip not in seen:
                seen.add(ip)
                ips.append(ip)
        return ips

    @staticmethod
    def _shared_pref_string(xml: str, key: str) -> str:
        m = re.search(rf'<string\s+name="{re.escape(key)}">([^<]*)</string>', xml or "")
        return m.group(1).strip() if m else ""

    @staticmethod
    def _sanitize_namespace_part(value: str) -> str:
        return re.sub(r"[^a-z0-9_]", "_", (value or "").lower()).strip("_")

    def _adb_namespace_candidates(self, serial: str) -> set[str]:
        """Best-effort ROS namespace candidates for an ADB serial."""
        candidates: set[str] = set()
        status = self._adb_status_for_serial(serial, refresh=False)
        for key in ("ros_namespace", "agent_id"):
            value = self._sanitize_namespace_part(str(status.get(key, "")).lstrip("/"))
            if value:
                candidates.add(value[:32])

        exp_xml = adb(
            f"shell run-as {PKG} cat shared_prefs/experiment_config.xml",
            serial=serial,
            timeout=4,
        )
        agent_id = self._sanitize_namespace_part(self._shared_pref_string(exp_xml, "agent_id"))
        if agent_id:
            candidates.add(agent_id[:32])

        rtt_xml = adb(
            f"shell run-as {PKG} cat shared_prefs/rtt_prefs.xml",
            serial=serial,
            timeout=4,
        )
        device_id = self._shared_pref_string(rtt_xml, "device_id_v1")
        model = adb("shell getprop ro.product.model", serial=serial, timeout=4).splitlines()
        model_core = self._sanitize_namespace_part(model[0] if model else "") or "phone"
        if device_id:
            candidates.add(f"{model_core}_{device_id.strip()[:8]}")
        return candidates

    def _vpn_ips_for_serial(self, serial: str) -> list[str]:
        """Return ZeroTier/VPN IPv4 addresses observed on a phone."""
        ips: list[str] = []
        status = self._adb_status_for_serial(serial, refresh=False)
        status_ips = status.get("vpn_ips", []) or status.get("local_vpn_ips", [])
        if isinstance(status_ips, str):
            status_ips = [status_ips]
        if isinstance(status_ips, list):
            for ip in status_ips:
                ip_s = str(ip).strip()
                if re.match(r"^10\.148\.\d+\.\d+$", ip_s) and ip_s not in ips:
                    ips.append(ip_s)

        out = adb("shell ip -o -4 addr show", serial=serial, timeout=4)
        for ip in re.findall(r"\binet\s+(10\.148\.\d+\.\d+)/", out):
            if ip not in ips:
                ips.append(ip)
        if not ips and ":" in serial:
            ips.append(serial.rsplit(":", 1)[0])
        return ips

    def _vpn_ips_by_namespace(self, namespaces: list[str]) -> dict[str, str]:
        """Best-effort mapping from ROS namespace to phone VPN IP."""
        ns_set = set(namespaces)
        mapping: dict[str, str] = {}
        serials = self._get_all_adb_serials()
        for serial in serials:
            ips = self._vpn_ips_for_serial(serial)
            if not ips:
                continue
            candidates = self._adb_namespace_candidates(serial)
            matched = [ns for ns in ns_set if ns in candidates]
            if len(matched) == 1:
                mapping[matched[0]] = ips[0]
        if not mapping and len(namespaces) == len(serials):
            # Last resort: stable display-only pairing when ADB has the same
            # number of phones as ROS controllable namespaces.
            serial_ips: list[str] = []
            for serial in serials:
                ips = self._vpn_ips_for_serial(serial)
                if ips:
                    serial_ips.append(ips[0])
            if len(serial_ips) == len(namespaces):
                mapping.update(dict(zip(sorted(namespaces), sorted(serial_ips))))
        return mapping

    def _start_one(self, ns: str):
        self.after(0, lambda: self.log(f"Starting experiment on {ns} …"))
        dev = self.devices.get(ns, {})
        mapped_serial = dev.get("adb_serial") if isinstance(dev, dict) else ""
        if mapped_serial and adb_serial_usable(mapped_serial):
            # Primary for phones: ADB, because ROS bidirectionality can fail across RMWs.
            self.after(0, lambda: self.log(
                f"[{ns}] using ADB START on mapped phone: {mapped_serial}"))
            is_ok = self._broadcast_to_serials("REMOTE_CMD", {"cmd": "start"}, [mapped_serial])
            result = "ADB broadcast completed" if is_ok else "ADB broadcast failed"
        else:
            # Fallback when no phone is reachable by ADB.
            result = ros2_topic_cmd(ns, "start")
            is_ok = "ERROR" not in result
        if not is_ok:
            # Last resort: ROS 2 service call
            self.after(0, lambda: self.log(f"[{ns}] ADB/ROS topic failed, trying service call…"))
            result = ros2_call(ns, "ftm/start_experiment")
            is_ok = "success" in result.lower() or "true" in result.lower()
        def _update():
            self.log(f"[{ns}] start → {'OK' if is_ok else result}")
            if ns in self.devices:
                self.devices[ns]["var"].set("RUNNING" if is_ok else "ERROR")
                self.devices[ns]["label"].configure(
                    style="Running.TLabel" if is_ok else "Stopped.TLabel"
                )
                self.devices[ns]["running"] = is_ok
        self.after(0, _update)

    def _stop_one(self, ns: str):
        self.after(0, lambda: self.log(f"Stopping experiment on {ns} …"))
        dev = self.devices.get(ns, {})
        mapped_serial = dev.get("adb_serial") if isinstance(dev, dict) else ""
        if mapped_serial and adb_serial_usable(mapped_serial):
            # Primary for phones: ADB, because ROS bidirectionality can fail across RMWs.
            self.after(0, lambda: self.log(
                f"[{ns}] using ADB STOP on mapped phone: {mapped_serial}"))
            is_ok = self._broadcast_to_serials("REMOTE_CMD", {"cmd": "stop"}, [mapped_serial])
            result = "ADB broadcast completed" if is_ok else "ADB broadcast failed"
        else:
            # Fallback when no phone is reachable by ADB.
            result = ros2_topic_cmd(ns, "stop")
            is_ok = "ERROR" not in result
        if not is_ok:
            # Last resort: ROS 2 service call
            self.after(0, lambda: self.log(f"[{ns}] ADB/ROS topic failed, trying service call…"))
            result = ros2_call(ns, "ftm/stop_experiment")
            is_ok = "success" in result.lower() or "true" in result.lower()
        def _update():
            self.log(f"[{ns}] stop → {'OK' if is_ok else result}")
            if ns in self.devices:
                self.devices[ns]["var"].set("STOPPED")
                self.devices[ns]["label"].configure(style="Stopped.TLabel")
                self.devices[ns]["running"] = False
        self.after(0, _update)

    def _echo_location(self, ns: str):
        topic = f"/{ns}/phone/location"
        self.after(0, lambda: self.log(f"Listening on {topic} …"))
        result = ros2_echo_once(topic, timeout=20)
        self.after(0, lambda: self.log(f"[{ns}] location: {result}"))

    def start_all_experiments(self):
        serials = self._get_all_adb_serials()
        if serials:
            self.log("Using ADB START on all connected phones (ROS/RMW independent).")
            self._run_in_thread(self._adb_remote_all, "start")
            return
        if not self.devices:
            self.log("No ADB phones or ROS UAVs discovered.")
            return
        for ns in self.devices:
            self._run_in_thread(self._start_one, ns)

    def stop_all_experiments(self):
        serials = self._get_all_adb_serials()
        if serials:
            self.log("Using ADB STOP on all connected phones (ROS/RMW independent).")
            self._run_in_thread(self._adb_remote_all, "stop")
            return
        if not self.devices:
            self.log("No ADB phones or ROS UAVs discovered.")
            return
        for ns in self.devices:
            self._run_in_thread(self._stop_one, ns)

    # ──────────────────────────────────────────────────────────────
    #  ADB actions
    # ──────────────────────────────────────────────────────────────

    def check_device(self):
        self.log("$ adb devices")
        self.log(adb("devices"))
        self._refresh_adb_serials()

    def _refresh_adb_serials(self):
        """Populate ALL ADB serial comboboxes from connected devices."""
        serials = ["(any)"] + self._get_all_adb_serials()
        for combo in self._all_serial_combos:
            current = combo.get()
            combo["values"] = serials
            if current in serials:
                combo.set(current)
            elif len(serials) == 2:
                combo.set(serials[1])
            else:
                combo.set("(any)")
        msg = ", ".join(serials[1:]) if len(serials) > 1 else "none"
        if hasattr(self, "adb_devices_var"):
            self.adb_devices_var.set(f"Connected: {msg}")
        self.log(f"ADB devices: {serials[1:] if len(serials) > 1 else 'none'}")
        self._set_check("adb", len(serials) > 1)

    def _get_all_adb_serials(self) -> list[str]:
        """Return connected ADB serials that really accept shell commands.

        ``adb devices`` can keep stale TCP transports as ``device`` after a VPN
        flap.  Those entries make the GUI think ADB is alive, but every button
        then hangs or times out.  Probe each serial with a short shell command
        and disconnect stale TCP transports.
        """
        out = adb("devices")
        serials = []
        for line in out.strip().splitlines():
            parts = line.split()
            if len(parts) >= 2 and parts[1] == "device":
                serial = parts[0]
                if adb_serial_usable(serial):
                    serials.append(serial)
                else:
                    self._thread_log(f"ADB stale/unreachable: {serial}")
                    if ":" in serial:
                        adb(f"disconnect {serial}", timeout=5)
        if not serials:
            for target in self._adb_tcp_targets():
                result = adb(f"connect {target}", timeout=8)
                if "connected" in result.lower() and adb_serial_usable(target):
                    self._thread_log(f"ADB reconnected: {target}")
                    serials.append(target)
                else:
                    hint = self._adb_connect_hint(result)
                    self._thread_log(f"ADB reconnect failed for {target}: {result}")
                    if hint:
                        self._thread_log(f"[{target}] {hint}")
                    adb(f"disconnect {target}", timeout=5)
        return serials

    def _get_usb_adb_serials(self) -> list[str]:
        """Return physical USB ADB serials, excluding already-connected TCP targets."""
        return [s for s in self._get_all_adb_serials() if ":" not in s]

    def _adb_start_all_apps(self):
        serials = self._get_all_adb_serials()
        if not serials:
            self.after(0, lambda: self.log("No ADB devices connected."))
            return
        for s in serials:
            result = adb_start_app(serial=s)
            self.after(0, lambda s=s, r=result: self.log(f"[{s}] start app → {r}"))

    def _adb_force_stop_all_apps(self):
        serials = self._get_all_adb_serials()
        if not serials:
            self.after(0, lambda: self.log("No ADB devices connected."))
            return
        for s in serials:
            result = adb_force_stop_app(serial=s)
            self.after(0, lambda s=s, r=result: self.log(f"[{s}] force-stop app → {r}"))

    def _adb_remote_all(self, cmd: str):
        serials = self._get_all_adb_serials()
        if not serials:
            self.after(0, lambda: self.log("No ADB devices connected."))
            return
        ok = self._broadcast_to_serials("REMOTE_CMD", {"cmd": cmd}, serials)
        def _update():
            self.log(f"ADB {cmd.upper()} all → {'OK' if ok else 'no phone acknowledged broadcast'}")
            if cmd == "start":
                self._set_all_device_status(running=True, ok=ok)
            elif cmd == "stop":
                self._set_all_device_status(running=False, ok=ok)
        self.after(0, _update)

    # ── Config persistence ──

    def _load_config(self) -> dict:
        try:
            return json.loads(CONFIG_FILE.read_text())
        except (OSError, json.JSONDecodeError):
            return {}

    def _save_config(self, updates: dict):
        cfg = self._load_config()
        cfg.update(updates)
        CONFIG_FILE.parent.mkdir(parents=True, exist_ok=True)
        CONFIG_FILE.write_text(json.dumps(cfg, indent=2))

    # ── ADB over TCP ──

    def _adb_enable_tcpip(self):
        """Switch phone ADB daemon to TCP mode (requires USB cable)."""
        usb_serials = self._get_usb_adb_serials()
        if not usb_serials:
            self.after(0, lambda: self.log(
                "No USB ADB device found. Connect by cable first, then press Enable TCP mode."))
            return
        self.after(0, lambda: self.log(
            f"Enabling ADB TCP mode on USB device(s): {', '.join(usb_serials)}"))
        for serial in usb_serials:
            result = adb("tcpip 5555", serial=serial)
            self.after(0, lambda s=serial, r=result: self.log(f"adb -s {s} tcpip 5555 → {r}"))

    def _adb_tcp_targets(self) -> list[str]:
        """Return TCP ADB targets from the IP field.

        Accepts one or more values separated by comma/space/semicolon.  Values
        may be plain IPs or full host:port targets.
        """
        raw = self.adb_net_ip.get().strip()
        default_port = self.adb_net_port.get().strip() or "5555"
        targets: list[str] = []
        for token in re.split(r"[,;\s]+", raw):
            token = token.strip()
            if not token:
                continue
            if ":" in token:
                targets.append(token)
            else:
                targets.append(f"{token}:{default_port}")
        return targets

    @staticmethod
    def _adb_connect_hint(result: str) -> str:
        low = (result or "").lower()
        if "connection refused" in low:
            return "Hint: ZeroTier/IP is reachable, but adbd is not listening on 5555. Reconnect USB and press Enable TCP mode."
        if "timed out" in low or "no route" in low or "unreachable" in low:
            return "Hint: check ZeroTier membership/routes and that the phone VPN is still active."
        if "unauthorized" in low:
            return "Hint: unlock the phone and accept the ADB authorization prompt."
        return ""

    def _adb_connect_tcp(self):
        """Connect to one or more phones over ADB TCP/IP (ZeroTier VPN)."""
        targets = self._adb_tcp_targets()
        if not targets:
            self.after(0, lambda: self.log("ADB TCP: enter one or more phone ZeroTier IPs"))
            return
        results: list[tuple[str, str, bool]] = []
        for target in targets:
            self.after(0, lambda t=target: self.log(f"$ adb connect {t}"))
            result = adb(f"connect {target}")
            connected = "connected" in result.lower() and adb_serial_usable(target)
            if not connected:
                adb(f"disconnect {target}", timeout=5)
            results.append((target, result, connected))
        def _update():
            connected_targets = []
            for target, result, connected in results:
                self.log(f"adb connect {target} → {result}")
                hint = self._adb_connect_hint(result)
                if hint:
                    self.log(f"[{target}] {hint}")
                if connected:
                    connected_targets.append(target)
            if connected_targets:
                label = ", ".join(connected_targets)
                self.adb_net_status.set(f"Connected ({label})")
                self._save_config({
                    "adb_ip": ",".join(t.rsplit(":", 1)[0] for t in targets),
                    "adb_port": self.adb_net_port.get().strip() or "5555",
                })
                # Auto-select this TCP serial for commands and refresh the full ADB list.
                self._refresh_adb_serials()
                if len(connected_targets) == 1 and connected_targets[0] in self.adb_serial_combo["values"]:
                    self.adb_serial_var.set(connected_targets[0])
                else:
                    self.adb_serial_var.set("(any)")
                self._set_check("adb", True)
            else:
                self.adb_net_status.set("Connection failed")
                self._set_check("adb", False)
        self.after(0, _update)

    def _adb_disconnect_tcp(self):
        """Disconnect ADB TCP session."""
        targets = self._adb_tcp_targets()
        if targets:
            results = [(target, adb(f"disconnect {target}")) for target in targets]
        else:
            results = [("all", adb("disconnect"))]
        def _update():
            for target, result in results:
                self.log(f"adb disconnect {target} → {result}")
            self.adb_net_status.set("Disconnected")
            self._set_check("adb", False)
            self._refresh_adb_serials()
        self.after(0, _update)

    def _coop_extras(self) -> dict:
        """Return cooperative extras for EXPERIMENT_CONFIG (empty values omitted)."""
        extras: dict = {}
        agent_id = self.coop_agent_id.get().strip()
        peer_id = self.coop_peer_id.get().strip()
        mode = self.coop_mode.get().strip()
        period = self.coop_period.get().strip()
        phase = self.coop_phase.get().strip()
        if agent_id:
            extras["agent_id"] = agent_id
        if peer_id:
            extras["peer_agent_id"] = peer_id
        if mode:
            extras["cooperative_mode"] = mode
        if period:
            extras["ranging_period_ms"] = period
        if phase:
            extras["ranging_phase_ms"] = phase
        return extras

    def _altitude_extras(self) -> dict:
        """Return payload keys for altitude handling.

        altitude_mode is always sent. AGL is included only when mode=manual_agl.
        In auto mode the anchor Z is recovered from drone vertical diversity.
        """
        mode = (self.exp_alt_mode.get() or "auto").strip().lower()
        if mode == "manual_agl":
            return {"altitude_mode": "manual_agl", "agl": self.exp_agl.get()}
        return {"altitude_mode": "auto"}

    def send_experiment(self):
        extras = {
            "label": self.exp_label.get(),
            "range_bias": self.exp_bias.get(),
            "notes": self.exp_notes.get(),
            "filter": self.exp_filter.get(),
        }
        extras.update(self._altitude_extras())
        extras.update(self._coop_extras())
        serial = self._adb_serial()
        ok = self._broadcast_to_selection("EXPERIMENT_CONFIG", extras, serial)
        self._set_check("config", ok)

    def send_experiment_all(self):
        """Send experiment config to ALL connected ADB devices."""
        serials = self._get_all_adb_serials()
        if not serials:
            self.log("No ADB devices connected.")
            return
        base_extras = {
            "label": self.exp_label.get(),
            "range_bias": self.exp_bias.get(),
            "notes": self.exp_notes.get(),
            "filter": self.exp_filter.get(),
        }
        base_extras.update(self._altitude_extras())
        base_extras.update(self._coop_extras())
        ok = self._broadcast_to_serials("EXPERIMENT_CONFIG", base_extras, serials)
        self._set_check("config", ok)

    # ──────────────────────────────────────────────────────────────
    #  N-OSA auto-config wizard
    # ──────────────────────────────────────────────────────────────

    def _compute_n_osa_plan(self) -> list[dict] | None:
        """Build the ring-topology assignment plan. Returns None on invalid input."""
        try:
            n = max(1, int(self.wiz_n.get()))
            period_ms = max(60, int(self.wiz_period.get()))
        except ValueError:
            self.after(0, lambda: self.log("N-OSA wizard: invalid N or period."))
            return None
        prefix = self.wiz_prefix.get().strip() or "osa"
        serials = self._get_all_adb_serials()
        if not serials:
            self.after(0, lambda: self.log("N-OSA wizard: no ADB devices connected."))
            return None
        n_eff = min(n, len(serials))
        agent_ids = [f"{prefix}{i + 1}" for i in range(n_eff)]
        plan: list[dict] = []
        for i, serial in enumerate(serials[:n_eff]):
            agent_id = agent_ids[i]
            peer_id = agent_ids[(i + 1) % n_eff] if n_eff >= 2 else ""
            phase_ms = int(round(i * period_ms / max(1, n_eff)))
            coop_mode = "fused_offboard" if n_eff >= 2 else "independent"
            plan.append({
                "serial": serial,
                "agent_id": agent_id,
                "peer_agent_id": peer_id,
                "cooperative_mode": coop_mode,
                "ranging_period_ms": period_ms,
                "ranging_phase_ms": phase_ms,
            })
        return plan

    def _preview_n_osa_assignment(self):
        plan = self._compute_n_osa_plan()
        if not plan:
            return
        self.log("── N-OSA assignment preview ──")
        self.log(f"  N = {len(plan)}   period = {plan[0]['ranging_period_ms']} ms")
        for p in plan:
            self.log(
                f"  {p['serial']:>22}  →  {p['agent_id']:<8} "
                f"peer={p['peer_agent_id']:<8} "
                f"phase={p['ranging_phase_ms']:>4} ms  mode={p['cooperative_mode']}"
            )
        self.log("──────────────────────────────")

    def _auto_configure_n_osas(self):
        plan = self._compute_n_osa_plan()
        if not plan:
            return
        n_eff = len(plan)
        period_ms = plan[0]["ranging_period_ms"]
        self.after(0, lambda: self.log(
            f"N-OSA wizard: configuring {n_eff} OSAs (ring, period={period_ms}ms)"))
        base = {
            "label": self.exp_label.get() or "coop",
            "range_bias": self.exp_bias.get(),
            "notes": self.exp_notes.get() or f"ring/{n_eff}",
            "filter": self.exp_filter.get(),
        }
        base.update(self._altitude_extras())
        for p in plan:
            extras = dict(base)
            extras.update({
                "agent_id": p["agent_id"],
                "peer_agent_id": p["peer_agent_id"],
                "cooperative_mode": p["cooperative_mode"],
                "ranging_period_ms": str(p["ranging_period_ms"]),
                "ranging_phase_ms": str(p["ranging_phase_ms"]),
                "notes": f"{base['notes']} {p['agent_id']}→{p['peer_agent_id']}",
            })
            self.after(0, lambda s=p["serial"], a=p["agent_id"],
                       ph=p["ranging_phase_ms"]:
                       self.log(f"  [{s}] → {a} (phase={ph}ms)"))
            self.log(broadcast("EXPERIMENT_CONFIG", extras, serial=p["serial"]))
        self.after(0, lambda: self.log(
            "N-OSA wizard: done. Restart each phone app for agent_id to rename ROS topics."))
        self._set_check("config", True)

    def _send_sync_window(self):
        val = self.sync_window_combo.get().strip()
        try:
            ms = int(val)
            if not (10 <= ms <= 30000):
                raise ValueError
        except ValueError:
            self.log(f"Invalid sync window value: {val}")
            return
        serial = self._adb_serial()
        cmd = f"set_sync_window:{ms}"
        self.log(f"Setting sync_window to {ms}ms …")
        self._broadcast_to_selection("REMOTE_CMD", {"cmd": cmd}, serial)

    def _ntrip_serial(self) -> str | None:
        """Return the ADB serial selected in the NTRIP tab."""
        v = self.ntrip_serial_var.get()
        return None if v == "(any)" else v

    def _gt_serial(self) -> str | None:
        """Return the ADB serial selected in the Ground Truth tab."""
        v = self.gt_serial_var.get()
        return None if v == "(any)" else v

    def send_ntrip(self):
        extras = {
            "host": self.ntrip_host.get(),
            "port": self.ntrip_port.get(),
            "mountpoint": self.ntrip_mount.get(),
            "user": self.ntrip_user.get(),
            "pass": self.ntrip_pass.get(),
        }
        serial = self._ntrip_serial()
        serials = self._serials_from_selection(serial)
        if not serials:
            return
        self._broadcast_to_serials("NTRIP_CONFIG", extras, serials)
        self._broadcast_to_serials("REMOTE_CMD", {"cmd": "ntrip_connect"}, serials)

    def send_ntrip_all(self):
        """Send NTRIP config to ALL connected ADB devices."""
        serials = self._get_all_adb_serials()
        if not serials:
            self.log("No ADB devices connected.")
            return
        extras = {
            "host": self.ntrip_host.get(),
            "port": self.ntrip_port.get(),
            "mountpoint": self.ntrip_mount.get(),
            "user": self.ntrip_user.get(),
            "pass": self.ntrip_pass.get(),
        }
        self._broadcast_to_serials("NTRIP_CONFIG", extras, serials)
        self._broadcast_to_serials("REMOTE_CMD", {"cmd": "ntrip_connect"}, serials)

    def send_gt(self):
        extras = {
            "bssid": self.gt_bssid.get(),
            "lat": self.gt_lat.get(),
            "lon": self.gt_lon.get(),
            "alt": self.gt_alt.get(),
            "description": self.gt_desc.get(),
        }
        serial = self._gt_serial()
        ok = self._broadcast_to_selection("GROUND_TRUTH", extras, serial)
        self._set_check("gt", ok)
        # Store GT and add to map
        try:
            bssid = extras["bssid"].strip().lower()
            lat, lon = float(extras["lat"]), float(extras["lon"])
            alt = float(extras.get("alt", "0"))
            self._ground_truth_positions[bssid] = (lat, lon, alt)
            self._map_add_gt_marker(bssid, lat, lon, alt)
        except (ValueError, KeyError):
            pass

    def send_gt_all(self):
        """Send ground truth to ALL connected ADB devices."""
        serials = self._get_all_adb_serials()
        if not serials:
            self.log("No ADB devices connected.")
            return
        extras = {
            "bssid": self.gt_bssid.get(),
            "lat": self.gt_lat.get(),
            "lon": self.gt_lon.get(),
            "alt": self.gt_alt.get(),
            "description": self.gt_desc.get(),
        }
        ok = self._broadcast_to_serials("GROUND_TRUTH", extras, serials)
        self._set_check("gt", ok)
        try:
            bssid = extras["bssid"].strip().lower()
            lat, lon = float(extras["lat"]), float(extras["lon"])
            alt = float(extras.get("alt", "0"))
            self._ground_truth_positions[bssid] = (lat, lon, alt)
            self._map_add_gt_marker(bssid, lat, lon, alt)
        except (ValueError, KeyError):
            pass

    def push_gt_csv(self):
        path = filedialog.askopenfilename(
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if not path:
            return
        dest = f"{DATA_DIR}/ground_truth.csv"
        serial = self._gt_serial()
        serials = self._serials_from_selection(serial)
        if not serials:
            return
        for s in serials:
            self.log(f"$ adb -s {s} push {path} {dest}")
            self.log(adb(f"push {path} {dest}", serial=s, timeout=60))

    def _logs_serial(self) -> str | None:
        v = self.logs_serial_var.get()
        return None if v == "(any)" else v

    def pull_logs(self):
        base = self.log_dest.get()
        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        serial = self._logs_serial()
        serials = self._serials_from_selection(serial)
        if not serials:
            return
        src = f"{DATA_DIR}/ftm_logs/"
        def _pull():
            for s in serials:
                dest = str(Path(base) / ts / s)
                Path(dest).mkdir(parents=True, exist_ok=True)
                self.after(0, lambda s=s, d=dest: self.log(f"Pulling logs from [{s}] into {d}"))
                result = adb(f"pull {src} {dest}", serial=s, timeout=120)
                self.after(0, lambda r=result, s=s: self.log(f"[{s}] {r}"))
        self._run_in_thread(_pull)

    def pull_logs_all(self):
        """Pull logs from every connected ADB device into per-device subdirs."""
        serials = self._get_all_adb_serials()
        if not serials:
            self.log("No ADB devices connected.")
            return
        base = self.log_dest.get()
        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        src = f"{DATA_DIR}/ftm_logs/"
        def _pull_all():
            for s in serials:
                dest = str(Path(base) / ts / s)
                Path(dest).mkdir(parents=True, exist_ok=True)
                self.after(0, lambda s=s, d=dest: self.log(f"Pulling logs from [{s}] into {d}"))
                result = adb(f"pull {src} {dest}", serial=s, timeout=120)
                self.after(0, lambda r=result, s=s: self.log(f"[{s}] {r}"))
            self.after(0, lambda: self.log(f"Done — pulled logs from {len(serials)} device(s)"))
        self._run_in_thread(_pull_all)

    def list_logs(self):
        serial = self._logs_serial()
        serials = self._serials_from_selection(serial)
        if not serials:
            return
        for s in serials:
            self.log(f"$ adb -s {s} shell ls -la {DATA_DIR}/ftm_logs/")
            self.log(adb(f"shell ls -la {DATA_DIR}/ftm_logs/", serial=s))

    def delete_logs(self):
        serial = self._logs_serial()
        serials = self._serials_from_selection(serial)
        if not serials:
            return
        label = ", ".join(serials)
        if not messagebox.askyesno(
            "Confirm", f"Delete ALL log files on {label}?"
        ):
            return
        for s in serials:
            self.log(f"$ adb -s {s} shell rm -rf {DATA_DIR}/ftm_logs/*")
            self.log(adb(f"shell rm -rf {DATA_DIR}/ftm_logs/*", serial=s))

    def browse_dest(self):
        d = filedialog.askdirectory()
        if d:
            self.log_dest.set(d)

    # ──────────────────────────────────────────────────────────────
    #  Multi-OSA Fusion process management
    # ──────────────────────────────────────────────────────────────

    def _browse_mosa_log_dir(self):
        d = filedialog.askdirectory()
        if d:
            self.mosa_log_dir.set(d)

    def _start_multi_osa(self):
        """Launch the active Networked-OSA fusion node as a subprocess."""
        if self._multi_osa_proc is not None and self._multi_osa_proc.poll() is None:
            self.after(0, lambda: self.log("Multi-OSA is already running."))
            return

        script = str(MULTI_OSA_SCRIPT)
        if not Path(script).exists():
            self.after(0, lambda: self.log(f"ERROR: {script} not found"))
            return

        # Build command
        cmd = [sys.executable, script]
        agl = self.mosa_agl.get().strip()
        if agl:
            cmd += ["--agl-hint", agl]
        bias = self.mosa_bias.get().strip()
        if bias and bias != "0.0":
            cmd += ["--range-bias", bias]
        sigma = self.mosa_sigma.get().strip()
        if sigma:
            cmd += ["--sigma-threshold", sigma]
        min_s = self.mosa_min_samples.get().strip()
        if min_s:
            cmd += ["--min-samples", min_s]
        gnss_acc = self.mosa_gnss_acc.get().strip()
        if gnss_acc:
            cmd += ["--gnss-acc-max", gnss_acc]
        z_prior = self.mosa_z_prior_sigma.get().strip()
        if z_prior:
            cmd += ["--z-prior-sigma", z_prior]
        log_dir = self.mosa_log_dir.get().strip()
        if log_dir:
            Path(log_dir).mkdir(parents=True, exist_ok=True)
            cmd += ["--log-dir", log_dir]

        # Divergence reset flags (exposed in Multi-OSA tab)
        if getattr(self, "mosa_reset_enabled", None) is not None:
            if self.mosa_reset_enabled.get():
                cmd += ["--reset-on-divergence"]
                streak = self.mosa_reset_streak.get().strip()
                if streak:
                    cmd += ["--divergence-reject-streak", streak]
                nis_mean = self.mosa_reset_nis.get().strip()
                if nis_mean:
                    cmd += ["--divergence-nis-mean", nis_mean]
            else:
                cmd += ["--no-reset-on-divergence"]

        env = _mobile_ros_env()
        rmw = env.get("RMW_IMPLEMENTATION", "<default>")
        self.after(0, lambda: self.log(f"Starting Multi-OSA ({rmw}): {' '.join(cmd)}"))

        try:
            self._multi_osa_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                env=env,
                preexec_fn=os.setsid,
            )
        except Exception as e:
            self.after(0, lambda: self.log(f"ERROR launching Multi-OSA: {e}"))
            return

        pid = self._multi_osa_proc.pid

        def _update_ui():
            self.mosa_status_var.set("RUNNING")
            self.mosa_status_lbl.configure(style="Running.TLabel")
            self.mosa_pid_var.set(f"PID {pid}")
            self.mosa_start_btn.configure(state="disabled")
            self.mosa_stop_btn.configure(state="normal")
            self.log(f"Multi-OSA started (PID {pid})")
            self._set_check("fusion", True)
        self.after(0, _update_ui)

        # Reader thread: pipe subprocess stdout into the mosa_output widget
        def _reader():
            proc = self._multi_osa_proc
            if proc is None or proc.stdout is None:
                return
            try:
                for line in proc.stdout:
                    self.after(0, lambda l=line: self._mosa_append(l))
            except Exception:
                pass
            # Process ended
            rc = proc.wait()
            def _done():
                self.mosa_status_var.set(f"STOPPED (rc={rc})")
                self.mosa_status_lbl.configure(style="Stopped.TLabel")
                self.mosa_pid_var.set("")
                self.mosa_start_btn.configure(state="normal")
                self.mosa_stop_btn.configure(state="disabled")
                self.log(f"Multi-OSA stopped (rc={rc})")
                self._set_check("fusion", False)
            self.after(0, _done)

        self._multi_osa_log_thread = threading.Thread(target=_reader, daemon=True)
        self._multi_osa_log_thread.start()

    def _stop_multi_osa(self):
        """Stop the running Networked-OSA subprocess."""
        proc = self._multi_osa_proc
        if proc is None or proc.poll() is not None:
            self.after(0, lambda: self.log("Multi-OSA is not running."))
            return
        self.after(0, lambda: self.log("Stopping Multi-OSA …"))
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass

    def _mosa_append(self, text: str):
        """Append text to the Multi-OSA output widget (main thread)."""
        self.mosa_output.configure(state="normal")
        self.mosa_output.insert("end", text)
        self.mosa_output.see("end")
        # Keep buffer bounded (max ~2000 lines)
        line_count = int(self.mosa_output.index("end-1c").split(".")[0])
        if line_count > 2000:
            self.mosa_output.delete("1.0", f"{line_count - 1500}.0")
        self.mosa_output.configure(state="disabled")

    # ──────────────────────────────────────────────────────────────
    #  AP Estimates polling
    # ──────────────────────────────────────────────────────────────

    def _toggle_est_polling(self):
        if self._est_polling:
            self._est_polling = False
            self.est_poll_btn.configure(text="Start polling")
            self.est_status_var.set("Stopped")
            self.log("AP estimates polling stopped.")
        else:
            self._est_polling = True
            self.est_poll_btn.configure(text="Stop polling")
            self.est_status_var.set("Polling …")
            self.log("AP estimates polling started.")
            self._est_poll_thread = threading.Thread(
                target=self._est_poll_loop, daemon=True)
            self._est_poll_thread.start()

    def _est_poll_loop(self):
        """Background loop that polls estimates at the configured interval."""
        import time
        while self._est_polling:
            self._poll_estimates_once()
            try:
                interval = max(1.0, float(self.est_interval_var.get()))
            except ValueError:
                interval = 3.0
            # Sleep in small chunks so we can stop quickly
            elapsed = 0.0
            while elapsed < interval and self._est_polling:
                time.sleep(0.25)
                elapsed += 0.25

    def _poll_estimates_once(self):
        """Fetch latest AP estimates from all sources and update the table."""
        rows: list[tuple] = []
        now_str = datetime.now().strftime("%H:%M:%S")

        # 1) Multi-OSA fusion status
        try:
            fusion_raw = ros2_echo_once("/fusion/status", timeout=4)
            if fusion_raw and not fusion_raw.startswith("ERROR"):
                # The topic is std_msgs/String with data field containing JSON
                # ros2 echo prints: data: '{"t": ..., "aps": {...}}'
                m = re.search(r"data:\s*['\"](.+)['\"]", fusion_raw, re.DOTALL)
                if not m:
                    # Maybe the raw output IS the JSON
                    m = re.search(r"\{.+\}", fusion_raw, re.DOTALL)
                if m:
                    status = json.loads(m.group(1) if m.lastindex else m.group(0))
                    aps = status.get("aps", {})
                    n_drones = status.get("n_drones", 0)
                    for bssid, info in aps.items():
                        enu = info.get("enu")
                        converged = info.get("converged", False)
                        sigma_h = info.get("sigma_h", -1)
                        n_acc = info.get("n_accepted", info.get("n_samples", 0))
                        # Get lat/lon from per-anchor estimate topic
                        if converged and enu:
                            rows.append((
                                f"Multi-OSA ({n_drones}d)",
                                bssid,
                                "", "",  # lat/lon from status are ENU, get them from topic
                                f"{enu[2]:.1f}" if enu else "",
                                f"{sigma_h:.3f}" if sigma_h >= 0 else "",
                                str(n_acc),
                                now_str,
                                "fusion",
                                        "fusion",
                            ))
                        elif enu:
                            rows.append((
                                f"Multi-OSA ({n_drones}d)",
                                bssid,
                                "", "",
                                f"{enu[2]:.1f}" if enu else "",
                                f"{sigma_h:.3f}" if sigma_h >= 0 else "—",
                                str(n_acc),
                                now_str,
                                "fusion",
                                "fusion",
                            ))
        except Exception:
            pass

        # 1b) Try to get lat/lon from per-anchor estimate topics
        try:
            anchor_topics = ros2_list_topics(r"/fusion/anchor/.*/estimate")
            for topic in anchor_topics:
                raw = ros2_echo_once(topic, timeout=3)
                if raw and not raw.startswith("ERROR"):
                    # Parse Float64MultiArray: data: [lat, lon, alt, ...]
                    vals = _parse_float64_multiarray(raw)
                    if vals:
                        if len(vals) >= 7:
                            lat, lon, alt = vals[0], vals[1], vals[2]
                            n_s = int(vals[6])
                            # Extract bssid from topic name
                            bm = re.search(r"ap_([0-9a-f]+)/estimate", topic)
                            bssid_raw = bm.group(1) if bm else "?"
                            # Format as aa:bb:cc:dd:ee:ff
                            if len(bssid_raw) == 12:
                                bssid_fmt = ":".join(
                                    bssid_raw[i:i+2] for i in range(0, 12, 2))
                            else:
                                bssid_fmt = bssid_raw
                            # Update or add row
                            found = False
                            for i, r in enumerate(rows):
                                if r[1] == bssid_fmt and "Multi-OSA" in r[0]:
                                    rows[i] = (
                                        r[0], r[1],
                                        f"{lat:.7f}", f"{lon:.7f}", f"{alt:.1f}",
                                        r[5], str(n_s), now_str, "fusion", "fusion",
                                    )
                                    found = True
                                    break
                            if not found:
                                rows.append((
                                    "Multi-OSA",
                                    bssid_fmt,
                                    f"{lat:.7f}", f"{lon:.7f}", f"{alt:.1f}",
                                    "", str(n_s), now_str, "fusion",
                                    "fusion",
                                ))
        except Exception:
            pass

        # 1c) Raw (non-stable) fusion state — only surfaced if no stable
        # /fusion/anchor/*/estimate has been published for that BSSID yet.
        # Lets the GUI visualise convergence in real time.
        try:
            seen_fusion = {r[1] for r in rows if r[-1] == "fusion"}
            raw_topics = ros2_list_topics(r"/fusion/anchor/.*/state")
            for topic in raw_topics:
                bm = re.search(r"ap_([0-9a-f]+)/state", topic)
                bssid_raw = bm.group(1) if bm else "?"
                if len(bssid_raw) == 12:
                    bssid_fmt = ":".join(
                        bssid_raw[i:i+2] for i in range(0, 12, 2))
                else:
                    bssid_fmt = bssid_raw
                if bssid_fmt in seen_fusion:
                    continue
                raw = ros2_echo_once(topic, timeout=3)
                if raw and not raw.startswith("ERROR"):
                    vals = _parse_float64_multiarray(raw)
                    if vals and len(vals) >= 7:
                        lat, lon, alt = vals[0], vals[1], vals[2]
                        sigma_h = vals[3]
                        n_acc = int(vals[5])
                        converged = vals[6] > 0.5
                        label = "Multi-OSA"
                        if not converged:
                            label += " (in progress)"
                        rows.append((
                            label, bssid_fmt,
                            f"{lat:.7f}", f"{lon:.7f}", f"{alt:.1f}",
                            f"{sigma_h:.3f}", str(n_acc), now_str,
                            "fusion", "fusion",
                        ))
        except Exception:
            pass

        # 2) Per-phone (OSA) anchor estimates
        try:
            phone_topics = ros2_list_topics(r"/[^/]+/ftm/anchor/.*/estimate")
            for topic in phone_topics:
                # topic like: /pixel_7_pro_da383195/ftm/anchor/ap_xxxx/estimate
                parts = topic.strip("/").split("/")
                ns = parts[0] if parts else "?"
                friendly = KNOWN_UAVS.get(ns, ns)

                raw = ros2_echo_once(topic, timeout=3)
                if raw and not raw.startswith("ERROR"):
                    vals = _parse_float64_multiarray(raw)
                    if vals:
                        if len(vals) >= 7:
                            lat, lon, alt = vals[0], vals[1], vals[2]
                            cov_xx, cov_yy = vals[3], vals[4]
                            sigma_h = (cov_xx + cov_yy) ** 0.5
                            n_s = int(vals[6])
                            bm = re.search(r"ap_([0-9a-f]+)/estimate", topic)
                            bssid_raw = bm.group(1) if bm else "?"
                            if len(bssid_raw) == 12:
                                bssid_fmt = ":".join(
                                    bssid_raw[i:i+2] for i in range(0, 12, 2))
                            else:
                                bssid_fmt = bssid_raw
                            rows.append((
                                f"OSA: {friendly}",
                                bssid_fmt,
                                f"{lat:.7f}", f"{lon:.7f}", f"{alt:.1f}",
                                f"{sigma_h:.3f}",
                                str(n_s), now_str, "phone", ns,
                            ))
        except Exception:
            pass

        # 3) Per-phone anchor STATE (in-progress, before convergence)
        #    data = [lat, lon, alt, sigma_h, sigma_z, n_accepted, converged, bssid_hash]
        seen_bssids = {r[1] for r in rows}  # skip if already have a converged estimate
        try:
            state_topics = ros2_list_topics(r"/[^/]+/ftm/anchor/.*/state")
            for topic in state_topics:
                parts = topic.strip("/").split("/")
                ns = parts[0] if parts else "?"
                friendly = KNOWN_UAVS.get(ns, ns)

                bm = re.search(r"ap_([0-9a-f]+)/state", topic)
                bssid_raw = bm.group(1) if bm else "?"
                if len(bssid_raw) == 12:
                    bssid_fmt = ":".join(
                        bssid_raw[i:i+2] for i in range(0, 12, 2))
                else:
                    bssid_fmt = bssid_raw

                if bssid_fmt in seen_bssids:
                    continue  # prefer converged estimate

                raw = ros2_echo_once(topic, timeout=3)
                if raw and not raw.startswith("ERROR"):
                    vals = _parse_float64_multiarray(raw)
                    if vals and len(vals) >= 7:
                        lat, lon, alt = vals[0], vals[1], vals[2]
                        sigma_h = vals[3]
                        n_acc = int(vals[5])
                        converged = vals[6] > 0.5
                        label = f"OSA: {friendly}"
                        if not converged:
                            label += " (in progress)"
                        rows.append((
                            label,
                            bssid_fmt,
                            f"{lat:.7f}", f"{lon:.7f}", f"{alt:.1f}",
                            f"{sigma_h:.3f}",
                            str(n_acc), now_str, "phone", ns,
                        ))
        except Exception:
            pass

        # Update treeview and map in main thread
        def _update():
            self._live_estimate_rows = rows
            self._apply_live_source_filters(now_str=now_str)
        self.after(0, _update)

    # ──────────────────────────────────────────────────────────────
    #  Phone status query & NTRIP control
    # ──────────────────────────────────────────────────────────────

    def _query_phone_status(self):
        """Ask the phone to write status.json, pull it, and update the GUI."""
        serial = self._single_serial_from_selection(self._ntrip_serial())
        if not serial:
            return
        self.after(0, lambda: self.log("Querying phone status …"))

        # 1) Tell the phone to write status.json
        result = broadcast("REMOTE_CMD", {"cmd": "get_status"}, serial=serial)
        if "Broadcast completed" not in result:
            self.after(0, lambda: self.log(f"get_status broadcast failed: {result}"))
            return

        # 2) Small delay for the file to be written
        import time
        time.sleep(0.5)

        # 3) Pull the file
        remote = f"{DATA_DIR}/status.json"
        local = Path(tempfile.gettempdir()) / "rtt_phone_status.json"
        pull_result = adb(f"pull {remote} {local}", serial=serial)
        if "error" in pull_result.lower() or not local.exists():
            self.after(0, lambda: self.log(f"Failed to pull status.json: {pull_result}"))
            return

        # 4) Parse and update GUI
        try:
            data = json.loads(local.read_text())
        except (json.JSONDecodeError, OSError) as e:
            self.after(0, lambda: self.log(f"Failed to parse status.json: {e}"))
            return

        def _update_ui():
            ntrip_ok = data.get("ntrip_connected", False)
            self.phone_ntrip_status.set("CONNECTED" if ntrip_ok else "DISCONNECTED")
            self._set_check("ntrip", ntrip_ok)
            self.phone_fix_label.set(data.get("fix_label", "—"))
            sats = data.get("satellites", -1)
            self.phone_sats.set(str(sats) if sats >= 0 else "—")
            hdop = data.get("hdop", -1.0)
            self.phone_hdop.set(f"{hdop:.1f}" if hdop >= 0 else "—")
            self.phone_rtcm.set(str(data.get("rtcm_bytes", 0)))
            ext = data.get("external_gnss_active", False)
            self.phone_ext_gnss.set("Active" if ext else "Inactive")
            self.phone_sync_window.set(f"{data.get('sync_window_ms', '—')} ms")
            self.phone_weighting.set(data.get("weighting", "—"))
            ranging = data.get("continuous_ranging", False)
            self.phone_ranging.set("RUNNING" if ranging else "STOPPED")

            # Also update NTRIP config fields if we got them from the phone
            nh = data.get("ntrip_host", "")
            if nh:
                self.ntrip_host.set(nh)
                self.ntrip_port.set(str(data.get("ntrip_port", 2101)))
                self.ntrip_mount.set(data.get("ntrip_mountpoint", ""))
                self.ntrip_user.set(data.get("ntrip_user", ""))

            self.log(f"Phone status: NTRIP={'ON' if ntrip_ok else 'OFF'}, "
                     f"fix={data.get('fix_label','—')}, sats={sats}, "
                     f"sync={data.get('sync_window_ms','—')}ms, "
                     f"ranging={'ON' if ranging else 'OFF'}")

        self.after(0, _update_ui)

    def _ntrip_connect(self):
        serials = self._serials_from_selection(self._ntrip_serial())
        if not serials:
            return
        ok = self._broadcast_to_serials("REMOTE_CMD", {"cmd": "ntrip_connect"}, serials)
        self.after(0, lambda: self.log(f"NTRIP connect → {'OK' if ok else 'failed'}"))

    def _ntrip_disconnect(self):
        serials = self._serials_from_selection(self._ntrip_serial())
        if not serials:
            return
        ok = self._broadcast_to_serials("REMOTE_CMD", {"cmd": "ntrip_disconnect"}, serials)
        self.after(0, lambda: self.log(f"NTRIP disconnect → {'OK' if ok else 'failed'}"))

    # ──────────────────────────────────────────────────────────────
    #  Per-OSA live telemetry (background poller)
    # ──────────────────────────────────────────────────────────────

    def _start_telemetry_poller(self):
        if self._telemetry_polling:
            return
        self._telemetry_polling = True
        self._telemetry_thread = threading.Thread(
            target=self._telemetry_loop, daemon=True)
        self._telemetry_thread.start()
        self.log("Per-OSA telemetry poller started.")

    def _telemetry_loop(self):
        """Round-robin over discovered OSAs, updating fix/age/rtt rate badges."""
        import time
        last_rtt_sample: dict[str, tuple[float, int]] = {}  # ns → (t, count proxy)
        while self._telemetry_polling:
            ns_list = list(self.devices.keys())
            if not ns_list:
                time.sleep(1.0)
                continue
            for ns in ns_list:
                if not self._telemetry_polling:
                    break
                self._poll_one_osa(ns, last_rtt_sample)
                time.sleep(0.2)  # small gap between OSAs
            # idle between full rounds
            time.sleep(1.0)

    def _poll_one_osa(self, ns: str, last_rtt_sample: dict):
        """Fetch /<ns>/phone/location and /<ns>/ftm_rtt stats for one OSA."""
        import time
        dev = self.devices.get(ns, {})
        if isinstance(dev, dict) and not dev.get("ros_visible", True):
            # ADB-only entries are intentionally listed even when ROS discovery
            # missed them. Do not spend seconds polling non-visible ROS topics.
            now = time.time()
            self._osa_telemetry[ns] = {
                "lat": None, "lon": None, "acc": -1.0, "fix_code": -1,
                "age_ms": -1, "rate_hz": -1.0, "updated": now,
            }
            self.after(0, lambda n=ns: self._render_osa_telemetry(n))
            return
        # 1) phone/location → lat, lon, [alt], [acc], [fix], [sats], [ts_ms]
        raw = ros2_echo_once(f"/{ns}/phone/location", timeout=2)
        lat = lon = None
        acc = -1.0
        fix_code = -1
        age_ms = -1
        if raw and not raw.startswith("ERROR"):
            vals = _parse_float64_multiarray(raw)
            if vals and len(vals) >= 2:
                lat, lon = vals[0], vals[1]
                if len(vals) > 3:
                    acc = vals[3]
                if len(vals) > 4:
                    fix_code = int(vals[4])
                if len(vals) > 6:
                    # phone timestamp is ms-since-epoch
                    try:
                        t_ms = float(vals[6])
                        now_ms = time.time() * 1000.0
                        age_ms = max(0, int(now_ms - t_ms))
                    except (ValueError, OverflowError):
                        age_ms = -1
                # trail update
                if lat is not None and lon is not None:
                    self._append_trail(ns, lat, lon)

        # 2) ftm_rtt → count one message received (approximate rate)
        rtt_raw = ros2_echo_once(f"/{ns}/ftm_rtt", timeout=1)
        now = time.time()
        rate_hz = -1.0
        if rtt_raw and not rtt_raw.startswith("ERROR"):
            prev_t, prev_c = last_rtt_sample.get(ns, (now, 0))
            new_c = prev_c + 1
            dt = now - prev_t
            if dt > 4.0:
                rate_hz = new_c / dt
                last_rtt_sample[ns] = (now, 0)
            else:
                last_rtt_sample[ns] = (prev_t, new_c)
        else:
            # No message this tick. Refresh rate only when window elapses.
            prev_t, prev_c = last_rtt_sample.get(ns, (now, 0))
            dt = now - prev_t
            if dt > 6.0:
                rate_hz = prev_c / dt if dt > 0 else 0.0
                last_rtt_sample[ns] = (now, 0)

        self._osa_telemetry[ns] = {
            "lat": lat, "lon": lon, "acc": acc, "fix_code": fix_code,
            "age_ms": age_ms, "rate_hz": rate_hz, "updated": now,
        }
        self.after(0, lambda n=ns: self._render_osa_telemetry(n))

    @staticmethod
    def _fix_label_from_code(code: int) -> tuple[str, str]:
        """Map numeric GNSS fix code → (label, ttk style)."""
        # Convention from the app: 0 none / 1 2D / 2 3D / 4 RTK float / 5 RTK fixed
        mapping = {
            5: ("RTK FIX", "OsaOk.TLabel"),
            4: ("RTK FLT", "OsaWarn.TLabel"),
            2: ("3D", "OsaWarn.TLabel"),
            1: ("2D", "OsaError.TLabel"),
            0: ("NO FIX", "OsaError.TLabel"),
        }
        if code in mapping:
            return mapping[code]
        return ("—", "OsaTelemetry.TLabel")

    def _render_osa_telemetry(self, ns: str):
        dev = self.devices.get(ns)
        tel = self._osa_telemetry.get(ns)
        if not dev or not tel:
            return
        # agent_id is only shown reliably from status.json; fallback to ns
        dev["agent_var"].set(f"agent={ns}")
        # cooperative mode unknown from topic; show placeholder (can be set via status.json)
        dev["coop_var"].set("mode=—")

        fix_label, fix_style = self._fix_label_from_code(int(tel.get("fix_code", -1)))
        dev["fix_var"].set(f"fix={fix_label}")
        dev["fix_lbl"].configure(style=fix_style)

        age = tel.get("age_ms", -1)
        if age is None or age < 0:
            dev["age_var"].set("age=—")
            dev["age_lbl"].configure(style="OsaTelemetry.TLabel")
        else:
            dev["age_var"].set(f"age={age} ms")
            style = ("OsaOk.TLabel" if age < 500
                     else "OsaWarn.TLabel" if age < 2000
                     else "OsaError.TLabel")
            dev["age_lbl"].configure(style=style)

        rate = tel.get("rate_hz", -1.0)
        if rate is None or rate < 0:
            dev["rate_var"].set("rtt=…")
            dev["rate_lbl"].configure(style="OsaTelemetry.TLabel")
        else:
            dev["rate_var"].set(f"rtt={rate:.1f}/s")
            style = ("OsaOk.TLabel" if rate >= 1.0
                     else "OsaWarn.TLabel" if rate >= 0.2
                     else "OsaError.TLabel")
            dev["rate_lbl"].configure(style=style)

        lat, lon = tel.get("lat"), tel.get("lon")
        if lat is not None and lon is not None:
            dev["pos_var"].set(f"pos={lat:.6f},{lon:.6f}")
        else:
            dev["pos_var"].set("pos=—")

    # ──────────────────────────────────────────────────────────────
    #  Per-OSA trajectory trails on the map
    # ──────────────────────────────────────────────────────────────

    def _color_for_ns(self, ns: str) -> str:
        """Return a stable colour for a drone namespace.

        The mapping is first-come-first-served against
        ``self._osa_trail_colors``: the Nth unique namespace gets the
        Nth palette entry. Colours wrap modulo the palette size so each
        drone uses the same colour for its live marker and trail.
        """
        if ns in self._ns_color_index:
            idx = self._ns_color_index[ns]
        else:
            idx = len(self._ns_color_index) % len(self._osa_trail_colors)
            self._ns_color_index[ns] = idx
        return self._osa_trail_colors[idx]

    def _append_trail(self, ns: str, lat: float, lon: float, width: int = 3):
        trail = self._osa_trails.get(ns)
        if trail is None:
            trail = {
                "points": [],
                "path": None,
                "color": self._color_for_ns(ns),
                "width": width,
            }
            self._osa_trails[ns] = trail
        else:
            trail["width"] = max(int(width), int(trail.get("width", 3)))
        pts = trail["points"]
        # Skip duplicates
        if pts and abs(pts[-1][0] - lat) < 1e-8 and abs(pts[-1][1] - lon) < 1e-8:
            return
        pts.append((lat, lon))
        if len(pts) > self._osa_trail_max:
            del pts[: len(pts) - self._osa_trail_max]
        self.after(0, lambda n=ns: self._redraw_trail(n))

    def _redraw_trail(self, ns: str):
        if self._map_widget is None:
            return
        trail = self._osa_trails.get(ns)
        if not trail or len(trail["points"]) < 2:
            return
        try:
            if trail["path"] is not None:
                trail["path"].delete()
        except Exception:
            pass
        if not self._is_live_source_visible(ns):
            trail["path"] = None
            return
        try:
            trail["path"] = self._map_widget.set_path(
                trail["points"], color=trail["color"],
                width=int(trail.get("width", 3)))
        except Exception:
            trail["path"] = None

    def clear_osa_trails(self):
        """Wipe all trajectory polylines (useful between experiments)."""
        for ns, trail in list(self._osa_trails.items()):
            try:
                if trail.get("path") is not None:
                    trail["path"].delete()
            except Exception:
                pass
            trail["points"].clear()
            trail["path"] = None
            trail["width"] = 3
            trail["kind"] = "live"
        self.log("Cleared per-OSA trajectory trails.")

    # ──────────────────────────────────────────────────────────────
    #  Per-tab user guide helpers + About tab
    # ──────────────────────────────────────────────────────────────

    def _new_tab_with_guide(
        self,
        nb: ttk.Notebook,
        title: str,
        guide_text: str,
        padding: int = 8,
    ) -> ttk.Frame:
        """Create a notebook tab that displays a collapsible English user
        guide above the actual content.

        Returns the inner content frame so the caller can keep using
        ``.pack`` or ``.grid`` as before — the guide lives in a sibling
        wrapper so it never interferes with the tab's own layout manager.
        """
        wrapper = ttk.Frame(nb)
        nb.add(wrapper, text=title)
        self._make_guide_bar(wrapper, title, guide_text)
        inner = ttk.Frame(wrapper, padding=padding)
        inner.pack(fill="both", expand=True)
        return inner

    def _make_guide_bar(self, parent: ttk.Frame, title: str, body: str):
        """Compact, collapsible info banner shown at the top of each tab."""
        bar = ttk.Frame(parent)
        bar.pack(fill="x", padx=4, pady=(4, 0))

        shown = tk.BooleanVar(value=False)
        toggle_btn = ttk.Button(bar, width=22)
        toggle_btn.pack(side="left")
        ttk.Label(
            bar,
            text=f"  {title} — quick user guide",
            foreground="#0d47a1",
            font=("", 9, "italic"),
        ).pack(side="left")

        body_frame = ttk.Frame(parent)
        body_lbl = ttk.Label(
            body_frame,
            text=body,
            justify="left",
            foreground="#333333",
            background="#f5f7fb",
            relief="solid",
            borderwidth=1,
            padding=8,
            font=("", 9),
            wraplength=900,
        )
        body_lbl.pack(fill="x", padx=4, pady=(2, 4))

        def _toggle():
            if shown.get():
                body_frame.pack_forget()
                toggle_btn.configure(text="ⓘ  Show user guide ▸")
                shown.set(False)
            else:
                body_frame.pack(fill="x", after=bar)
                toggle_btn.configure(text="ⓘ  Hide user guide ▾")
                shown.set(True)

        toggle_btn.configure(text="ⓘ  Show user guide ▸", command=_toggle)

    def _build_about_tab(self, nb: ttk.Notebook):
        """First tab: welcome + basic end-to-end operating steps."""
        tab = ttk.Frame(nb, padding=10)
        nb.add(tab, text="About")

        ttk.Label(
            tab,
            text="RTT-SearchAgent — Mission Control",
            style="Header.TLabel",
            font=("", 14, "bold"),
        ).pack(anchor="w")
        ttk.Label(
            tab,
            text=(
                "Cooperative multilateration of WiFi victims with a swarm "
                "of Android-based WiFi-RTT drones."
            ),
            foreground="gray",
        ).pack(anchor="w", pady=(0, 8))

        # Scrollable text area so the About content is always fully visible
        # even on small laptops.
        container = ttk.Frame(tab)
        container.pack(fill="both", expand=True)

        txt = tk.Text(
            container, wrap="word", font=("", 10),
            background="#f5f7fb", foreground="#222222",
            relief="solid", borderwidth=1, padx=10, pady=10,
        )
        vsb = ttk.Scrollbar(container, orient="vertical", command=txt.yview)
        txt.configure(yscrollcommand=vsb.set)
        txt.pack(side="left", fill="both", expand=True)
        vsb.pack(side="right", fill="y")
        txt.insert("1.0", GUIDE_ABOUT)
        txt.configure(state="disabled")

        footer = ttk.Frame(tab)
        footer.pack(fill="x", pady=(8, 0))
        ttk.Label(
            footer,
            text=(
                "Tip: every other tab has its own ⓘ 'Show user guide' button "
                "with task-specific instructions."
            ),
            foreground="#0d47a1",
            font=("", 9, "italic"),
        ).pack(anchor="w")

    # ──────────────────────────────────────────────────────────────
    #  Window close
    # ──────────────────────────────────────────────────────────────

    def _on_close(self):
        """Kill Multi-OSA subprocess (if running) and close the window."""
        self._est_polling = False
        self._telemetry_polling = False
        proc = self._multi_osa_proc
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=3)
            except Exception:
                proc.kill()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
