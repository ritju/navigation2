#!/usr/bin/env python3
"""反复启动/关闭 Nav2 launch，检查节点能否正常起来、能否干净退出。

默认命令:
  USE_CURB_OR_WALL=Wall ros2 launch simulation_launcher nav2_only.launch.py

就绪判定（同时满足）:
  1. 日志出现 lifecycle manager 的 "Managed nodes are active"
  2. 所有已出现的 lifecycle_manager/*/is_active 服务返回 success=True
  3. （可选）ros2 node list 包含要求的节点名

退出判定:
  1. 先调用 lifecycle manager 的 manage_nodes(SHUTDOWN=4)
     /lifecycle_manager_navigation/manage_nodes
     /lifecycle_manager_localization/manage_nodes
  2. 再向 launch 进程组发 SIGINT
  3. 超时内进程组清空，且 ros2 node list 不再包含本轮启动的节点。

示例:
  python3 tools/test_nav2_launch_cycle.py
  python3 tools/test_nav2_launch_cycle.py --cycles 20 --dwell 15 --startup-timeout 180
"""

from __future__ import annotations

import argparse
import os
import re
import shlex
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple


READY_LOG = "Managed nodes are active"
FAIL_LOGS = (
    "Failed to bring up all requested nodes",
    "CRITICAL FAILURE: SERVER",
)
DEFAULT_REQUIRE_NODES = (
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "collision_monitor",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
    "map_server",
    "lifecycle_manager_navigation",
    "lifecycle_manager_localization",
)
DEFAULT_LAUNCH = ["ros2", "launch", "simulation_launcher", "nav2_only.launch.py"]
# nav2_msgs/srv/ManageLifecycleNodes: STARTUP=0 PAUSE=1 RESUME=2 RESET=3 SHUTDOWN=4
LIFECYCLE_SHUTDOWN_COMMAND = 4
DEFAULT_SHUTDOWN_SERVICES = (
    "/lifecycle_manager_navigation/manage_nodes",
    "/lifecycle_manager_localization/manage_nodes",
)
ROS_PROC_HINTS = (
    "ros2",
    "nav2_",
    "lifecycle_manager",
    "component_container",
    "controller_server",
    "planner_server",
    "bt_navigator",
    "map_server",
    "amcl",
    "rviz2",
    "robot_state_publisher",
)
NAV2_CHILD_HINTS = (
    "component_container",
    "plan_footprint_viz",
    "gzserver",
    "gzclient",
    "spawn_entity",
)


@dataclass
class CycleResult:
    index: int
    start_ok: bool = False
    stop_ok: bool = False
    start_sec: float = 0.0
    stop_sec: float = 0.0
    ready_managers: List[str] = field(default_factory=list)
    missing_nodes: List[str] = field(default_factory=list)
    leftover_pids: List[str] = field(default_factory=list)
    leftover_nodes: List[str] = field(default_factory=list)
    error: str = ""

    @property
    def passed(self) -> bool:
        return self.start_ok and self.stop_ok and not self.error


def classify_failure(result: CycleResult) -> Tuple[str, str]:
    """把一轮失败归成短类别 + 具体原因，便于累计统计。"""
    err = (result.error or "").strip()
    if result.passed:
        return "成功", ""
    if not result.start_ok:
        if "Failed to bring up" in err or "bringup" in err.lower():
            return "启动失败", "节点 bringup 失败"
        if "CRITICAL FAILURE" in err:
            return "启动失败", "CRITICAL FAILURE"
        if "dwell" in err:
            return "启动失败", "就绪后 dwell 期间进程退出"
        if "提前退出" in err:
            return "启动失败", "launch 进程提前退出"
        if "超时" in err or "timeout" in err.lower():
            return "启动失败", "启动超时未就绪"
        if result.missing_nodes:
            return "启动失败", "缺少节点: " + ", ".join(result.missing_nodes)
        return "启动失败", err or "未知启动失败"
    if not result.stop_ok:
        if "残留 Nav2 进程" in err:
            return "关闭失败", "残留 Nav2 进程"
        if "lifecycle SHUTDOWN" in err or "manage_nodes" in err:
            return "关闭失败", "lifecycle SHUTDOWN 未成功"
        if "残留 ROS 节点" in err:
            return "关闭失败", "残留 ROS 节点"
        if "SIGTERM" in err or "SIGKILL" in err:
            return "关闭失败", "需 SIGTERM/SIGKILL 才能退出"
        return "关闭失败", err or "未知关闭失败"
    return "失败", err or "未知失败"


def running_stats_line(results: Sequence[CycleResult], total_cycles: int) -> str:
    done = len(results)
    passed = sum(1 for r in results if r.passed)
    failed = done - passed
    rate = (100.0 * passed / done) if done else 0.0
    return (
        f"统计: 已完成 {done}/{total_cycles} | "
        f"成功 {passed} | 失败 {failed} | 成功率 {rate:.1f}%"
    )


def failure_reason_counts(results: Sequence[CycleResult]) -> Dict[str, List[int]]:
    grouped: Dict[str, List[int]] = {}
    for r in results:
        if r.passed:
            continue
        kind, reason = classify_failure(r)
        key = f"{kind} | {reason}"
        grouped.setdefault(key, []).append(r.index)
    return grouped


def build_summary_lines(
    results: Sequence[CycleResult],
    planned_cycles: int,
    interrupted: bool = False,
) -> List[str]:
    done = len(results)
    passed = sum(1 for r in results if r.passed)
    failed = done - passed
    start_fail = sum(1 for r in results if not r.start_ok)
    stop_fail = sum(1 for r in results if r.start_ok and not r.stop_ok)
    rate = (100.0 * passed / done) if done else 0.0
    lines = [
        f"循环测试次数: {done}"
        + (f" / 计划 {planned_cycles}" if planned_cycles != done else ""),
        f"成功次数: {passed}",
        f"失败次数: {failed}",
        f"成功率: {rate:.1f}%",
        f"启动失败: {start_fail}",
        f"关闭失败: {stop_fail}",
    ]
    if interrupted:
        lines.append("状态: 中途中断 (Ctrl+C)")
    grouped = failure_reason_counts(results)
    lines.extend(["", "失败原因统计:"])
    if not grouped:
        lines.append("  (无失败)")
    else:
        for key, cycles in grouped.items():
            ids = ", ".join(f"{n:03d}" for n in cycles)
            lines.append(f"  {key}: {len(cycles)} 次  [cycle {ids}]")
    lines.extend(["", "各轮明细:"])
    if not results:
        lines.append("  (尚未完成任何一轮)")
    for r in results:
        status = "PASS" if r.passed else "FAIL"
        kind, reason = classify_failure(r)
        extra = ""
        if not r.passed:
            extra = f" | {kind}: {reason}"
            if r.error and r.error != reason:
                extra += f" | detail={r.error}"
        lines.append(
            f"cycle {r.index:03d} {status} start={r.start_sec:.1f}s "
            f"stop={r.stop_sec:.1f}s managers={','.join(r.ready_managers) or '-'}"
            f"{extra}"
        )
    return lines


def write_summary(path: Path, lines: Sequence[str]) -> None:
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def ts() -> str:
    return datetime.now().strftime("%H:%M:%S")


def log(msg: str) -> None:
    print(f"[{ts()}] {msg}", flush=True)


def run_cmd(cmd: Sequence[str], timeout: float = 15.0) -> subprocess.CompletedProcess:
    return subprocess.run(
        list(cmd),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=timeout,
        check=False,
    )


def list_ros_nodes(timeout: float = 10.0) -> Set[str]:
    try:
        proc = run_cmd(["ros2", "node", "list"], timeout=timeout)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return set()
    if proc.returncode != 0:
        return set()
    return {line.strip() for line in proc.stdout.splitlines() if line.strip()}


def short_node_name(name: str) -> str:
    return name.rstrip("/").split("/")[-1]


def missing_required_nodes(required: Sequence[str], live: Iterable[str]) -> List[str]:
    live_short = {short_node_name(n) for n in live}
    live_full = set(live)
    missing = []
    for req in required:
        if req in live_full or req in live_short or any(n.endswith("/" + req) for n in live_full):
            continue
        missing.append(req)
    return missing


def list_is_active_services(timeout: float = 10.0) -> List[str]:
    try:
        proc = run_cmd(["ros2", "service", "list"], timeout=timeout)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return []
    if proc.returncode != 0:
        return []
    names = []
    for line in proc.stdout.splitlines():
        name = line.strip()
        if re.search(r"/lifecycle_manager[^/]*/is_active$", name):
            names.append(name)
    return sorted(set(names))


def list_manage_nodes_services(timeout: float = 10.0) -> List[str]:
    try:
        proc = run_cmd(["ros2", "service", "list"], timeout=timeout)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return []
    if proc.returncode != 0:
        return []
    names = []
    for line in proc.stdout.splitlines():
        name = line.strip()
        if re.search(r"/lifecycle_manager[^/]*/manage_nodes$", name):
            names.append(name)
    return sorted(set(names))


def manager_from_manage_nodes(service: str) -> str:
    parts = service.strip("/").split("/")
    if len(parts) >= 2 and parts[-1] == "manage_nodes":
        return parts[-2]
    return short_node_name(service)


def resolve_shutdown_services(preferred: Sequence[str]) -> List[str]:
    live = list_manage_nodes_services()
    resolved: List[str] = []
    used: Set[str] = set()
    for pref in preferred:
        target = pref if pref.startswith("/") else "/" + pref
        match = ""
        want_mgr = manager_from_manage_nodes(target)
        for name in live:
            if name == target or name.endswith("/" + target.lstrip("/")):
                match = name
                break
            if manager_from_manage_nodes(name) == want_mgr:
                match = name
                break
        if match:
            if match not in used:
                resolved.append(match)
                used.add(match)
        else:
            log(f"未找到 lifecycle 关停服务 {target}")
    return resolved


def call_lifecycle_shutdown(service: str, timeout: float) -> tuple[bool, str]:
    request = f"{{command: {LIFECYCLE_SHUTDOWN_COMMAND}}}"
    try:
        proc = run_cmd(
            [
                "ros2", "service", "call", service,
                "nav2_msgs/srv/ManageLifecycleNodes",
                request,
            ],
            timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        return False, f"{service} 调用超时 ({timeout:.0f}s)"
    except FileNotFoundError:
        return False, "找不到 ros2 命令"
    text = ((proc.stdout or "") + (proc.stderr or "")).strip()
    compact = text.replace(" ", "").replace("\n", "")
    if proc.returncode == 0 and "success=True" in compact:
        return True, ""
    snippet = text.replace("\n", " ")[:400]
    return False, f"{service} 失败: {snippet or f'returncode={proc.returncode}'}"


def shutdown_lifecycle_managers(
    services: Sequence[str],
    per_call_timeout: float,
) -> tuple[bool, str, float, List[str]]:
    t0 = time.monotonic()
    resolved = resolve_shutdown_services(services)
    if not resolved:
        live = list_manage_nodes_services()
        return (
            False,
            "未发现 manage_nodes 服务"
            + (f" (当前: {', '.join(live)})" if live else ""),
            time.monotonic() - t0,
            [],
        )

    errors: List[str] = []
    ok_services: List[str] = []
    for service in resolved:
        log(f"调用 {service} command={LIFECYCLE_SHUTDOWN_COMMAND} (SHUTDOWN)")
        ok, err = call_lifecycle_shutdown(service, per_call_timeout)
        if ok:
            log(f"{service} SHUTDOWN 成功")
            ok_services.append(service)
        else:
            log(err)
            errors.append(err)
    elapsed = time.monotonic() - t0
    if errors:
        return False, " ; ".join(errors), elapsed, ok_services
    return True, "", elapsed, ok_services


def is_active_true(service: str, timeout: float = 8.0) -> bool:
    try:
        proc = run_cmd(
            ["ros2", "service", "call", service, "std_srvs/srv/Trigger"],
            timeout=timeout,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False
    text = (proc.stdout or "") + (proc.stderr or "")
    return proc.returncode == 0 and "success=True" in text.replace(" ", "")


def pids_in_group(pgid: int) -> List[int]:
    pids: List[int] = []
    proc_dir = Path("/proc")
    if not proc_dir.is_dir():
        return pids
    for entry in proc_dir.iterdir():
        if not entry.name.isdigit():
            continue
        stat_path = entry / "stat"
        try:
            content = stat_path.read_text(encoding="utf-8", errors="ignore")
        except (OSError, FileNotFoundError):
            continue
        rparen = content.rfind(")")
        if rparen < 0:
            continue
        fields = content[rparen + 2 :].split()
        # After comm: state ppid pgrp ...
        if len(fields) < 3:
            continue
        try:
            pgrp = int(fields[2])
        except ValueError:
            continue
        if pgrp == pgid:
            pids.append(int(entry.name))
    return sorted(pids)


def proc_cmdline(pid: int) -> str:
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\x00", b" ")
        return raw.decode("utf-8", errors="replace").strip()
    except (OSError, FileNotFoundError):
        return ""


def proc_state(pid: int) -> str:
    try:
        content = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8", errors="ignore")
    except (OSError, FileNotFoundError):
        return ""
    rparen = content.rfind(")")
    if rparen < 0:
        return ""
    fields = content[rparen + 2 :].split()
    return fields[0] if fields else ""


def is_nav2_child_cmd(cmdline: str) -> bool:
    return any(h in cmdline for h in NAV2_CHILD_HINTS)


def is_ignorable_orphan(cmdline: str) -> bool:
    text = cmdline.lower()
    if "ros2 daemon" in text:
        return True
    if re.search(r"ros2(\s+|/.*/)(node|lifecycle|service|daemon)\b", text):
        return True
    if "launch_ros" in text and "component_container" not in text:
        return True
    return False


def looks_like_launch_orphan(cmdline: str, extra_hints: Sequence[str]) -> bool:
    text = cmdline.lower()
    hints = list(ROS_PROC_HINTS) + list(extra_hints)
    return any(h.lower() in text for h in hints)


def snapshot_ros_like_pids() -> Set[int]:
    pids: Set[int] = set()
    proc_dir = Path("/proc")
    if not proc_dir.is_dir():
        return pids
    for entry in proc_dir.iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        cmd = proc_cmdline(pid)
        if cmd and looks_like_launch_orphan(cmd, ()):
            pids.add(pid)
    return pids


def read_new_text(path: Path, offset: int) -> tuple[str, int]:
    if not path.exists():
        return "", offset
    data = path.read_bytes()
    if offset > len(data):
        offset = 0
    chunk = data[offset:].decode("utf-8", errors="replace")
    return chunk, len(data)


def extract_manager_from_log_line(line: str) -> Optional[str]:
    # [lifecycle_manager_navigation]: Managed nodes are active
    match = re.search(r"\[([^\]\s]*lifecycle_manager[^\]\s]*)\]", line)
    if match:
        return short_node_name(match.group(1))
    return None


def start_launch(
    launch_cmd: Sequence[str],
    extra_env: dict,
    log_path: Path,
) -> subprocess.Popen:
    env = os.environ.copy()
    env.update(extra_env)
    env.setdefault("RCUTILS_LOGGING_BUFFERED_STREAM", "1")
    env.setdefault("PYTHONUNBUFFERED", "1")
    # launch_ros lives in this process; UDPv4 avoids Fast-DDS SHM hang after _Exit.
    env.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = open(log_path, "w", encoding="utf-8", buffering=1)
    proc = subprocess.Popen(
        list(launch_cmd),
        stdout=log_file,
        stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL,
        env=env,
        start_new_session=True,
        close_fds=True,
    )
    # log_file 由子进程持有，父进程可关掉写入端
    log_file.close()
    return proc


def leftover_is_launch_only(pids: List[int]) -> bool:
    if not pids:
        return True
    for pid in pids:
        if proc_state(pid) in ("", "Z"):
            continue
        if is_nav2_child_cmd(proc_cmdline(pid)):
            return False
    return True


def signal_pids(pids: Sequence[int], sig: int) -> None:
    for pid in pids:
        try:
            os.kill(pid, sig)
        except (ProcessLookupError, PermissionError, OSError):
            pass


def terminate_group(
    proc: subprocess.Popen,
    pgid: int,
    timeout: float,
) -> tuple[bool, float, List[int]]:
    deadline = time.monotonic() + timeout
    sent_int = False
    sent_term = False
    t0 = time.monotonic()

    def remaining() -> float:
        return max(0.0, deadline - time.monotonic())

    leftover = pids_in_group(pgid)
    if not leftover and proc.poll() is not None:
        return True, 0.0, []

    # Only SIGINT the launch process. killpg(SIGINT) also hits children and
    # races Humble launch's own shutdown (launch hangs after children exit 0).
    try:
        os.kill(proc.pid, signal.SIGINT)
        sent_int = True
    except ProcessLookupError:
        sent_int = True

    sigint_budget = min(15.0, timeout * 0.5) if timeout > 5 else timeout
    sigint_until = time.monotonic() + max(2.0, sigint_budget)
    launch_only_since: Optional[float] = None

    while time.monotonic() < sigint_until:
        leftover = pids_in_group(pgid)
        if not leftover:
            return True, time.monotonic() - t0, []
        if leftover_is_launch_only(leftover):
            if launch_only_since is None:
                launch_only_since = time.monotonic()
            elif time.monotonic() - launch_only_since >= 2.0:
                # Nav2 children already gone; Humble launch_ros/rclpy can hang
                # in Fast-DDS after composed-node _Exit. That is not a Nav2 fail.
                signal_pids(leftover, signal.SIGTERM)
                time.sleep(1.0)
                leftover = pids_in_group(pgid)
                signal_pids(leftover, signal.SIGKILL)
                time.sleep(0.3)
                leftover = pids_in_group(pgid)
                return (not leftover), time.monotonic() - t0, leftover
        else:
            launch_only_since = None
        time.sleep(0.3)

    leftover = pids_in_group(pgid)
    if leftover and leftover_is_launch_only(leftover):
        signal_pids(leftover, signal.SIGTERM)
        time.sleep(1.0)
        leftover = pids_in_group(pgid)
        signal_pids(leftover, signal.SIGKILL)
        time.sleep(0.3)
        leftover = pids_in_group(pgid)
        return (not leftover), time.monotonic() - t0, leftover

    if leftover and remaining() > 0:
        try:
            os.killpg(pgid, signal.SIGTERM)
            sent_term = True
        except ProcessLookupError:
            return True, time.monotonic() - t0, []
        term_until = min(time.monotonic() + 5.0, deadline)
        while time.monotonic() < term_until:
            leftover = pids_in_group(pgid)
            if not leftover:
                return False, time.monotonic() - t0, []
            time.sleep(0.2)

    leftover = pids_in_group(pgid)
    if leftover:
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            leftover = []
        time.sleep(0.4)
        leftover = pids_in_group(pgid)

    elapsed = time.monotonic() - t0
    # SIGINT 后还需要对 Nav2 子进程发 SIGTERM/SIGKILL 视为退出不干净
    clean = (not leftover) and sent_int and not sent_term
    return clean, elapsed, leftover


def wait_launched_nodes_gone(baseline_nodes: Set[str], timeout: float) -> List[str]:
    deadline = time.monotonic() + timeout
    launched_left: List[str] = []
    while True:
        after_nodes = list_ros_nodes()
        launched_left = sorted(
            n for n in after_nodes - baseline_nodes if "launch_ros" not in n
        )
        if not launched_left:
            return []
        if time.monotonic() >= deadline:
            return launched_left
        time.sleep(0.5)


def refresh_ros_daemon() -> None:
    try:
        run_cmd(["ros2", "daemon", "stop"], timeout=10)
        time.sleep(0.3)
        run_cmd(["ros2", "daemon", "start"], timeout=10)
        time.sleep(0.4)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass


def wait_until_ready(
    proc: subprocess.Popen,
    log_path: Path,
    startup_timeout: float,
    ready_settle: float,
    min_managers: int,
    require_nodes: Sequence[str],
) -> tuple[bool, List[str], List[str], str, float]:
    t0 = time.monotonic()
    deadline = t0 + startup_timeout
    offset = 0
    ready_managers: Set[str] = set()
    last_service_set: tuple[str, ...] = ()
    services_stable_since: Optional[float] = None
    fail_reason = ""
    missing: List[str] = []

    while time.monotonic() < deadline:
        if proc.poll() is not None:
            return (
                False,
                sorted(ready_managers),
                missing,
                f"launch 进程提前退出, returncode={proc.returncode}",
                time.monotonic() - t0,
            )

        chunk, offset = read_new_text(log_path, offset)
        for line in chunk.splitlines():
            for fail in FAIL_LOGS:
                if fail in line:
                    fail_reason = fail
            if READY_LOG in line:
                mgr = extract_manager_from_log_line(line)
                ready_managers.add(mgr or "unknown_lifecycle_manager")

        if fail_reason:
            return False, sorted(ready_managers), missing, fail_reason, time.monotonic() - t0

        services = list_is_active_services()
        service_key = tuple(services)
        now = time.monotonic()
        if service_key != last_service_set:
            last_service_set = service_key
            services_stable_since = now if services else None

        all_active = False
        if services and services_stable_since is not None:
            if now - services_stable_since >= ready_settle:
                all_active = all(is_active_true(s) for s in services)

        live_nodes: Set[str] = set()
        if all_active:
            live_nodes = list_ros_nodes()
            missing = missing_required_nodes(require_nodes, live_nodes) if require_nodes else []

        managers_ok = len(ready_managers) >= min_managers
        if all_active and managers_ok and not missing:
            return True, sorted(ready_managers), [], "", time.monotonic() - t0

        time.sleep(0.5)

    if not last_service_set:
        fail_reason = "超时: 未发现 lifecycle_manager/*/is_active 服务"
    elif not ready_managers:
        fail_reason = f"超时: 未看到日志 '{READY_LOG}'"
    elif missing:
        fail_reason = "超时: 节点未齐: " + ", ".join(missing)
    else:
        fail_reason = (
            f"超时: manager 就绪 {sorted(ready_managers)}, "
            f"is_active 服务 {list(last_service_set)}"
        )
    return False, sorted(ready_managers), missing, fail_reason, time.monotonic() - t0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="反复启动并关闭 Nav2 launch，检查启动成功与干净退出。"
    )
    parser.add_argument("--cycles", type=int, default=10, help="循环次数 (默认 10)")
    parser.add_argument(
        "--dwell",
        type=float,
        default=10.0,
        help="检测到全部节点就绪后，保持运行的秒数再关闭 (默认 10)",
    )
    parser.add_argument(
        "--gap",
        type=float,
        default=5.0,
        help="上一轮退出后到下一轮启动前的间隔秒数 (默认 5)",
    )
    parser.add_argument(
        "--startup-timeout",
        type=float,
        default=120.0,
        help="等待全部节点就绪的超时秒数 (默认 120)",
    )
    parser.add_argument(
        "--shutdown-timeout",
        type=float,
        default=60.0,
        help="lifecycle SHUTDOWN 之后等待 SIGINT 退出的超时秒数 (默认 60)",
    )
    parser.add_argument(
        "--lifecycle-shutdown-timeout",
        type=float,
        default=45.0,
        help="每个 manage_nodes(SHUTDOWN) 服务调用超时秒数 (默认 45)",
    )
    parser.add_argument(
        "--shutdown-service",
        action="append",
        default=None,
        help="关停时调用的 manage_nodes 服务，可重复。默认先 navigation 再 localization",
    )
    parser.add_argument(
        "--ready-settle",
        type=float,
        default=3.0,
        help="lifecycle manager 服务列表稳定后再判定就绪的秒数 (默认 3)",
    )
    parser.add_argument(
        "--min-managers",
        type=int,
        default=2,
        help="至少要看到几个 lifecycle manager 打出 Managed nodes are active (默认 2)",
    )
    parser.add_argument(
        "--launch",
        default="ros2 launch simulation_launcher nav2_only.launch.py",
        help="launch 命令",
    )
    parser.add_argument(
        "--env",
        action="append",
        default=["USE_CURB_OR_WALL=Wall"],
        help="额外环境变量 KEY=VALUE，可重复。默认 USE_CURB_OR_WALL=Wall",
    )
    parser.add_argument(
        "--require-node",
        action="append",
        default=None,
        help="必须出现在 ros2 node list 中的节点名，可重复。默认使用 Nav2 常用节点列表",
    )
    parser.add_argument(
        "--no-require-nodes",
        action="store_true",
        help="不检查 ros2 node list，只看 lifecycle manager",
    )
    parser.add_argument(
        "--log-dir",
        default="",
        help="每轮 launch 日志目录。默认 ./nav2_launch_cycle_logs/<时间戳>",
    )
    parser.add_argument(
        "--stop-on-fail",
        action="store_true",
        help="某一轮失败后立即停止（默认失败也继续跑完）",
    )
    return parser.parse_args()


def parse_env(items: Sequence[str]) -> dict:
    env = {}
    for item in items:
        if not item:
            continue
        if "=" not in item:
            raise SystemExit(f"环境变量格式错误，应为 KEY=VALUE: {item}")
        key, value = item.split("=", 1)
        env[key] = value
    return env


def main() -> int:
    args = parse_args()
    if shutil_which("ros2") is None:
        log("错误: PATH 中找不到 ros2，请先 source 工作空间 setup.bash")
        return 2

    launch_cmd = shlex.split(args.launch)
    extra_env = parse_env(args.env)
    require_nodes: Sequence[str]
    if args.no_require_nodes:
        require_nodes = []
    elif args.require_node:
        require_nodes = args.require_node
    else:
        require_nodes = DEFAULT_REQUIRE_NODES

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = Path(args.log_dir) if args.log_dir else Path("nav2_launch_cycle_logs") / stamp
    log_dir.mkdir(parents=True, exist_ok=True)

    log(f"日志目录: {log_dir.resolve()}")
    log(f"命令: {' '.join(f'{k}={v}' for k, v in extra_env.items())} {' '.join(launch_cmd)}")
    log(
        f"轮次={args.cycles}, 就绪后保持={args.dwell}s, 启动超时={args.startup_timeout}s, "
        f"lifecycle关停超时={args.lifecycle_shutdown_timeout}s/服务, "
        f"SIGINT超时={args.shutdown_timeout}s, 轮间间隔={args.gap}s"
    )
    if require_nodes:
        log("要求节点: " + ", ".join(require_nodes))

    baseline_nodes = list_ros_nodes()
    baseline_pids = snapshot_ros_like_pids()
    if baseline_nodes:
        log(f"启动前已有 ROS 节点 {len(baseline_nodes)} 个，将忽略它们")

    results: List[CycleResult] = []
    interrupted = False
    try:
        for i in range(1, args.cycles + 1):
            result = CycleResult(index=i)
            cycle_log = log_dir / f"cycle_{i:03d}.log"
            log(f"======== 第 {i}/{args.cycles} 轮: 启动 ========")
            try:
                proc = start_launch(launch_cmd, extra_env, cycle_log)
            except FileNotFoundError as exc:
                result.error = f"无法启动: {exc}"
                log(result.error)
                results.append(result)
                break

            pgid = os.getpgid(proc.pid)
            log(f"launch pid={proc.pid} pgid={pgid}, 日志={cycle_log}")

            start_ok, managers, missing, err, start_sec = wait_until_ready(
                proc=proc,
                log_path=cycle_log,
                startup_timeout=args.startup_timeout,
                ready_settle=args.ready_settle,
                min_managers=args.min_managers,
                require_nodes=require_nodes,
            )
            result.start_ok = start_ok
            result.start_sec = start_sec
            result.ready_managers = managers
            result.missing_nodes = missing
            result.error = err

            if start_ok:
                log(
                    f"启动成功, 耗时 {start_sec:.1f}s, managers={managers or ['(from is_active)']}"
                )
                log(f"保持运行 {args.dwell:.1f}s ...")
                dwell_until = time.monotonic() + args.dwell
                while time.monotonic() < dwell_until:
                    if proc.poll() is not None:
                        result.start_ok = False
                        result.error = (
                            f"就绪后 dwell 期间 launch 自行退出, returncode={proc.returncode}"
                        )
                        log(result.error)
                        break
                    time.sleep(0.3)
            else:
                log(f"启动失败 ({start_sec:.1f}s): {err}")

            shutdown_services = args.shutdown_service or list(DEFAULT_SHUTDOWN_SERVICES)
            lifecycle_ok = True
            lifecycle_sec = 0.0
            if result.start_ok:
                log(
                    f"第 {i} 轮: lifecycle SHUTDOWN "
                    f"({' -> '.join(shutdown_services)})"
                )
                lifecycle_ok, lifecycle_err, lifecycle_sec, ok_svcs = shutdown_lifecycle_managers(
                    shutdown_services,
                    args.lifecycle_shutdown_timeout,
                )
                if lifecycle_ok:
                    log(f"lifecycle SHUTDOWN 完成, 耗时 {lifecycle_sec:.1f}s, 服务={ok_svcs}")
                else:
                    log(f"lifecycle SHUTDOWN 失败 ({lifecycle_sec:.1f}s): {lifecycle_err}")
                    result.error = (
                        (result.error + " ; " if result.error else "") + lifecycle_err
                    )
            else:
                log("启动未成功，跳过 lifecycle SHUTDOWN，直接 SIGINT")

            log(f"第 {i} 轮: 关闭 (SIGINT)")
            clean, stop_sec, leftover = terminate_group(proc, pgid, args.shutdown_timeout)
            stop_sec = lifecycle_sec + stop_sec
            # 再等 launch 主进程 wait，避免僵尸
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                pass

            leftover = [pid for pid in leftover if proc_state(pid) not in ("", "Z")]
            leftover_cmds = []
            nav2_left = []
            launch_left = []
            for pid in leftover:
                cmd = proc_cmdline(pid) or "<unknown>"
                if is_nav2_child_cmd(cmd):
                    nav2_left.append(f"{pid}: {cmd}")
                else:
                    launch_left.append(f"{pid}: {cmd}")
                    try:
                        os.kill(pid, signal.SIGKILL)
                    except (ProcessLookupError, PermissionError, OSError):
                        pass
            leftover_cmds = nav2_left

            refresh_ros_daemon()
            launched_left = wait_launched_nodes_gone(baseline_nodes, 5.0)

            after_pids = snapshot_ros_like_pids() - baseline_pids
            extra_orphans = []
            for pid in sorted(after_pids):
                if pid in leftover:
                    continue
                if proc_state(pid) in ("", "Z"):
                    continue
                cmd = proc_cmdline(pid)
                if not cmd or is_ignorable_orphan(cmd):
                    continue
                if is_nav2_child_cmd(cmd):
                    extra_orphans.append(f"{pid}: {cmd}")

            if not lifecycle_ok and not leftover_cmds and not extra_orphans:
                try:
                    cycle_text = cycle_log.read_text(encoding="utf-8", errors="replace")
                except OSError:
                    cycle_text = ""
                if (
                    "Managed nodes have been shut down" in cycle_text
                    and "finished cleanly" in cycle_text
                    and "process has died" not in cycle_text
                ):
                    log("lifecycle 客户端超时，但 launch 日志显示 Nav2 已干净关停")
                    lifecycle_ok = True
                    result.error = ""

            result.stop_sec = stop_sec
            result.leftover_pids = leftover_cmds + extra_orphans
            nav2_clean = not leftover_cmds and not extra_orphans
            if launched_left and nav2_clean:
                log("DDS 图仍有节点，但 Nav2 进程已退出: " + ", ".join(launched_left))
                launched_left = []
            result.leftover_nodes = launched_left
            result.stop_ok = bool(lifecycle_ok and nav2_clean)
            if launch_left and result.stop_ok:
                log("launch 进程未自行退出，已结束（Nav2 子进程已干净退出）")

            if result.stop_ok:
                log(f"关闭成功, 耗时 {stop_sec:.1f}s")
            else:
                parts = []
                if not lifecycle_ok:
                    parts.append("lifecycle SHUTDOWN 未成功")
                if leftover_cmds or extra_orphans:
                    parts.append("残留 Nav2 进程:\n  " + "\n  ".join(result.leftover_pids))
                if launched_left:
                    parts.append("残留 ROS 节点: " + ", ".join(launched_left))
                detail = " | ".join(parts) if parts else "未知原因"
                if result.error:
                    result.error += " ; " + detail
                else:
                    result.error = detail
                log(f"关闭异常 ({stop_sec:.1f}s): {detail}")
                # 尽量清场，避免污染下一轮
                for pid in list(after_pids) + leftover:
                    try:
                        os.kill(pid, signal.SIGKILL)
                    except (ProcessLookupError, PermissionError, OSError):
                        pass

            results.append(result)
            log(
                f"第 {i} 轮结果: "
                f"{'PASS' if result.passed else 'FAIL'} "
                f"(start={'OK' if result.start_ok else 'NG'}, "
                f"stop={'OK' if result.stop_ok else 'NG'})"
            )
            if not result.passed:
                kind, reason = classify_failure(result)
                log(f"第 {i} 轮失败原因: {kind} | {reason}")
                if result.error and result.error != reason:
                    log(f"第 {i} 轮失败详情: {result.error}")
            log(running_stats_line(results, args.cycles))
            write_summary(
                log_dir / "summary.txt",
                build_summary_lines(results, args.cycles),
            )

            if args.stop_on_fail and not result.passed:
                log("遇到失败，按 --stop-on-fail 停止")
                break

            if i < args.cycles and args.gap > 0:
                log(f"等待 {args.gap:.1f}s 后开始下一轮")
                time.sleep(args.gap)
    except KeyboardInterrupt:
        interrupted = True
        log("收到 Ctrl+C，测试中止，写入当前统计")

    summary_path = log_dir / "summary.txt"
    lines = build_summary_lines(results, args.cycles, interrupted=interrupted)
    write_summary(summary_path, lines)

    print()
    log("======== 汇总 ========")
    print("\n".join(lines), flush=True)
    log(f"摘要已写入 {summary_path}")
    if interrupted:
        return 130
    failed = sum(1 for r in results if not r.passed)
    return 0 if failed == 0 else 1


def shutil_which(name: str) -> Optional[str]:
    from shutil import which

    return which(name)


if __name__ == "__main__":
    sys.exit(main())
