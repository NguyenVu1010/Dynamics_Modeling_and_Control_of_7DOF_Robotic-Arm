"""tools/wsl/env.sh: per-project ROS 2 / Gazebo isolation and the stale colcon workspace guard.

Another ROS 2 project in the same WSL (graph_slam) uses the default domain 0 and the default
Gazebo partition, so two simulations running at once would share /clock, /joint_states and the
gz transport topics. `source tools/wsl/env.sh` sets ROS_DOMAIN_ID (COBOT_ROS_DOMAIN_ID, default
42, overriding the 0 from ~/.bashrc) and GZ_PARTITION (COBOT_GZ_PARTITION, default cobot).

cobot_ws_guard WS REPO wipes WS/build and WS/install when a package there was configured from a
source directory outside REPO (a deleted worktree, another clone): CMake would otherwise stop
with "The source directory ... does not exist".
"""

import shutil
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
ENV_SH = REPO / "tools/wsl/env.sh"

pytestmark = pytest.mark.skipif(shutil.which("bash") is None, reason="needs bash")


def bash(script, tmp_path, env_extra=None):
    env = {"HOME": str(tmp_path), "PATH": "/usr/bin:/bin", "ROS_DOMAIN_ID": "0"}
    env.update(env_extra or {})
    return subprocess.run(
        ["bash", "-c", script], capture_output=True, text=True, env=env, timeout=60, check=False
    )


def exported(tmp_path, env_extra=None):
    script = f'source "{ENV_SH}" >/dev/null 2>&1; echo "$ROS_DOMAIN_ID|$GZ_PARTITION"'
    proc = bash(script, tmp_path, env_extra)
    assert proc.returncode == 0, proc.stderr
    return proc.stdout.strip().splitlines()[-1].split("|")


def test_defaults_override_the_shell_domain(tmp_path):
    assert exported(tmp_path) == ["42", "cobot"]


def test_overrides_from_the_environment(tmp_path):
    extra = {"COBOT_ROS_DOMAIN_ID": "7", "COBOT_GZ_PARTITION": "arm-a"}
    assert exported(tmp_path, extra) == ["7", "arm-a"]


def fake_ws(tmp_path, source_dir):
    ws = tmp_path / "ws"
    for pkg in ("cobot_sim", "cobot_ros2"):
        build = ws / "build" / pkg
        build.mkdir(parents=True)
        cache = f"CMAKE_HOME_DIRECTORY:INTERNAL={source_dir}/{pkg}\n"
        (build / "CMakeCache.txt").write_text(cache, encoding="utf-8")
    (ws / "install").mkdir()
    (ws / "install" / "marker").write_text("x", encoding="utf-8")
    return ws


def guard(tmp_path, ws, repo):
    proc = bash(f'source "{ENV_SH}" >/dev/null 2>&1; cobot_ws_guard "{ws}" "{repo}"', tmp_path)
    assert proc.returncode == 0, proc.stderr
    return proc


def test_guard_wipes_a_workspace_built_from_another_source(tmp_path):
    ws = fake_ws(tmp_path, "/mnt/c/old-worktree")
    proc = guard(tmp_path, ws, "/mnt/c/repo")
    assert not (ws / "build").exists()
    assert not (ws / "install").exists()
    assert "/mnt/c/old-worktree" in proc.stdout + proc.stderr


def test_guard_keeps_a_workspace_built_from_this_repo(tmp_path):
    ws = fake_ws(tmp_path, "/mnt/c/repo")
    guard(tmp_path, ws, "/mnt/c/repo")
    assert (ws / "build" / "cobot_sim" / "CMakeCache.txt").exists()
    assert (ws / "install" / "marker").exists()


def test_guard_does_not_match_a_sibling_with_the_same_prefix(tmp_path):
    # /mnt/c/repo-old is not inside /mnt/c/repo.
    ws = fake_ws(tmp_path, "/mnt/c/repo-old")
    guard(tmp_path, ws, "/mnt/c/repo")
    assert not (ws / "build").exists()


def test_guard_on_a_missing_workspace_is_a_no_op(tmp_path):
    guard(tmp_path, tmp_path / "nothing", "/mnt/c/repo")
    assert not (tmp_path / "nothing").exists()
