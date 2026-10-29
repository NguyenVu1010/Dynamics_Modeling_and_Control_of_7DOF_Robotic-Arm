"""tools/wsl/paths.sh and prune.sh: one WSL folder per project.

Every generated file of this repo in WSL lives under COBOT_ROOT (default
~/projects/7dof_dynamic_control), so it cannot mix with another project's builds:

    $COBOT_ROOT/build/<branch>[-asan|-cov]   CMake builds (build.sh)
    $COBOT_ROOT/colcon/<branch>              colcon workspace (sim.sh, env.sh)
    $COBOT_ROOT/venv                         Python virtualenv (setup.sh, pytest.sh, lint.sh)
    $COBOT_ROOT/out                          run outputs

<branch> is the checked-out branch with "/" replaced by "-" ("detached" without a branch).
prune.sh removes build/ and colcon/ entries whose branch no longer exists locally.
"""

import shutil
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
PATHS_SH = REPO / "tools/wsl/paths.sh"
PRUNE_SH = REPO / "tools/wsl/prune.sh"

pytestmark = pytest.mark.skipif(
    shutil.which("bash") is None or shutil.which("git") is None, reason="needs bash and git"
)


def run(cmd, cwd, env):
    proc = subprocess.run(cmd, cwd=cwd, env=env, capture_output=True, text=True, timeout=60)
    assert proc.returncode == 0, proc.stderr
    return proc.stdout


@pytest.fixture
def fake_repo(tmp_path):
    """A git repo with branches main, feat/a and fix/b; feat/a checked out."""
    home = tmp_path / "home"
    home.mkdir()
    repo = tmp_path / "repo"
    repo.mkdir()
    env = {
        "HOME": str(home),
        "PATH": "/usr/bin:/bin",
        "GIT_AUTHOR_NAME": "t",
        "GIT_AUTHOR_EMAIL": "t@t",
        "GIT_COMMITTER_NAME": "t",
        "GIT_COMMITTER_EMAIL": "t@t",
    }
    run(["git", "init", "-q", "-b", "main"], repo, env)
    run(["git", "commit", "-q", "--allow-empty", "-m", "init"], repo, env)
    run(["git", "branch", "fix/b"], repo, env)
    run(["git", "checkout", "-q", "-b", "feat/a"], repo, env)
    return repo, home, env


def paths(repo, env, expr):
    script = f'source "{PATHS_SH}" "{repo}"; {expr}'
    return run(["bash", "-c", script], repo, env).strip()


def test_defaults_live_under_one_project_folder(fake_repo):
    repo, home, env = fake_repo
    root = home / "projects" / "7dof_dynamic_control"
    assert paths(repo, env, 'echo "$COBOT_ROOT"') == str(root)
    assert paths(repo, env, "cobot_slug") == "feat-a"
    assert paths(repo, env, "cobot_build_dir") == str(root / "build" / "feat-a")
    assert paths(repo, env, "cobot_build_dir asan") == str(root / "build" / "feat-a-asan")
    assert paths(repo, env, "cobot_build_dir cov") == str(root / "build" / "feat-a-cov")
    assert paths(repo, env, "cobot_colcon_ws") == str(root / "colcon" / "feat-a")
    assert paths(repo, env, "cobot_venv") == str(root / "venv")
    assert paths(repo, env, "cobot_out") == str(root / "out")


def test_overrides(fake_repo, tmp_path):
    repo, _, env = fake_repo
    env = env | {
        "COBOT_ROOT": str(tmp_path / "elsewhere"),
        "COBOT_COLCON_WS": "/x/ws",
        "COBOT_VENV": "/x/venv",
    }
    assert paths(repo, env, "cobot_build_dir") == str(tmp_path / "elsewhere" / "build" / "feat-a")
    assert paths(repo, env, "cobot_colcon_ws") == "/x/ws"
    assert paths(repo, env, "cobot_venv") == "/x/venv"


def test_detached_head_has_its_own_slug(fake_repo):
    repo, _, env = fake_repo
    run(["git", "checkout", "-q", "--detach"], repo, env)
    assert paths(repo, env, "cobot_slug") == "detached"


def make_entries(home):
    root = home / "projects" / "7dof_dynamic_control"
    names = ["main", "main-asan", "feat-a", "feat-a-cov", "fix-b", "feat-gone", "feat-gone-asan"]
    for name in names:
        (root / "build" / name).mkdir(parents=True)
    for name in ["main", "feat-a", "feat-gone"]:
        (root / "colcon" / name).mkdir(parents=True)
    (root / "out" / "keep").mkdir(parents=True)
    return root


def test_prune_removes_only_entries_of_deleted_branches(fake_repo):
    repo, home, env = fake_repo
    root = make_entries(home)
    out = run(["bash", str(PRUNE_SH), "--repo", str(repo)], repo, env)
    assert sorted(p.name for p in (root / "build").iterdir()) == [
        "feat-a",
        "feat-a-cov",
        "fix-b",
        "main",
        "main-asan",
    ]
    assert sorted(p.name for p in (root / "colcon").iterdir()) == ["feat-a", "main"]
    assert (root / "out" / "keep").is_dir()
    assert "feat-gone" in out


def test_prune_dry_run_removes_nothing(fake_repo):
    repo, home, env = fake_repo
    root = make_entries(home)
    out = run(["bash", str(PRUNE_SH), "--repo", str(repo), "--dry-run"], repo, env)
    assert (root / "build" / "feat-gone-asan").is_dir()
    assert (root / "colcon" / "feat-gone").is_dir()
    assert "feat-gone-asan" in out


def test_prune_without_project_folder_is_a_no_op(fake_repo):
    repo, home, env = fake_repo
    run(["bash", str(PRUNE_SH), "--repo", str(repo)], repo, env)
    assert not (home / "projects").exists()
