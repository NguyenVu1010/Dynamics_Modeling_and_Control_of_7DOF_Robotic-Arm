# Sourced by the WSL scripts. A git worktree created by Windows git stores a Windows path
# ("gitdir: C:/...") in its .git file, which git inside WSL cannot open. When that happens,
# export GIT_DIR / GIT_WORK_TREE with the /mnt/<drive>/ form so git commands work.
# Usage: source git_env.sh <repo-root>
cobot_git_env() {
  local root="$1"
  if git -C "$root" rev-parse --git-dir >/dev/null 2>&1; then return 0; fi
  if [ -f "$root/.git" ]; then
    local gitdir
    gitdir="$(sed -n 's/^gitdir: //p' "$root/.git" | tr -d '\r')"
    if [[ "$gitdir" =~ ^([A-Za-z]):/(.*)$ ]]; then
      local drive="${BASH_REMATCH[1],,}"
      export GIT_DIR="/mnt/${drive}/${BASH_REMATCH[2]}"
      export GIT_WORK_TREE="$root"
    fi
  fi
}
cobot_git_env "$1"
