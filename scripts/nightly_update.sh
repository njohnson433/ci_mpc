#!/bin/bash
# Nightly backup of the day's work (cron, 23:55): refresh the controller patch, let Claude (headless, create-readme
# skill) update README.md and add a daily summary to docs/PROGRESS.md, then commit and push to origin/main.
# Does nothing if there were no commits today and nothing is uncommitted.
#   scripts/nightly_update.sh            # normal run (cron)
#   scripts/nightly_update.sh --dry-run  # everything except commit / push (shows the diff stat)
# Log: ~/.local/state/ci_mpc_nightly.log
set -u
export PATH="$HOME/.local/bin:/usr/local/bin:/usr/bin:/bin"
REPO="$HOME/franka_ros2_ws/src/ci_mpc"
CTRL="$HOME/franka_ros2_ws/src/cartesian_impedance_control"
LOG="$HOME/.local/state/ci_mpc_nightly.log"
DRY=0; [ "${1:-}" = "--dry-run" ] && DRY=1
mkdir -p "$(dirname "$LOG")"
exec >> "$LOG" 2>&1
echo "=== $(date '+%F %T') nightly update (dry run: $DRY)"
cd "$REPO" || exit 1
TODAY=$(date +%F)

# controller backup: regenerate the patch from the local branch (upstream has no push access)
if [ -d "$CTRL/.git" ]; then
  (cd "$CTRL" && git format-patch main..isaac-sim-pinocchio --stdout) > patches/cartesian_impedance_control_isaac-sim-pinocchio.patch \
    || echo "patch regeneration failed"
fi

COMMITS_TODAY=$(git log --since="$TODAY 00:00" --oneline | wc -l)
if [ "$COMMITS_TODAY" -eq 0 ] && [ -z "$(git status --porcelain)" ]; then
  echo "nothing happened today, skipping"
  exit 0
fi

PROMPT="Nightly documentation update for the ci_mpc repo (today is $TODAY). Do not change any code.
1. Add a section '### $TODAY: daily summary' at the top of the Log in docs/PROGRESS.md (below '## Log'), unless one
   for $TODAY exists already: 3-8 bullets on what was done today, with the concrete numbers, based on
   'git log --since=\"$TODAY 00:00\" --stat' and 'git diff HEAD --stat' and the existing PROGRESS.md entries of today.
   Be honest about negative results. Keep the style of the existing entries.
2. Invoke the create-readme skill (Skill tool) and follow ALL of its rules for README.md, including its final pass:
   every number with its context (metric, set, n, baseline), every internal name defined or left out, mechanisms and
   reasons instead of adjectives. Restructure the existing README where it does not follow the skill (do not just
   patch table rows). Reflect today's work. Facts must stay true and current; per-experiment numbers belong in docs/:
   link there. Before dropping a fact from the README, make sure it is recorded in docs/.
Only edit README.md and docs/PROGRESS.md."
claude -p "$PROMPT" --allowedTools "Skill,Read,Edit,Write,Glob,Grep,WebFetch,Bash(git log:*),Bash(git diff:*),Bash(git status:*),Bash(ls:*)" \
  --permission-mode acceptEdits < /dev/null || echo "claude run failed (committing the work anyway)"

# only what this job writes: a blanket add would sweep in other sessions' uncommitted code (1606d5f did)
git add README.md docs patches
if git diff --cached --quiet; then
  echo "no changes to commit"
  exit 0
fi
git diff --cached --stat | tail -5
if [ "$DRY" -eq 1 ]; then
  git reset -q
  echo "dry run: not committing"
  exit 0
fi
git commit -q -m "Nightly update $TODAY: daily summary, README, controller patch

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" && echo "committed $(git log --oneline | head -1)"
git push origin main && echo "pushed" || echo "push failed"
