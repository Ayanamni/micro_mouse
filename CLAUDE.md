# CLAUDE.md

This file is intentionally short to reduce every-turn context.

## Mandatory Read

Every round, Claude reads only `项目维护协议.md` section 0.
Read more only when the task needs it:

- Process/rules conflict, repeated failure, or new module: read the relevant later section of `项目维护协议.md`.
- Controller design dispute: read `控制器方案报告.md`.
- Project map, commands, active path, or gotchas: read `项目速查.md`.
- Current execution plan: read `gpt计划.md` or `claude计划.md`.
- Compact recovery, handoff, or recent verification results: read `gpt执行记录.md`.

## Current Contract

- Remote: `https://github.com/Ayanamni/micro_mouse.git`, branch `main`; do not force-push unless explicitly asked.
- Active path: `control_core.vw_omega_step` / `vw_control_tick`, not legacy `control_core.step`.
- Priority: bandwidth/response time first, small overshoot second, max omega/speed range last.
- After each completed work block, prune completed or obsolete items from plan docs and `gpt执行记录.md`.
- Line-following `eval_vw.py single/sweep` remains paused until the user clearly releases that constraint.
- Dirty worktree may contain another agent's changes; do not revert or commit unrelated files.

## On-Demand Entry Points

- Controller core: `cpp/control_core/vw_controller.hpp`, `cpp/control_core/vw_controller.cpp`, `cpp/control_core/control_core.cpp`.
- Shared controller defaults: `micromouse_sim/config/vw_controller_config.py`.
- Evaluation tools: `scripts/eval_vw.py`, `scripts/bench_vw.py`, `scripts/test_omega_bandwidth.py`.
- Main simulation harness: `scripts/workbench.py`.
