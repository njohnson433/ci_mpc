---
name: create-readme
description: 'Create or update README.md for ci_mpc: precise, sourced, every term and number given its context'
---

## Reader

Write for a robotics researcher who knows MPC, impedance control and Isaac Sim but has **not** read `docs/PROGRESS.md`:
a new collaborator, a reviewer, or the user coming back after a month. After reading the README they must be able to say
what the project does, how each part works, what works now (under which conditions, measured how), what failed and why,
and how to run it. Write for that reader, not to make the project look good.

## Rules

### 1. Every number carries its context
A number is only useful if the reader knows **what was measured, on what, how many trials, and compared to what**.
- Success rates: give the count and the set, e.g. "30/30 insertions (3 repeats × 10 hole offsets up to 5 mm)".
- Errors: give the quantity and the task, e.g. "final position error 0.11 mm after a Cartesian step of X cm", not "reach 0.11 mm".
- Improvements: give the baseline and both values, e.g. "mean contact time 23.9 s vs 45.7 s for the hand-written
  likelihood on the held-out set R", not "about half the contact time".
- State the source (sim or real robot) wherever it isn't obvious from the section.
- If you can't find the context for a number in the code or docs, leave the number out. Don't guess it.

### 2. Define every project-internal name or leave it out
Benchmark IDs (D1, R, R2, H1–H4, X), job names (A7, B14, RA_v3), model versions (v3, v4), strategy names
("plain probe", "GPU entry", "refine"), flags and acronyms (PdZ, C3+, FISTA, ICNN) mean nothing to the reader.
- Define a term at first use in one plain clause, or put the recurring ones in a short **Benchmarks** /
  **Terms** table and link to it.
- Run IDs (A7, B14, ...) belong in PROGRESS.md, not in the README. Name the configuration instead.
- Expand an acronym once and add a reference (paper or link) for any method the project builds on (C3, Factory,
  AutoMate, momentum observer).

### 3. Explain the mechanism, not just the outcome
For each main component write a short **How it works**: the problem, the approach, the key design decision and why it
was made. Example: the hole-position belief is a grid over ±6 mm; each probe updates it by Bayes with a likelihood
p(reading | hole) that an MLP learns contrastively from Isaac probe logs. Do the same for negative results: say what was
tried, the numbers, and the **reason** it lost (e.g. "the off-policy distance prior made the belief chase its own probes").
"Didn't help" without a reason isn't allowed.

### 4. Status is explicit and bounded
For each component state one of: *works* (with the conditions it was tested under), *partial* (what works, what
doesn't), *failed / dropped* (why), *not started*. Say "sim only" / "not tested on the robot" where it applies. Never
imply generality beyond what was tested (one peg size, one friction value, N runs).

### 5. No filler
- Banned words unless backed by a number in the same sentence: robust, realistic, comprehensive, seamless,
  powerful, efficient, state-of-the-art, cutting-edge, leverages, enables, simply, easily, significant(ly).
- No badges (except real versions the reader needs), no nav bars, no emojis (✅/❌ included), no marketing
  "Features" list, no "Key highlights", no closing summary paragraph.
- One idea per sentence. Prefer a table when 3+ items share the same fields (task, metric, result, baseline,
  conditions).
- Don't restate the same fact in two sections.

### 6. Scope: README vs docs/
The README holds the **current** state: the best configuration per task with its baseline, how it works, how to run it.
Per-experiment history, sweeps and superseded results stay in `docs/PROGRESS.md`. Link to the dated entry
(`docs/PROGRESS.md#...`) instead of copying it. If a fact exists only in the README, move it to the right doc before
dropping it from the README.

### 7. Everything is checked
- Every number: find it in `docs/PROGRESS.md`, `docs/*.md` or a result file, and use the latest entry. Older entries
  may be superseded (e.g. results marked "pre-C3-fix").
- Every command, script, flag and path: confirm it exists (`ls`, grep the argparse). Don't invent options.
- Known breakages and gotchas from `CLAUDE.md` "Facts that bite" that affect running the code go in admonitions
  (GFM `> [!WARNING]` etc.).

## Structure

Use this order. Drop a section only if it would be empty.

1. **Title + 2–3 sentences:** what the system does, on which robot, current stage, sim or real.
2. **Status table:** component | what it does | result (metric, n, conditions) | baseline | status. One row per
   component, best configuration only.
3. **How it works:** architecture diagram (setpoint interface, rates), then one subsection per component following
   rule 3, including what was tried and dropped with the reason.
4. **Benchmarks and terms:** what each benchmark set is (task, offsets, seeds, repeats, success criterion, how contact
   time is measured), plus the remaining internal terms.
5. **Getting started:** prerequisites (versions), workspace, environments, quick start, warnings.
6. **Usage:** demo/benchmark commands with what each one shows and its expected result.
7. **Repository layout** and **Documentation** index.

## Final pass (do it, don't skip)

Read the draft sentence by sentence as the reader above and fix every sentence where they would ask:
"compared to what?", "measured how?", "how many runs?", "what is X?", "why?", or "does this hold on the real robot?".
Then grep the draft for the banned words in rule 5 and for undefined short IDs (`\b[A-Z][0-9]+\b`, `v[0-9]`).
