# Point-Mass Time-Reallocation Patent Drafting Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce a Chinese invention-patent draft of at least 20,000 Chinese characters that accurately abstracts the active point-mass time-reallocation implementation without exposing source-code interfaces, while leaving the simulation-results section unfilled.

**Architecture:** Treat the active implementation as the sole source of truth, first translating executable behavior into a code-to-mathematics ledger, then drafting claims and description around the verified algorithmic chain. Keep upstream trajectory generation generic throughout the invention, and confine EGO-Planner to an unfilled comparative-simulation template. Express implementation flow through equations and pseudocode rather than C++/ROS identifiers.

**Tech Stack:** Markdown, Chinese patent-document structure, LaTeX-style mathematical notation, pseudocode, shell-based consistency and character-count checks.

**Spec:** `docs/superpowers/specs/2026-09-14-point-mass-time-reallocation-patent-design.md`

## Global Constraints

- The protected subject is the independently usable point-mass trajectory time-reallocation algorithm.
- EGO-Planner is a third-party simulation frontend/baseline and is not a required feature of the invention.
- Every mandatory algorithmic statement must map to active behavior in `PointMassPathSearching.cpp` or be explicitly labeled as an optional embodiment.
- Do not disclose C++ class names, function names, ROS topics, message types, file paths, library APIs, or source-code signatures in the patent draft.
- Use equations and implementation-neutral pseudocode to explain the process.
- The final draft must contain at least 20,000 Chinese characters.
- The simulation section must contain protocol, metric, table, and figure placeholders but no invented values or conclusions.
- “Time-optimal” means optimal within the given waypoint sequence, candidate-speed set, endpoint-state conditions, and acceleration bounds.

---

### Task 1: Establish the code-to-mathematics conformance ledger

**Files:**
- Create: `docs/patents/point-mass-code-conformance-ledger.md`
- Read: `swarm-playground/main_ws/src/planner/point_mass_trajectory_generation/include/PointMassPathSearching.h`
- Read: `swarm-playground/main_ws/src/planner/point_mass_trajectory_generation/src/PointMassPathSearching.cpp`
- Read: `swarm-playground/main_ws/src/planner/plan_manage/src/traj_server.cpp`

**Interfaces:**
- Consumes: active source behavior and the approved patent-design spec.
- Produces: a private drafting ledger mapping algorithm stages, equations, edge cases, and claim-safe abstractions to exact source locations.

- [ ] **Step 1: Record the active input and output model**

Document the waypoint positions, waypoint direction vectors, current position, current velocity, acceleration bounds, sampling interval, and position/velocity/acceleration output. Mark current attitude storage as unused by the active point-mass search so it is excluded from mandatory patent features.

- [ ] **Step 2: Record candidate-speed and graph construction behavior**

Document that active intermediate-node candidates are collinear with a normalized reference direction and use sampled magnitudes; the current implementation creates 21 values but inserts the first 20, corresponding to 0.5 through 10.0 in increments of 0.5. Record the start node as the supplied current velocity and the terminal node as zero velocity.

- [ ] **Step 3: Derive both one-dimensional two-stage acceleration equations**

For each axis, write the velocity and displacement constraints for positive-then-negative and negative-then-positive acceleration orders. Verify that eliminating the second duration yields the quadratic coefficients used by the active implementation and that only roots with nonnegative first and second durations are feasible.

- [ ] **Step 4: Record three-axis synchronization behavior**

Document that the edge cost is the maximum of the three independently minimized axis durations. Identify the dominant axis, preserve its full-scale acceleration, and record how the non-dominant axes solve for a common duration using a shared acceleration-scale coefficient and a revised switching time.

- [ ] **Step 5: Record graph search and reconstruction behavior**

Document cumulative-time relaxation on a layered directed acyclic graph, parent-pointer storage, reverse traversal, and per-axis sampling of two constant-acceleration phases. Record zero-displacement axis handling and the minimum output-length padding behavior separately from the core claim.

- [ ] **Step 6: Mark non-active or non-guaranteed features**

Explicitly classify cone-direction sampling, full-state optimization, attitude/thrust output, collision guarantees between waypoints, jerk continuity, energy optimality, and continuous-domain global optimality as either inactive, optional, or unsupported.

- [ ] **Step 7: Verify the ledger contains no patent-facing source interfaces**

Run:

```bash
rg -n "PointMassPathSearching|solve\(|drawTrajectory|getPointMassTrajectory|/point_mass_cmd|ros::|Eigen::" docs/patents/point-mass-code-conformance-ledger.md
```

Expected: source identifiers may occur only in a clearly marked internal cross-reference column, never in proposed patent wording.

### Task 2: Confirm patent structure and prior-art framing

**Files:**
- Modify: `docs/patents/point-mass-code-conformance-ledger.md`
- Create: `docs/patents/point-mass-time-reallocation-patent-draft.md`

**Interfaces:**
- Consumes: the conformance ledger and authoritative patent-drafting requirements.
- Produces: a source-backed drafting boundary and initial patent-document skeleton.

- [ ] **Step 1: Check authoritative Chinese patent-document requirements**

Consult current CNIPA rules/guidance for abstract, claims, description support, clarity, and computer-implemented inventions. Record only requirements that materially affect the draft.

- [ ] **Step 2: Review primary technical literature for background framing**

Review primary papers or official project publications on geometric trajectory generation, time-optimal path parameterization, bang-bang acceleration profiles, and graph-based velocity planning. Use them only to avoid overclaiming and to describe general limitations; do not import their protected details into the invention.

- [ ] **Step 3: Create the complete patent skeleton**

Create headings for title, abstract, claims, technical field, background, invention content, drawings, symbols, detailed embodiments, alternatives, simulation template, and concluding scope language.

- [ ] **Step 4: Fix terminology before drafting**

Use “航点”, “候选速度”, “速度层”, “航段”, “轴向独立最短时间”, “航段公共时间”, “加速度缩放因子”, and “切换时刻” consistently. Define “时间重分配” as changing temporal parameterization and dynamic state assignment while retaining prescribed waypoint passage constraints.

### Task 3: Draft claims, abstract, and invention summary

**Files:**
- Modify: `docs/patents/point-mass-time-reallocation-patent-draft.md`

**Interfaces:**
- Consumes: verified algorithm stages from Tasks 1–2.
- Produces: approximately 20 claims plus an abstract and the description’s summary sections.

- [ ] **Step 1: Draft the independent method claim**

Cover generic waypoint input, candidate-speed layers, one-axis two-stage constrained transfer, three-axis common-time synchronization, edge-weighted shortest-path search, parent-chain recovery, and analytical trajectory reconstruction. Do not bind the claim to fixed candidate counts, fixed magnitudes, ROS, EGO-Planner, or specific source identifiers.

- [ ] **Step 2: Draft dependent method claims**

Add claims for direction normalization, magnitude sampling, two acceleration orders, quadratic-root feasibility filtering, maximum-axis edge cost, non-dominant-axis scaling, parent-pointer backtracking, zero-motion-axis handling, fixed-step output, endpoint conditions, and rolling recalculation.

- [ ] **Step 3: Draft system, device, and storage-medium claims**

Mirror the method’s mandatory data dependencies without adding unsupported physical components or results.

- [ ] **Step 4: Draft the abstract**

Summarize the technical problem, complete algorithm chain, and bounded benefits without claiming EGO-Planner ownership, universal global optimality, collision preservation, jerk continuity, or energy optimality.

- [ ] **Step 5: Draft invention purpose, technical solution, and beneficial effects**

Tie every stated effect to a mechanism: direction-constrained sampling reduces search dimension; layered graph search selects a minimum cumulative time in the candidate set; axis synchronization makes a three-axis segment executable under componentwise acceleration bounds; analytic reconstruction provides consistent position, velocity, and acceleration samples.

### Task 4: Draft the mathematical and algorithmic embodiments

**Files:**
- Modify: `docs/patents/point-mass-time-reallocation-patent-draft.md`

**Interfaces:**
- Consumes: claim terminology and code-conformance equations.
- Produces: a detailed, enabling description that supports every claim.

- [ ] **Step 1: Explain the point-mass model and optimization domain**

Define state, control, per-axis bounds, waypoint constraints, candidate sets, objective function, and the discrete-optimality qualification.

- [ ] **Step 2: Explain candidate-speed generation and layered graph construction**

Provide general formulas plus a code-conforming numerical embodiment using collinear velocity directions and sampled magnitudes.

- [ ] **Step 3: Explain the one-axis solver**

Derive both acceleration orders, quadratic equations, duration recovery, root filtering, and minimum feasible duration selection in sufficient detail to implement independently.

- [ ] **Step 4: Explain three-axis synchronization**

Derive common-time selection, dominant-axis retention, non-dominant-axis switching-time equation, acceleration-scale recovery, admissible scale interval, and switching state computation.

- [ ] **Step 5: Explain shortest-path search and backtracking**

Give implementation-neutral pseudocode for layer-by-layer relaxation using cumulative time and predecessor storage; explain why the returned chain is candidate-set optimal under nonnegative edge costs.

- [ ] **Step 6: Explain analytical reconstruction**

Give piecewise formulas for position, velocity, and acceleration before and after the switching time, followed by pseudocode for fixed-step sampling and segment concatenation.

- [ ] **Step 7: Explain edge cases and optional embodiments**

Cover zero displacement, degenerate quadratic coefficients, infeasible roots, numerical tolerances, asymmetric acceleration bounds, nonuniform candidate magnitudes, optional directional cones, alternative shortest-path implementations, and rolling replanning while clearly distinguishing each from the active baseline implementation.

### Task 5: Add a blank but operational simulation-comparison template

**Files:**
- Modify: `docs/patents/point-mass-time-reallocation-patent-draft.md`

**Interfaces:**
- Consumes: the user’s intended EGO-Planner comparison.
- Produces: a fill-ready simulation chapter with no fabricated result.

- [ ] **Step 1: Define the comparison protocol**

State that the same EGO-Planner output supplies the baseline trajectory and the waypoint/direction input to the independent reconstruction algorithm, with identical environment, initial state, terminal target, controller, and logging period where applicable.

- [ ] **Step 2: Define metrics and formulas**

Provide formulas for completion time, reconstruction computation time, path length, peak and RMS velocity, peak and RMS acceleration, tracking RMSE, maximum tracking error, waypoint passage error, minimum obstacle clearance, and repeated-trial statistics.

- [ ] **Step 3: Insert numbered table and figure placeholders**

Include blank tables for parameters and results and blank captions for path, time history, velocity, acceleration, tracking error, clearance, and computation-time plots. Each data cell must visibly say “待仿真后填写” or remain blank by design.

- [ ] **Step 4: Leave result interpretation empty**

Provide prompts for observations and conclusions but no comparative adjectives, numerical values, or claims of superiority.

### Task 6: Perform legal-technical consistency review

**Files:**
- Modify: `docs/patents/point-mass-time-reallocation-patent-draft.md`
- Read: `docs/patents/point-mass-code-conformance-ledger.md`

**Interfaces:**
- Consumes: complete patent draft and conformance ledger.
- Produces: a corrected draft whose claims, equations, terminology, and implementation statements agree.

- [ ] **Step 1: Check claim support**

For each claim, locate at least one detailed-description paragraph that defines and enables every feature. Revise unsupported language.

- [ ] **Step 2: Check code conformance**

Compare every mandatory pseudocode step and equation against the ledger. Remove or label any implementation extension that is not active in the source.

- [ ] **Step 3: Scan for forbidden implementation identifiers**

Run:

```bash
rg -n "PointMassPathSearching|calculateTperAxis|refineTperAxis|shortestPathSearching|drawTrajectory|Trajectories::|ros::|Eigen::|/point_mass|\.cpp|\.h" docs/patents/point-mass-time-reallocation-patent-draft.md
```

Expected: no matches.

- [ ] **Step 4: Scan EGO-Planner placement**

Run:

```bash
rg -n "EGO[- ]?Planner" docs/patents/point-mass-time-reallocation-patent-draft.md
```

Expected: matches occur only in the simulation-comparison chapter and its non-invention disclaimer.

- [ ] **Step 5: Check overclaiming terms**

Run:

```bash
rg -n "全局最优|绝对最优|完全避免碰撞|保证无碰撞|能量最优|姿态最优|推力最优|加加速度连续" docs/patents/point-mass-time-reallocation-patent-draft.md
```

Expected: no unconditional technical-effect claim; any occurrence is a limitation disclaimer.

### Task 7: Verify length, formatting, and deliverables

**Files:**
- Modify: `docs/patents/point-mass-time-reallocation-patent-draft.md`
- Modify: `docs/patents/point-mass-code-conformance-ledger.md`

**Interfaces:**
- Consumes: reviewed documents.
- Produces: final verified patent draft and internal traceability record.

- [ ] **Step 1: Count Chinese characters and total characters**

Run:

```bash
perl -CSD -0777 -ne '$h=()=/\p{Han}/g; $n=length($_); $l=()=/\S.*(?:\R|\z)/g; print "han=$h total=$n nonempty_lines=$l\n"' docs/patents/point-mass-time-reallocation-patent-draft.md
```

Expected: `han` is at least 20000.

- [ ] **Step 2: Check Markdown and whitespace**

Run:

```bash
git diff --check -- docs/patents/point-mass-time-reallocation-patent-draft.md docs/patents/point-mass-code-conformance-ledger.md
```

Expected: no output and exit status 0.

- [ ] **Step 3: Verify intentional simulation blanks**

Read the complete simulation chapter and confirm that it contains test setup, formulas, and placeholders but no fabricated result or conclusion.

- [ ] **Step 4: Verify repository scope**

Run:

```bash
git status --short
```

Expected: only the two patent documents and this plan are modified by the task; preserve the user’s unrelated `swarm-playground/main_ws/.github/` content.

- [ ] **Step 5: Commit the completed patent documents**

```bash
git add docs/patents/point-mass-time-reallocation-patent-draft.md docs/patents/point-mass-code-conformance-ledger.md
git commit -m "docs: draft point-mass time-reallocation patent"
```
