# Point-Mass Patent Theoretical Correction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rewrite the point-mass trajectory reconstruction patent so that every feasibility and optimality statement is mathematically valid within an explicitly bounded problem domain, while retaining a Chinese patent draft of at least 20,000 Han characters and leaving the simulation results section genuinely blank.

**Architecture:** Treat each waypoint velocity candidate as a node in a layered directed acyclic graph. Define every edge by a three-axis synchronous bounded-acceleration transition, compute its cost as the minimum common feasible duration, use layered dynamic programming for the finite-graph optimum, and reconstruct the trajectory from the exact parameters validated during edge evaluation.

**Tech Stack:** Chinese Markdown patent document, LaTeX equations, shell-based consistency checks, Git.

## Global Constraints

- Work directly on the user-approved `main` branch.
- Do not modify the unrelated untracked `swarm-playground/main_ws/.github/` directory.
- Do not mention source-code class names, function names, message types, topics, or file interfaces in the patent.
- Do not describe implementation completion status in the patent.
- Limit “time optimal” to the fixed waypoint order, finite candidate-velocity graph, stated acceleration constraints, and stated segment control family.
- Treat `max(T_x^*,T_y^*,T_z^*)` only as a lower bound unless it is feasible for all axes.
- Retain EGO-Planner only as a third-party comparison frontend in the reserved simulation section.
- Use `apply_patch` for authored file changes.

---

### Task 1: Establish an auditable mathematical baseline

**Files:**
- Modify: `docs/patents/point-mass-code-conformance-ledger.md`
- Create: `docs/patents/point-mass-patent-review-report.md`

- [ ] Separate observed program behavior from the corrected theoretical method in the conformance ledger.
- [ ] Record the original draft's critical issues: non-minimal root/order selection, unchecked acceleration scaling, unconditional maximum-axis synchronization, ambiguous tie handling, shortest-path update semantics, endpoint sampling, unsupported constraints, and overfilled simulation section.
- [ ] State the corrected mathematical domain and the precise meaning of finite-candidate global optimality.
- [ ] Record items that still require the inventor or patent professional rather than treating them as mathematical facts: inventorship, prior-art novelty, claim strategy, filing entity, and experimental data.

### Task 2: Rewrite the claims around common-time feasibility

**Files:**
- Modify: `docs/patents/point-mass-time-reallocation-patent-draft.md`

- [ ] Rewrite independent method claim 1 so each graph edge exists only when all three axes admit the same duration and its edge weight is the minimum such duration.
- [ ] Define the single-axis two-order equations, retain every real nonnegative-duration solution, and select the smallest duration rather than an order-preferred or longer solution.
- [ ] Define the synchronous feasible-time set with explicit switch-time and actual-acceleration bounds.
- [ ] State that the maximum of independent axis minima is only a lower bound and becomes the edge time only after a common-feasibility test.
- [ ] Rewrite graph-search claims as layered dynamic programming with predecessor and validated edge-parameter storage.
- [ ] Align system, device, and storage-medium claims with the corrected method.
- [ ] Remove claims based on unsupported speed-reversal checks, position-direction checks, arbitrary infeasibility penalties, and recomputation with a different parameter model.

### Task 3: Rewrite the description and proofs

**Files:**
- Modify: `docs/patents/point-mass-time-reallocation-patent-draft.md`

- [ ] Replace the abstract with a version no longer than 300 Han characters and consistent with claim 1.
- [ ] State the exact second-order point-mass model, inputs, admissible controls, objective, and exclusions.
- [ ] Derive the two-stage endpoint equations and the single-axis quadratic coefficients, including linear and static-axis degeneracies.
- [ ] Prove why both acceleration orders and all feasible roots must be compared.
- [ ] Define common-time feasibility and explain a finite-order constrained solution procedure without assuming monotonic feasibility or using an unjustified binary search.
- [ ] Prove the layered-DAG dynamic-programming recurrence by the Bellman principle.
- [ ] Explain trajectory reconstruction, switch continuity, waypoint position/velocity continuity, acceleration discontinuity, and inclusion of the terminal endpoint.
- [ ] Provide pseudocode matching the corrected theory while avoiding concrete software interfaces.
- [ ] Keep the implementation examples illustrative and avoid fabricating numerical results.

### Task 4: Reduce simulation disclosure to a true placeholder

**Files:**
- Modify: `docs/patents/point-mass-time-reallocation-patent-draft.md`

- [ ] Retain only the heading “仿真验证说明（预留）”.
- [ ] State that EGO-Planner is a third-party trajectory frontend used solely to supply a comparison trajectory and is not part of the invention.
- [ ] Leave parameter-table, result-table, figure, result-analysis, and simulation-conclusion placeholders without claims or invented outcomes.

### Task 5: Verify the corrected application package

**Files:**
- Verify: `docs/patents/point-mass-time-reallocation-patent-draft.md`
- Verify: `docs/patents/point-mass-code-conformance-ledger.md`
- Verify: `docs/patents/point-mass-patent-review-report.md`

- [ ] Run a symbolic/numerical residual check for the stated single-axis equations and fixed-common-time equations.
- [ ] Count Han characters and confirm at least 20,000 in the patent draft.
- [ ] Count abstract Han characters and confirm no more than 300.
- [ ] Confirm claims are consecutively numbered and all formula symbols are defined.
- [ ] Search the patent for implementation-status language and concrete code identifiers.
- [ ] Confirm the only occurrence of EGO-Planner is in the reserved simulation section.
- [ ] Review the final diff, confirm the unrelated untracked directory remains untouched, and commit the corrected documents.
