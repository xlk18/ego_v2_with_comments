# test_pm Example Results Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run the existing `point_mass_model/test_pm` ROS node, capture its real trajectory, create an RViz screenshot and trajectory-data plot, and replace the current disclosure example with the measured node example.

**Architecture:** Add a small ROS data-capture/plot utility and a dedicated RViz configuration, then execute the existing compiled node once to generate auditable CSV/JSON data and both images. Update only the current Markdown variant with source-derived parameters and measured results, keeping all output values traceable to the captured run.

**Tech Stack:** ROS Noetic, `rospy`, `nav_msgs/Path`, RViz, Python 3, NumPy, Matplotlib, CSV/JSON, ImageMagick, Markdown.

## Global Constraints

- Modify `docs/patents/point-mass-trajectory-reconstruction-technical-disclosure-no-figures.md` directly, as explicitly requested by the user.
- Do not modify or regenerate any DOCX.
- Use `swarm-playground/main_ws/src/planner/point_mass_trajectory_generation/src/test_point_mass_trajectory.cpp` as the only parameter source.
- Obtain trajectory point count and trajectory coordinates from an actual `/generated_trajectory` message published by a real `test_pm` run.
- Obtain computation time from the same run's `Trajectory calculation finished in ... seconds` log line.
- Construct sample time as `t_k = 0.03*k`; do not infer per-point time from identical `PoseStamped` headers.
- Do not claim velocity or acceleration curves from `nav_msgs/Path`, because the node publishes positions only.
- The RViz image must be a real screenshot, not a generated or reconstructed illustration.
- Preserve all disclosure content outside the “具体实施例/有益效果” title and example-body changes.
- Do not restore the deleted figure-description, simulation-placeholder, or proof sections.
- Do not stage, restore, or modify the user-edited disclosure DOCX, its lock file, Figure 1 files, or `swarm-playground/main_ws/.github/`.

---

### Task 1: Add reproducible trajectory capture and RViz configuration

**Files:**
- Create: `docs/patents/tools/capture_test_pm_example.py`
- Create: `docs/patents/tools/test-pm-example.rviz`
- Read: `swarm-playground/main_ws/src/planner/point_mass_trajectory_generation/src/test_point_mass_trajectory.cpp`

**Interfaces:**
- `wait_for_path(topic: str, timeout: float) -> nav_msgs.msg.Path`
- `path_samples(message: Path, dt: float) -> numpy.ndarray` returning columns `t,x,y,z`
- `circular_waypoints(count: int, radius: float, height: float) -> numpy.ndarray`
- `parse_compute_time(log_path: pathlib.Path) -> float`
- `write_outputs(samples, waypoints, compute_time, csv_path, summary_path, plot_path) -> None`
- CLI consumes `--topic`, `--node-log`, `--csv`, `--summary`, and `--plot` paths.

- [x] **Step 1: Create the capture-and-plot utility**

Implement a Python ROS node that waits for one nonempty `nav_msgs/Path`, validates `frame_id == "map"`, converts every pose position to `t,x,y,z` using `dt=0.03`, parses the computation-time line from the supplied node log, and writes:

```text
CSV header: t_s,x_m,y_m,z_m
JSON keys: node, topic, frame_id, sample_period_s, point_count,
           sampled_duration_s, computation_time_s, acceleration_upper_mps2,
           acceleration_lower_mps2, configured_circle_points,
           total_waypoints_including_start, radius_m, height_m,
           start_position_m, start_velocity_mps, fix_replan
```

The plot must use Matplotlib's noninteractive `Agg` backend and create a 1800-by-900-pixel PNG with two panels:

- left: three-dimensional published trajectory plus the source-defined start point and seven circular waypoints;
- right: `x(t)`, `y(t)`, and `z(t)` from the published Path.

Use `Noto Sans CJK SC`; label axes and units; show legends and grids; include actual point count and sampled duration in the title. Do not calculate or display velocity/acceleration.

- [x] **Step 2: Create the RViz configuration**

Create a minimal RViz config containing only:

```text
Displays panel
Grid display on XY plane
Axes display using the fixed frame
Path display named Generated trajectory
Path topic /generated_trajectory
Line style Billboards, width at least 0.08 m, high-contrast color
Global fixed frame map
Orbit view centered near (25,0,2), with the complete 50 m diameter trajectory visible
```

Keep the Displays panel visible so the screenshot proves the configured Path topic. Disable save prompts.

- [x] **Step 3: Run static checks**

Run:

```bash
python3 -m py_compile docs/patents/tools/capture_test_pm_example.py
python3 docs/patents/tools/capture_test_pm_example.py --help
rg -n '/generated_trajectory|Fixed Frame|Generated trajectory|rviz/Path|rviz/Grid|rviz/Axes' \
  docs/patents/tools/test-pm-example.rviz
```

Expected: Python compilation succeeds, CLI help exits 0, and every RViz requirement appears.

- [x] **Step 4: Commit Task 1**

```bash
git add docs/patents/tools/capture_test_pm_example.py \
        docs/patents/tools/test-pm-example.rviz
git commit -m "docs: add test_pm result capture tools"
```

### Task 2: Run test_pm and generate real result artifacts

**Files:**
- Create: `docs/patents/data/test-pm-generated-trajectory.csv`
- Create: `docs/patents/data/test-pm-run-summary.json`
- Create: `docs/patents/figures/test-pm-rviz-result.png`
- Create: `docs/patents/figures/test-pm-trajectory-curves.png`
- Read: `docs/patents/tools/capture_test_pm_example.py`
- Read: `docs/patents/tools/test-pm-example.rviz`

**Interfaces:**
- Consumes: the compiled `devel/lib/point_mass_model/test_pm` executable and Task 1 tools.
- Produces: one real Path data set, its measured summary, and two images from the same ROS run.

- [x] **Step 1: Start an isolated ROS run and capture the Path**

Use a temporary run directory and explicit PIDs. Source `swarm-playground/main_ws/devel/setup.bash`, start `roscore`, start `rosrun point_mass_model test_pm` with stdout/stderr redirected to the run log, and wait until the computation-time log line appears. Then run:

```bash
python3 docs/patents/tools/capture_test_pm_example.py \
  --topic /generated_trajectory \
  --node-log "$run_dir/test_pm.log" \
  --csv docs/patents/data/test-pm-generated-trajectory.csv \
  --summary docs/patents/data/test-pm-run-summary.json \
  --plot docs/patents/figures/test-pm-trajectory-curves.png
```

Expected: capture exits 0 after receiving a nonempty latched Path.

- [x] **Step 2: Start RViz and capture the real window**

While the same ROS master and node remain alive, start:

```bash
rviz -d docs/patents/tools/test-pm-example.rviz
```

Wait until the Path status is OK and the complete trajectory is visible. Use `wmctrl -lx` to locate the RViz window, size it to at least 1400-by-900 pixels, and use ImageMagick `import -window WINDOW_ID` to create:

```text
docs/patents/figures/test-pm-rviz-result.png
```

After capture, terminate only the explicitly recorded RViz, node, and roscore PIDs. Do not use broad process-kill commands.

- [x] **Step 3: Validate numerical artifacts**

Run a Python check that:

```text
CSV row count equals JSON point_count
first CSV time is 0
consecutive times differ by 0.03 within 1e-12
last time equals JSON sampled_duration_s
all coordinates are finite
frame_id is map
point_count is positive
computation_time_s is finite and positive
configured_circle_points is 7
total_waypoints_including_start is 8
radius_m is 25
height_m is 2
```

Use `identify` to verify both images are nonempty PNG files, the curve image is exactly 1800-by-900 pixels, and the RViz image is at least 1200 pixels wide and 750 pixels high.

- [x] **Step 4: Visually inspect both images**

Inspect at original detail. Confirm:

```text
RViz: complete trajectory, grid, axes, Displays panel, map fixed frame,
      Generated trajectory display and /generated_trajectory topic visible,
      no error status or obscuring window
Plot: 3D path and eight source-defined waypoint markers are visible;
      x/y/z curves, units, legend, point count and duration are legible;
      no invented velocity or acceleration curve appears
```

If an image fails, correct the RViz view or plotting utility and regenerate from the same captured CSV/JSON whenever the underlying data remain unchanged.

- [x] **Step 5: Commit Task 2 artifacts**

```bash
git add docs/patents/data/test-pm-generated-trajectory.csv \
        docs/patents/data/test-pm-run-summary.json \
        docs/patents/figures/test-pm-rviz-result.png \
        docs/patents/figures/test-pm-trajectory-curves.png
git commit -m "docs: capture test_pm example results"
```

### Task 3: Rewrite the example and verify the disclosure

**Files:**
- Modify: `docs/patents/point-mass-trajectory-reconstruction-technical-disclosure-no-figures.md:589-617`
- Read: `docs/patents/data/test-pm-run-summary.json`
- Read: `docs/patents/data/test-pm-generated-trajectory.csv`

**Interfaces:**
- Consumes: source-derived parameters and Task 2 measured results.
- Produces: the final Markdown with renamed sections, actual example narrative, and relative image links.

- [x] **Step 1: Rename both headings**

Apply exactly:

```text
## 具体实施例 -> ## 具体示例
## 有益效果 -> ## 算法增益效果
```

- [x] **Step 2: Replace the example body with the real node example**

Replace the current parameterized example with a self-contained description of:

- node/package/topic/frame and actual run environment;
- exact initial state, acceleration limits, seven-point circular formula and normalized reference direction;
- complete-search mode and 0.03 s sampling;
- solve, analytic sampling, Path conversion and latched publication;
- measured computation time, point count and sampled duration copied from JSON;
- what the RViz image establishes;
- what the position-data plot establishes and why it does not claim velocity/acceleration output.

Insert the two relative image links specified in the approved design directly below their respective explanatory paragraphs.

- [x] **Step 3: Validate document-to-data consistency**

Run a Python validator that reads the JSON and Markdown and asserts:

```text
headings 具体示例 and 算法增益效果 exist
old headings 具体实施例 and 有益效果 do not exist
both relative image links exist and resolve to nonempty PNG files
Markdown contains the exact measured point_count
Markdown contains computation_time_s rounded exactly as displayed in JSON
Markdown contains sampled_duration_s rounded exactly as displayed in JSON
all source parameters from the approved design are present
no 仿真验证与效果说明 or 附图说明 section is restored
```

Also verify all Markdown fences/math delimiters remain balanced and `git diff --check` passes.

- [x] **Step 4: Review scope and commit**

Confirm the original full Markdown is unchanged and the user-edited DOCX/lock, Figure 1 files, and `.github/` remain unstaged. Then commit only the updated Markdown and completed plan:

```bash
git add docs/patents/point-mass-trajectory-reconstruction-technical-disclosure-no-figures.md \
        docs/superpowers/plans/2026-09-22-test-pm-example-results.md
git commit -m "docs: add measured test_pm disclosure example"
```
