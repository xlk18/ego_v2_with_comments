# Patent Figure 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce a technically accurate black-and-white SVG flowchart and PNG preview for Figure 1 of the point-mass trajectory reconstruction patent disclosure.

**Architecture:** Author the diagram directly as deterministic SVG so Chinese labels, arrows, branches, and patent line-art styling remain exact and editable. Validate the SVG structure and required wording with XML/text checks, render it with Inkscape using the installed Noto Sans CJK SC font, and visually inspect the resulting PNG before presenting it to the user.

**Tech Stack:** SVG 1.1, XML, Python 3 standard library, Inkscape, Noto Sans CJK SC.

## Global Constraints

- Follow `docs/superpowers/specs/2026-09-22-patent-figure-1-design.md` exactly.
- Use a portrait, A4-friendly view box and black line art on a white background.
- Keep all text in simplified Chinese and all labels editable as SVG text.
- Do not use color, gradients, shadows, icons, rasterized text, code identifiers, repository paths, or third-party planner names.
- Treat the maximum of the independent axis minimum times only as the common-time lower bound.
- Only validated feasible edges may contribute parameters to the layered-DAG optimization and analytic reconstruction.
- Do not modify or remove the unrelated untracked Word lock file or `swarm-playground/main_ws/.github/`.

---

### Task 1: Draw and validate the editable SVG

**Files:**
- Create: `docs/patents/figures/figure-1-overall-flow.svg`
- Read: `docs/superpowers/specs/2026-09-22-patent-figure-1-design.md`
- Read: `docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.md:512-609`

**Interfaces:**
- Consumes: the approved Figure 1 node order, branch semantics, and patent drawing conventions.
- Produces: a standalone SVG with reusable CSS classes, one arrow marker, labeled process nodes, two decision branches, and a candidate-edge loop.

- [ ] **Step 1: Create the SVG canvas and reusable styles**

Create an SVG with `viewBox="0 0 1600 2700"`, a white background, and these reusable classes:

```svg
<style>
  .node { fill:#fff; stroke:#000; stroke-width:3; }
  .flow { fill:none; stroke:#000; stroke-width:3; marker-end:url(#arrow); }
  .loop { fill:none; stroke:#000; stroke-width:3; marker-end:url(#arrow); }
  .label { font-family:'Noto Sans CJK SC','Microsoft YaHei',sans-serif; font-size:32px; fill:#000; text-anchor:middle; }
  .small { font-size:28px; }
  .branch { font-family:'Noto Sans CJK SC','Microsoft YaHei',sans-serif; font-size:28px; fill:#000; }
</style>
```

Define a black arrowhead marker and use only SVG geometry/text elements; do not embed bitmap data.

- [ ] **Step 2: Draw the main vertical flow**

Place the following nodes on a single central axis with consistent spacing:

```text
开始
输入航点、参考方向、端点速度、候选速度幅值及各轴加速度边界
输入合法性与退化情形检查
构建各航点的候选速度层
枚举相邻层候选节点对
求解三个坐标轴的单轴最短转移时间
取各轴最短时间的最大值作为公共时间下界
三轴共同时间是否可行？
相邻层候选边是否全部处理完成？
在分层有向无环图上执行动态规划
回溯获得最优候选速度序列
依据已验证的同一组边参数进行分段解析重构
补入终端状态并输出位置、速度、加速度及时间序列
结束
```

Use rounded terminators for `开始/结束`, parallelograms for input/output, rectangles for processing, and diamonds for the two questions.

- [ ] **Step 3: Add feasibility branches and the candidate-edge loop**

From `三轴共同时间是否可行？`:

```text
是 -> 保存边持续时间、切换时刻和缩放系数
否 -> 删除对应有向边
```

Merge both branches before `相邻层候选边是否全部处理完成？`. From that decision, label the downward branch `是`; label the return branch `否` and route it outside the left side of all nodes back to `枚举相邻层候选节点对`. Keep loop lines outside node boundaries and avoid arrow crossings.

- [ ] **Step 4: Run structural and content validation**

Run:

```bash
python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET

p = Path('docs/patents/figures/figure-1-overall-flow.svg')
t = p.read_text(encoding='utf-8')
root = ET.fromstring(t)
assert root.tag.endswith('svg')
assert root.attrib['viewBox'] == '0 0 1600 2700'
for phrase in [
    '开始', '输入合法性与退化情形检查', '构建各航点的候选速度层',
    '枚举相邻层候选节点对', '单轴最短转移时间',
    '最大值作为公共时间下界', '三轴共同时间是否可行',
    '删除对应有向边', '保存边持续时间、切换时刻和缩放系数',
    '分层有向无环图上执行动态规划', '回溯获得最优候选速度序列',
    '已验证的同一组边参数', '补入终端状态', '结束'
]:
    assert phrase in t, phrase
for forbidden in ['EGO-Planner', 'calculateTperAxis', 'refineTperAxis', 'data:image', '#ff', '#00', '#cc']:
    assert forbidden not in t, forbidden
assert t.count('marker-end:url(#arrow)') >= 15
print('SVG PASS')
PY
```

Expected: `SVG PASS`.

- [ ] **Step 5: Commit the SVG**

```bash
git add docs/patents/figures/figure-1-overall-flow.svg
git commit -m "docs: draw patent figure 1 flowchart"
```

### Task 2: Render and inspect the PNG preview

**Files:**
- Read: `docs/patents/figures/figure-1-overall-flow.svg`
- Create: `docs/patents/figures/figure-1-overall-flow-preview.png`

**Interfaces:**
- Consumes: the validated editable SVG from Task 1.
- Produces: a 1600-pixel-wide PNG preview for user review without replacing the SVG source.

- [ ] **Step 1: Render the SVG with Inkscape**

Run:

```bash
inkscape docs/patents/figures/figure-1-overall-flow.svg \
  --export-type=png \
  --export-width=1600 \
  --export-filename=docs/patents/figures/figure-1-overall-flow-preview.png
```

Expected: Inkscape exits successfully and creates a nonempty PNG.

- [ ] **Step 2: Verify file dimensions and format**

Run:

```bash
identify docs/patents/figures/figure-1-overall-flow-preview.png
```

Expected: reports PNG format, width `1600`, and a positive portrait height.

- [ ] **Step 3: Visually inspect the preview**

Open the PNG and verify:

```text
all Chinese labels are legible and contained within their nodes
all main arrows point downward
the 是/否 labels sit next to the correct branches
the left return loop reconnects to candidate-node enumeration
no arrow crosses a box or another label
all shapes are black and white
the common-time lower-bound wording is present
the final reconstruction wording refers to the same validated edge parameters
```

If any item fails, adjust the SVG, rerun Task 1 Step 4, and render again.

- [ ] **Step 4: Verify repository scope and commit the preview**

Run:

```bash
git diff --check
git status --short
```

Expected: only the PNG preview is new or modified; the existing unrelated Word lock file and `swarm-playground/main_ws/.github/` remain untracked and unstaged.

Then run:

```bash
git add docs/patents/figures/figure-1-overall-flow-preview.png
git commit -m "docs: add patent figure 1 preview"
```
