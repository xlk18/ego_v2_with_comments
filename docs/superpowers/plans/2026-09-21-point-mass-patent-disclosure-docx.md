# Point-Mass Patent Disclosure DOCX Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce a single, agency-ready Chinese patent technical disclosure in editable DOCX form while preserving the complete, theory-correct point-mass trajectory reconstruction content.

**Architecture:** Reorganize the reviewed patent material into one disclosure-oriented Markdown source, excluding formal claims and internal audit material. Convert the source with Pandoc, then post-process the DOCX Open XML package with a small standard-library Python utility to enforce Chinese typography, A4 margins, heading hierarchy, table of contents, code styling, and page numbering; validate both semantic content and file readability.

**Tech Stack:** Markdown, LaTeX math, Pandoc, Python 3 standard library (`zipfile`, `xml.etree.ElementTree`, `subprocess`), LibreOffice headless conversion, Git.

## Global Constraints

- Work directly on the user-approved `main` branch.
- Do not modify the unrelated untracked `swarm-playground/main_ws/.github/` directory.
- Preserve at least 20,000 Han characters in the disclosure source.
- Do not include a formal claims section, key-protection-points section, or alternatives section.
- Do not include internal audit notes, code-conformance discussion, implementation status, source-code identifiers, middleware interfaces, or repository paths.
- Keep the three distinct optimality domains explicit: single-axis endpoint transfer, common-time segment transfer within the stated control family, and finite-candidate layered-graph optimization.
- Treat the maximum of independent axis minimum times only as a lower bound until common-time feasibility is verified.
- Retain a simulation-results placeholder without invented data or conclusions.
- The DOCX must remain editable and open successfully in LibreOffice.

---

### Task 1: Create the single technical-disclosure source

**Files:**
- Create: `docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.md`
- Read from: `docs/patents/point-mass-time-reallocation-patent-draft.md`
- Read from: `docs/patents/point-mass-patent-review-report.md`

**Interfaces:**
- Consumes: the theory-correct equations, definitions, proofs, pseudocode, and implementation example in the reviewed patent draft.
- Produces: one standalone Markdown document that Pandoc can convert without relying on the audit report or conformance ledger.

- [x] **Step 1: Write the document front matter and disclosure structure**

Create the source with the exact top-level title `质点轨迹重构算法专利技术交底书` and the following second-level sections in this order:

```text
发明名称
技术领域
背景技术
现有技术存在的问题
发明目的
技术方案总体说明
数学模型与适用条件
候选航点速度与分层图构建
单轴时间最优转移原理及完整推导
三轴共同时间重分配与可行性判定
分层图动态规划与全局最优性证明
轨迹解析重构及连续性说明
退化情形、数值容差和异常处理
完整算法伪代码
具体实施例
有益效果
附图说明
仿真验证与效果说明（预留）
```

- [x] **Step 2: Recast the reviewed material as disclosure prose**

Remove the original front-page disclaimer, abstract, formal claims, numbered patent paragraphs, “其他说明”, audit language, and code-state comparisons. Merge repeated derivations so each equation is introduced once, while retaining definitions and proof conditions.

- [x] **Step 3: Preserve the complete theory**

Ensure the source explicitly contains:

```text
p_dot = v, v_dot = a
a_minus < 0 < a_plus
both saturated acceleration orders and all feasible roots
T_lb = max(T_x*, T_y*, T_z*) as a lower bound only
T_edge = min(F_x intersect F_y intersect F_z)
0 <= tau <= T and 0 <= alpha <= 1
layered-DAG recurrence g(i+1,l) = min_k(g(i,k)+T_edge(i,k,l))
reconstruction from the same validated edge parameters
position/velocity continuity and allowed acceleration jumps
explicit inclusion of the final endpoint
```

- [x] **Step 4: Add the intentionally blank simulation section**

The final section must contain only neutral placeholders for simulation platform and parameters, original-versus-reconstructed trajectory plots, position/velocity/acceleration curves, completion time and computation time, result analysis, and conclusion. It may state that a third-party planner supplies only the comparison trajectory and is not part of the invention.

- [x] **Step 5: Run source-content checks**

Run:

```bash
python3 - <<'PY'
import re
from pathlib import Path
p = Path('docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.md')
t = p.read_text(encoding='utf-8')
han = len(re.findall(r'[\u3400-\u4dbf\u4e00-\u9fff\uf900-\ufaff]', t))
assert han >= 20000, han
for forbidden in ['## 权利要求书', '关键保护点', '可替代方案', 'calculateTperAxis', 'refineTperAxis', 'PointMassPathSearching', '当前实现', '程序缺陷']:
    assert forbidden not in t, forbidden
assert '## 仿真验证与效果说明（预留）' in t
print(f'PASS han={han}')
PY
```

Expected: `PASS han=<number at least 20000>`.

### Task 2: Add a reproducible DOCX formatter

**Files:**
- Create: `docs/patents/tools/build_disclosure_docx.py`
- Read from: `docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.md`
- Produce: `docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.docx`

**Interfaces:**
- Consumes: `Path` objects for source Markdown and output DOCX.
- Produces: an editable DOCX package with Pandoc-generated equations and a post-processed Open XML style layer.

- [x] **Step 1: Implement the command runner and Pandoc build**

Create a Python utility with these exact functions:

```python
def run(command: list[str]) -> None: ...
def build_with_pandoc(source: Path, output: Path) -> None: ...
def restyle_docx(path: Path) -> None: ...
def validate_docx(path: Path) -> None: ...
def main() -> int: ...
```

`build_with_pandoc` must invoke:

```text
pandoc SOURCE --from=markdown+tex_math_dollars --to=docx --toc --number-sections --metadata=lang:zh-CN --output=OUTPUT
```

- [x] **Step 2: Implement Word style updates**

In `restyle_docx`, open the DOCX as a ZIP package, parse `word/styles.xml`, and update or create run properties for these style IDs:

```text
Normal: eastAsia=宋体, ascii=Times New Roman, size=24 half-points
Title: eastAsia=黑体, ascii=Arial, size=36 half-points, bold, centered
Heading1: eastAsia=黑体, ascii=Arial, size=32 half-points, bold
Heading2: eastAsia=黑体, ascii=Arial, size=28 half-points, bold
Heading3: eastAsia=黑体, ascii=Arial, size=24 half-points, bold
SourceCode: eastAsia=等线, ascii=Courier New, size=20 half-points
```

Set the Normal paragraph style to 1.5-line spacing and a first-line indent of 480 twips. Set `word/document.xml` section margins to 1440 twips on all four sides and page size to A4 portrait (`w=11906`, `h=16838`).

- [x] **Step 3: Add centered page numbering**

Create `word/footer1.xml` containing a centered `PAGE` field; add its relationship to `word/_rels/document.xml.rels`, add its content-type override to `[Content_Types].xml`, and add a default footer reference to every final section property in `word/document.xml`. Repackage the files without changing unrelated Pandoc content.

- [x] **Step 4: Implement package validation**

`validate_docx` must verify that the ZIP contains:

```text
[Content_Types].xml
word/document.xml
word/styles.xml
word/footer1.xml
word/_rels/document.xml.rels
```

It must assert the presence of the Chinese title, the simulation-placeholder heading, the `PAGE` field, A4 dimensions, 1440-twip margins, and all required style IDs before printing the output path.

### Task 3: Build and visually inspect the DOCX

**Files:**
- Generate: `docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.docx`
- Generate temporarily: `/tmp/point-mass-disclosure-preview.pdf`

**Interfaces:**
- Consumes: the Markdown source and formatting utility from Tasks 1 and 2.
- Produces: the final DOCX and a temporary PDF used only for inspection.

- [ ] **Step 1: Generate the DOCX**

Run:

```bash
python3 docs/patents/tools/build_disclosure_docx.py
```

Expected: prints the absolute DOCX output path and exits with status 0.

- [ ] **Step 2: Verify LibreOffice can load and export it**

Run:

```bash
preview_dir=$(mktemp -d)
libreoffice --headless --convert-to pdf --outdir "$preview_dir" docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.docx
test -s "$preview_dir/point-mass-trajectory-reconstruction-technical-disclosure.pdf"
pdfinfo "$preview_dir/point-mass-trajectory-reconstruction-technical-disclosure.pdf" | rg '^Pages:'
```

Expected: conversion succeeds, the PDF is non-empty, and `Pages:` reports a positive page count.

- [ ] **Step 3: Inspect representative pages**

Render the first page, a formula-heavy middle page, a pseudocode page, and the final simulation-placeholder page to PNG using `pdftoppm`; inspect them for title placement, Chinese glyphs, heading hierarchy, clipped equations, broken tables, and footer page numbers. If any defect is present, adjust the formatter and rebuild.

### Task 4: Final semantic and package verification

**Files:**
- Verify: `docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.md`
- Verify: `docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.docx`
- Verify: `docs/patents/tools/build_disclosure_docx.py`

**Interfaces:**
- Consumes: all final deliverables.
- Produces: verification evidence and a committed disclosure package.

- [ ] **Step 1: Verify formulas against symbolic identities**

Use SymPy to confirm the single-axis quadratic coefficients and the fixed-common-time synchronization coefficients reduce to zero residual, and verify a symmetric accelerate-then-decelerate example reaches the requested endpoint.

- [ ] **Step 2: Verify document scope and counts**

Check that the source contains at least 20,000 Han characters, all 18 required sections in order, no formal claims section, no excluded headings, no internal-development wording, no concrete source identifiers, and only neutral simulation placeholders.

- [ ] **Step 3: Verify DOCX/source correspondence**

Extract `word/document.xml` text from the DOCX and confirm every second-level source heading appears in order. Confirm the DOCX contains editable OMML math elements and the pseudocode text.

- [ ] **Step 4: Review the final diff and commit**

Run `git diff --check`, verify `swarm-playground/main_ws/.github/` remains untracked and unstaged, then commit only the disclosure source, DOCX, formatter, and completed plan:

```bash
git add docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.md \
        docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.docx \
        docs/patents/tools/build_disclosure_docx.py \
        docs/superpowers/plans/2026-09-21-point-mass-patent-disclosure-docx.md
git commit -m "docs: add point-mass patent disclosure"
```
