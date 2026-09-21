# No-Figures Patent Disclosure Variant Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create a separate Markdown copy of the point-mass patent disclosure with all requested carrier, figure, simulation-placeholder, and proof-section content removed while preserving the original source and the remaining technical method.

**Architecture:** Duplicate the reviewed disclosure once, then use heading-bounded edits so each requested section is removed without rewriting retained theory. Apply a final reference-cleanup pass and validate the result through exact forbidden-content, retained-section, Markdown-structure, and source-integrity assertions.

**Tech Stack:** Markdown, Git, Python 3 standard library, `rg`, `apply_patch`.

## Global Constraints

- The original `docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.md` must remain byte-identical to its committed version.
- Create only `docs/patents/point-mass-trajectory-reconstruction-technical-disclosure-no-figures.md` as the disclosure deliverable.
- Do not generate or modify any DOCX.
- Delete all electronic-device, processor-instruction, memory, computer-readable-storage-medium, and program-carrier descriptions.
- Delete all patent-figure content and figure-number references, but retain mathematical graph-theory terms such as “分层图”“有向图”“图节点” and “图搜索”.
- Delete the complete simulation-placeholder section.
- Delete the complete sections titled “最优控制结构及候选完备性”, “固定时间消元的充分性与退化分支”, “共同可行集合求解的完备性与精度”, and “分层图动态规划与全局最优性证明”.
- Preserve all other formulas, pseudocode, implementation example, and beneficial-effect content except the minimum wording changes needed to remove dangling references.
- Do not add editorial notes stating that content was removed or will be provided later.
- Do not stage, restore, or modify the user-edited disclosure DOCX, its Word lock file, the Figure 1 files, or `swarm-playground/main_ws/.github/`.

---

### Task 1: Create and verify the no-figures Markdown disclosure

**Files:**
- Read: `docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.md`
- Read: `docs/superpowers/specs/2026-09-22-patent-disclosure-no-figures-design.md`
- Create: `docs/patents/point-mass-trajectory-reconstruction-technical-disclosure-no-figures.md`

**Interfaces:**
- Consumes: the complete reviewed disclosure and the exact deletion boundaries in the approved design.
- Produces: a standalone Markdown disclosure with the remaining text in its original order and no dependency on the source file at read time.

- [x] **Step 1: Record source integrity and verify the target is initially absent**

Run:

```bash
git show HEAD:docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.md | sha256sum
test ! -e docs/patents/point-mass-trajectory-reconstruction-technical-disclosure-no-figures.md
```

Expected: a source hash is printed and `test` exits with status 0.

- [x] **Step 2: Copy the source and remove electronic-device carrier content**

Copy the source to the exact target path without overwriting an existing file. In the copy only:

- change the technology-field sentence ending in “方法、系统、电子设备及计算机可读存储介质” so that it ends at “生成三维质点轨迹的方法”；
- delete the complete paragraph beginning “本发明还提供一种电子设备”；
- delete the complete paragraph beginning “电子设备的处理器可以是”。

Use `apply_patch` for all content edits after the initial file copy.

- [x] **Step 3: Remove all figure and simulation content**

In the copy only:

- replace “下面结合附图和数学推导对本发明作进一步说明” with “下面结合数学推导对本发明作进一步说明”；
- delete the complete paragraph beginning “图3可绘制为”；
- replace “图1所示总体流程可以用如下伪代码表示” with “本发明的总体流程可以用如下伪代码表示”；
- delete from `## 附图说明` through the line immediately before `## 仿真验证与效果说明（预留）`；
- delete `## 仿真验证与效果说明（预留）` and all following content through end of file.

After this step, the document must end with the final paragraph of `## 有益效果`.

- [x] **Step 4: Remove the four specified theory sections by heading boundary**

In the copy only, delete:

```text
from ### 最优控制结构及候选完备性
up to but not including ## 三轴共同时间重分配与可行性判定

from ### 固定时间消元的充分性与退化分支
up to but not including ### 共同可行集合求解的完备性与精度

from ### 共同可行集合求解的完备性与精度
up to but not including ## 分层图动态规划与全局最优性证明

from ## 分层图动态规划与全局最优性证明
up to but not including ## 轨迹解析重构及连续性说明
```

Do not delete dynamic-programming operations retained in other sections or in the pseudocode.

- [x] **Step 5: Remove dangling proof and figure references**

Read the complete copy from beginning to end. Remove or minimally rewrite only sentences that point exclusively to deleted material, including variants of:

```text
结合附图
如图所示
图1所示
图3可绘制
由后文证明
将在后文证明
参见附图
```

Do not replace deleted content with a summary, disclaimer, deletion note, or new proof.

- [x] **Step 6: Run exact automated validation**

Run:

```bash
python3 - <<'PY'
from pathlib import Path
import re
import subprocess

src = Path('docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.md')
dst = Path('docs/patents/point-mass-trajectory-reconstruction-technical-disclosure-no-figures.md')
s = src.read_bytes()
t = dst.read_text(encoding='utf-8')
committed = subprocess.check_output([
    'git', 'show', 'HEAD:docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.md'
])
assert s == committed, 'original Markdown changed'
assert t.startswith('# 质点轨迹重构算法专利技术交底书')

for forbidden in [
    '电子设备', '计算机可读存储介质', '处理器读取', '电子设备的处理器',
    '存储器中存储', '程序指令', '处理器', '程序单元',
    '物理硬件', '物理分离的硬件', '由递推证明', '逐层递推证明',
    '优化目标\\(J\\)', '## 附图说明',
    '## 仿真验证与效果说明（预留）', '图1所示', '图3可绘制',
    '示意图', '流程图', '对比图', '可视化布局',
    '### 最优控制结构及候选完备性',
    '### 固定时间消元的充分性与退化分支',
    '### 共同可行集合求解的完备性与精度',
    '## 分层图动态规划与全局最优性证明',
]:
    assert forbidden not in t, forbidden
assert not re.search(r'图[一二三四五六七八九十0-9]+', t)

h2 = re.findall(r'^## ([^#].*)$', t, flags=re.M)
assert len(h2) == 15, h2
for required in [
    '发明名称', '技术领域', '背景技术', '现有技术存在的问题', '发明目的',
    '技术方案总体说明', '数学模型与适用条件', '候选航点速度与分层图构建',
    '单轴时间最优转移原理及完整推导', '三轴共同时间重分配与可行性判定',
    '轨迹解析重构及连续性说明', '退化情形、数值容差和异常处理',
    '完整算法伪代码', '具体实施例', '有益效果',
]:
    assert required in h2, required

assert '算法一：航点速度搜索与质点轨迹时间重分配' in t
assert '算法二：单轴两阶段转移求解' in t
assert '算法三：求候选边的最小共同可行时间' in t
assert t.count('```') % 2 == 0
assert t.count('\\[') == t.count('\\]')
assert t.count('\\(') == t.count('\\)')
print(f'PASS: H2={len(h2)}, chars={len(t)}')
PY
```

Expected: `PASS: H2=15, chars=<positive number>`.

- [x] **Step 7: Perform semantic diff review**

Run:

```bash
git diff --no-index --word-diff=plain \
  docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.md \
  docs/patents/point-mass-trajectory-reconstruction-technical-disclosure-no-figures.md
```

Review the output and confirm every removal belongs to the approved deletion list or is a minimal dangling-reference edit. Confirm no retained equation, pseudocode branch, implementation example parameter, or beneficial-effect statement was accidentally changed.

- [x] **Step 8: Commit only the new Markdown and completed plan**

Run:

```bash
git diff --check
git status --short
git add docs/patents/point-mass-trajectory-reconstruction-technical-disclosure-no-figures.md \
        docs/superpowers/plans/2026-09-22-patent-disclosure-no-figures.md
git commit -m "docs: add no-figures patent disclosure variant"
```

Expected: the new Markdown and plan are committed; the user-edited DOCX, Word lock file, Figure 1 files, and unrelated `.github/` directory are not staged.
