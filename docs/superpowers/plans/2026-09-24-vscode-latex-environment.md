# VS Code–LaTeX Writing Environment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a complete VS Code–LaTeX environment on the existing TeX Live 2019 installation and convert the current Chinese patent disclosure Markdown into a verified, independently compilable XeLaTeX document and PDF.

**Architecture:** Keep the operating system's TeX Live 2019 as the only TeX distribution, add the missing command-line tools through APT, and use LaTeX Workshop as the editor/build integration. Store reusable editor behavior in VS Code user settings, project-specific output and extension recommendations under `.vscode/`, and keep the generated LaTeX source beside the existing patent source with all build artifacts isolated under `docs/patents/build/`.

**Tech Stack:** VS Code 1.127, LaTeX Workshop, LTeX+, TeX Live 2019, XeLaTeX, latexmk, Biber/BibTeX, ChkTeX, latexindent, TeXcount, Pandoc 2.5, CTeX/XeCJK.

## Global Constraints

- Preserve TeX Live 2019; do not install a second TeX distribution or replace the system default.
- Preserve existing VS Code user settings and merge only LaTeX-related keys.
- Do not overwrite either existing Markdown or Word disclosure document.
- Keep every mathematical expression as editable LaTeX; do not rasterize formulas.
- Preserve all technical prose, heading order, pseudocode, parameters, measured results, and both existing result images.
- Use XeLaTeX as the default engine and place generated artifacts under a `build/` directory.
- Do not modify or stage the user's unrelated Word changes, lock file, or `swarm-playground/main_ws/.github/` directory.

---

### Task 1: Install and verify the editor extensions and TeX tools

**Files:**
- No repository files changed.
- External state: VS Code extension directory under `~/.vscode/extensions/`.
- External state: system packages managed by APT.

**Interfaces:**
- Consumes: Existing `/usr/bin/xelatex`, `/usr/bin/pdflatex`, `/usr/bin/bibtex`, `/usr/bin/synctex`, and TeX Live 2019.
- Produces: `latexmk`, `biber`, `chktex`, `latexindent`, and `texcount` commands plus the `James-Yu.latex-workshop` and `ltex-plus.vscode-ltex-plus` extensions.

- [ ] **Step 1: Capture the pre-install baseline**

Run:

```bash
code --version
code --list-extensions | sort
for tool in xelatex latexmk biber bibtex chktex latexindent texcount; do
  command -v "$tool" || true
done
```

Expected: VS Code 1.127 and XeLaTeX/BibTeX are present; the two target extensions and one or more auxiliary tools are absent.

- [ ] **Step 2: Install the VS Code extensions**

Run:

```bash
code --install-extension James-Yu.latex-workshop --force
code --install-extension ltex-plus.vscode-ltex-plus --force
```

Expected: each command ends with a successful installation message. If the Marketplace rejects LTeX+ because of platform compatibility, record the exact message and continue with LaTeX Workshop; do not substitute an unreviewed extension.

- [ ] **Step 3: Install the missing TeX utilities without replacing TeX Live**

Run:

```bash
sudo apt-get update
sudo apt-get install -y latexmk biber chktex texlive-extra-utils texlive-latex-extra texlive-fonts-recommended texlive-lang-chinese fonts-noto-cjk
sudo mktexlsr
```

Expected: APT reports successful installation, retains TeX Live 2019 packages, and `mktexlsr` refreshes the filename database. If non-interactive sudo access is unavailable, stop this task and report that the user must run these exact three commands in a terminal; do not install a second TeX distribution.

- [ ] **Step 4: Verify the installed command-line interface**

Run:

```bash
code --list-extensions | rg -i '^(James-Yu\.latex-workshop|ltex-plus\.vscode-ltex-plus)$'
for tool in xelatex latexmk biber bibtex chktex latexindent texcount synctex; do
  command -v "$tool"
done
kpsewhich ctexart.cls
kpsewhich xeCJK.sty
latexmk -v | head -n 2
biber --version
chktex --version | head -n 1
latexindent --version
texcount -version
```

Expected: every command exits with status 0, `ctexart.cls` and `xeCJK.sty` resolve to files under `/usr/share/texlive`, and the installed tools report versions compatible with TeX Live 2019.

---

### Task 2: Configure VS Code for reusable XeLaTeX authoring

**Files:**
- Modify: `/home/yyf/.config/Code/User/settings.json`
- Modify: `.vscode/settings.json`
- Create: `.vscode/extensions.json`
- Create: `docs/patents/latex-environment-guide.md`

**Interfaces:**
- Consumes: Tool paths and extension IDs verified in Task 1.
- Produces: a `latexmk (xelatex)` recipe, save-time builds, internal PDF preview, SyncTeX, formatter integration, project extension recommendations, and a user-facing operating guide.

- [ ] **Step 1: Back up and validate the existing user settings before editing**

Run:

```bash
python3 -m json.tool /home/yyf/.config/Code/User/settings.json >/dev/null
cp -a /home/yyf/.config/Code/User/settings.json /home/yyf/.config/Code/User/settings.json.pre-latex
```

Expected: JSON validation succeeds and the backup has the same SHA-256 hash as the original.

- [ ] **Step 2: Merge the LaTeX Workshop configuration into user settings**

Add the following top-level keys without removing or changing existing non-LaTeX keys. For LaTeX Workshop 10.19.0, remove the unsupported `latex-workshop.latex.clean.enabled` and `latex-workshop.latex.clean.onFailBuild.enabled` keys if present; use `latex-workshop.latex.autoClean.run` instead. Disable build magic comments so a source-level `% !TeX program = xelatex` does not bypass the configured `latexmk` recipe and output directory.

```json
"latex-workshop.latex.autoBuild.run": "onSave",
"latex-workshop.latex.build.enableMagicComments": false,
"latex-workshop.latex.outDir": "%DIR%/build",
"latex-workshop.latex.recipe.default": "latexmk (xelatex)",
"latex-workshop.view.pdf.viewer": "tab",
"latex-workshop.synctex.afterBuild.enabled": true,
"latex-workshop.latex.autoClean.run": "onFailed",
"latex-workshop.latex.tools": [
  {
    "name": "latexmk-xelatex",
    "command": "latexmk",
    "args": [
      "-xelatex",
      "-synctex=1",
      "-interaction=nonstopmode",
      "-file-line-error",
      "-outdir=%OUTDIR%",
      "%DOC%"
    ]
  },
  {
    "name": "latexmk-clean",
    "command": "latexmk",
    "args": ["-outdir=%OUTDIR%", "-c", "%DOC%"]
  }
],
"latex-workshop.latex.recipes": [
  {
    "name": "latexmk (xelatex)",
    "tools": ["latexmk-xelatex"]
  }
],
"latex-workshop.formatting.latex": "latexindent",
"[latex]": {
  "editor.defaultFormatter": "James-Yu.latex-workshop",
  "editor.formatOnSave": true,
  "editor.wordWrap": "on"
}
```

Use `apply_patch` for the edit, then run:

```bash
python3 -m json.tool /home/yyf/.config/Code/User/settings.json >/dev/null
```

Expected: the merged settings remain valid JSON and all pre-existing non-LaTeX keys remain present.

- [ ] **Step 3: Add project-specific settings and extension recommendations**

Change `.vscode/settings.json` to:

```json
{
  "cmake.sourceDirectory": "/home/yyf/EGO-Planner-v2/swarm-playground/main_ws/src/VIO",
  "latex-workshop.latex.outDir": "%DIR%/build",
  "latex-workshop.latex.recipe.default": "latexmk (xelatex)",
  "latex-workshop.latex.build.enableMagicComments": false,
  "latex-workshop.view.pdf.viewer": "tab",
  "latex-workshop.latex.autoBuild.run": "onSave"
}
```

Create `.vscode/extensions.json`:

```json
{
  "recommendations": [
    "James-Yu.latex-workshop",
    "ltex-plus.vscode-ltex-plus"
  ]
}
```

Run:

```bash
python3 -m json.tool .vscode/settings.json >/dev/null
python3 -m json.tool .vscode/extensions.json >/dev/null
```

Expected: both workspace files are valid JSON.

- [ ] **Step 4: Write the operating guide**

Create `docs/patents/latex-environment-guide.md` with these exact sections and concrete commands:

````markdown
# VS Code–LaTeX 使用说明

## 打开与编译

在 VS Code 中打开 `.tex` 文件，保存后由 `latexmk (xelatex)` 自动编译。执行命令面板中的 `LaTeX Workshop: View LaTeX PDF file` 可在标签页中查看 PDF。

命令行等价操作：

```bash
latexmk -xelatex -synctex=1 -interaction=nonstopmode -file-line-error -outdir=build 文件名.tex
```

## 正反向搜索

在 `.tex` 中执行 `SyncTeX from cursor` 跳转到 PDF；在内置 PDF 中按 Ctrl 并单击可返回源文件。

## 参考文献

BibLaTeX 文档使用 Biber，传统 BibTeX 文档使用 BibTeX。`latexmk` 会依据文档配置自动执行所需步骤。

## 格式化、检查与统计

```bash
mkdir -p build
latexindent -c build/ 文件名.tex > /tmp/formatted.tex
chktex -q 文件名.tex
texcount -inc -sum 文件名.tex
```

## 清理构建产物

```bash
latexmk -outdir=build -c 文件名.tex
```

## 常见问题

本环境固定使用系统 TeX Live 2019。若出版社模板要求更新宏包，应先核对模板要求，不要同时安装第二套 TeX Live；确需升级时再整体迁移发行版。
````

- [ ] **Step 5: Validate and commit the editor configuration**

Run:

```bash
python3 -m json.tool /home/yyf/.config/Code/User/settings.json >/dev/null
python3 -m json.tool .vscode/settings.json >/dev/null
python3 -m json.tool .vscode/extensions.json >/dev/null
git diff --check -- .vscode/settings.json .vscode/extensions.json docs/patents/latex-environment-guide.md
git add .vscode/settings.json .vscode/extensions.json docs/patents/latex-environment-guide.md
git commit -m "build: configure VS Code LaTeX workflow"
```

Expected: all validation commands pass and the commit contains only the three repository files; the user-level backup and settings are not added to Git.

---

### Task 3: Convert the patent disclosure to native XeLaTeX

**Files:**
- Consume without modification: `docs/patents/point-mass-trajectory-reconstruction-technical-disclosure-no-figures.md`
- Consume without modification: `docs/patents/figures/test-pm-rviz-result.png`
- Consume without modification: `docs/patents/figures/test-pm-trajectory-curves.png`
- Create: `docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.tex`

**Interfaces:**
- Consumes: the 15-section Markdown disclosure, native TeX math blocks, five pseudocode blocks, and two relative image references.
- Produces: one self-contained `ctexart` source file that builds with Task 2's `latexmk (xelatex)` recipe.

- [ ] **Step 1: Record source invariants before conversion**

Run:

```bash
source=docs/patents/point-mass-trajectory-reconstruction-technical-disclosure-no-figures.md
test "$(rg -c '^## ' "$source")" -eq 15
test "$(rg -c '^```text$' "$source")" -eq 5
test "$(rg -c '^!\[' "$source")" -eq 2
rg -n '363|10\.86|0\.0226|-0\.6082|4\.6077' "$source"
```

Expected: 15 second-level sections, five text pseudocode blocks, two image references, and every measured value is found.

- [ ] **Step 2: Generate a Pandoc body in a temporary file**

Run:

```bash
tmp_dir=$(mktemp -d /tmp/patent-latex-conversion.XXXXXX)
pandoc \
  --from=markdown+raw_tex+tex_math_dollars \
  --to=latex \
  --wrap=preserve \
  docs/patents/point-mass-trajectory-reconstruction-technical-disclosure-no-figures.md \
  -o "$tmp_dir/body.tex"
```

Expected: Pandoc exits successfully; `body.tex` contains the disclosure headings, display equations, five `verbatim` environments, and two `includegraphics` commands.

- [ ] **Step 3: Create the standalone XeLaTeX document**

Create `docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.tex` with this preamble, followed by the reviewed Pandoc body and `\end{document}`:

```tex
% !TeX program = xelatex
% !TeX encoding = UTF-8
\documentclass[UTF8,a4paper,12pt,fontset=fandol]{ctexart}

\usepackage{amsmath,amssymb,bm,mathtools}
\usepackage{geometry}
\usepackage{graphicx}
\usepackage{float}
\usepackage{xcolor}
\usepackage{listings}
\usepackage{enumitem}
\usepackage{booktabs,longtable,array}
\usepackage[hidelinks,unicode]{hyperref}
\usepackage{bookmark}

\geometry{top=2.54cm,bottom=2.54cm,left=3.0cm,right=2.6cm}
\setlength{\parindent}{2em}
\setlength{\parskip}{0.35em}
\linespread{1.35}
\setcounter{tocdepth}{2}
\graphicspath{{figures/}}
\hypersetup{
  pdftitle={质点轨迹重构算法专利技术交底书},
  pdfauthor={},
  pdfsubject={质点轨迹重构算法}
}

\lstdefinestyle{patentpseudo}{
  basicstyle=\ttfamily\small,
  columns=fullflexible,
  keepspaces=true,
  breaklines=true,
  breakatwhitespace=false,
  frame=single,
  rulecolor=\color{black!25},
  backgroundcolor=\color{black!2},
  showstringspaces=false
}

\title{质点轨迹重构算法专利技术交底书}
\author{}
\date{}

\begin{document}
\maketitle
\tableofcontents
\clearpage
```

During review, apply these deterministic transformations:

- remove the duplicated Pandoc-rendered top-level title;
- map Markdown `##` headings to `\section` and `###` headings to `\subsection`;
- replace the five `verbatim` blocks with `\begin{lstlisting}[style=patentpseudo]` and `\end{lstlisting}`;
- normalize image paths to `figures/test-pm-rviz-result.png` and `figures/test-pm-trajectory-curves.png`;
- keep each image and its Chinese caption in a `figure[H]` environment with `\centering` and width no greater than `0.95\textwidth`;
- preserve every display equation as native TeX and add no equation numbers that are absent from the Markdown source.

- [ ] **Step 4: Verify source-content parity before compiling**

Run:

```bash
tex=docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.tex
test "$(rg -c '^\\section\{' "$tex")" -eq 15
test "$(rg -c '^\\begin\{lstlisting\}' "$tex")" -eq 5
test "$(rg -c '^\\includegraphics' "$tex")" -eq 2
rg -n '363|10\.86|0\.0226|-0\.6082|4\.6077' "$tex"
rg -n '\\boldsymbol|\\widehat|\\lVert|\\operatorname' "$tex"
```

Expected: all counts and measured values match the Markdown source and the representative mathematical commands remain present.

- [ ] **Step 5: Commit the native LaTeX source**

Run:

```bash
git diff --check -- docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.tex
git add docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.tex
git commit -m "docs: convert trajectory disclosure to XeLaTeX"
```

Expected: the commit contains only the new `.tex` file and does not modify the Markdown or Word sources.

---

### Task 4: Compile, inspect, and verify the complete workflow

**Files:**
- Consume: `docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.tex`
- Generate but do not commit: `docs/patents/build/*`
- Generate: `docs/patents/build/point-mass-trajectory-reconstruction-technical-disclosure.pdf`

**Interfaces:**
- Consumes: Task 1 tools, Task 2 recipe semantics, and Task 3 source.
- Produces: a compiled PDF and evidence that the full VS Code-equivalent build, checks, formatter, and word counter work.

- [ ] **Step 1: Run the same XeLaTeX build used by VS Code**

Run:

```bash
cd docs/patents
mkdir -p build
latexmk -xelatex -synctex=1 -interaction=nonstopmode -file-line-error \
  -outdir=build point-mass-trajectory-reconstruction-technical-disclosure.tex
```

Expected: `latexmk` exits with status 0 and creates the PDF, log, auxiliary files, table of contents, and SyncTeX data under `docs/patents/build/`.

- [ ] **Step 2: Check the compiler log for substantive errors**

Run:

```bash
log=docs/patents/build/point-mass-trajectory-reconstruction-technical-disclosure.log
! rg -n '(^! |Undefined control sequence|LaTeX Error|Package .* Error|Missing character|Citation .* undefined|Reference .* undefined)' "$log"
```

Expected: no fatal error, undefined reference, missing character, or package error is reported. Inspect each overfull box; revise the source if text or formulas visibly leave the page.

- [ ] **Step 3: Inspect the PDF structure and content**

Run:

```bash
pdf=docs/patents/build/point-mass-trajectory-reconstruction-technical-disclosure.pdf
pdfinfo "$pdf" | rg '^(Pages|Page size):'
pdftotext -layout "$pdf" /tmp/patent-disclosure-latex.txt
rg -n '技术方案总体说明|完整算法伪代码|具体示例|算法增益效果' /tmp/patent-disclosure-latex.txt
rg -n '363|10\.86|0\.0226|-0\.6082|4\.6077' /tmp/patent-disclosure-latex.txt
pdfimages -list "$pdf" | tail -n +3
```

Expected: the PDF uses A4 pages; every named section and measured value is extractable; at least two embedded raster images are listed.

- [ ] **Step 4: Render representative pages for visual inspection**

Run:

```bash
render_dir=$(mktemp -d /tmp/patent-latex-render.XXXXXX)
pdftoppm -png -r 120 -f 1 -singlefile \
  docs/patents/build/point-mass-trajectory-reconstruction-technical-disclosure.pdf \
  "$render_dir/title"
pdftoppm -png -r 120 -f 2 -singlefile \
  docs/patents/build/point-mass-trajectory-reconstruction-technical-disclosure.pdf \
  "$render_dir/toc"
```

Then locate the two image pages with `pdftotext -f N -l N` and render them with the same command. Inspect title, contents, a formula-heavy page, a pseudocode page, and both result-image pages using the local image viewer.

Expected: Chinese glyphs render correctly, formulas are legible, code wraps inside the page, captions stay with images, and no content is clipped.

- [ ] **Step 5: Verify the auxiliary writing tools**

Run:

```bash
cd docs/patents
chktex -q point-mass-trajectory-reconstruction-technical-disclosure.tex || true
mkdir -p build
latexindent -c build/ point-mass-trajectory-reconstruction-technical-disclosure.tex \
  > /tmp/point-mass-disclosure-formatted.tex
test -s /tmp/point-mass-disclosure-formatted.tex
texcount -inc -sum point-mass-trajectory-reconstruction-technical-disclosure.tex
```

Expected: ChkTeX executes and its actionable warnings are reviewed; latexindent emits a nonempty formatted copy without modifying the source; TeXcount reports nonzero text, header, and formula counts.

- [ ] **Step 6: Run final repository and preservation checks**

Run:

```bash
git diff --check
git status --short
git diff --name-only HEAD -- \
  docs/patents/point-mass-trajectory-reconstruction-technical-disclosure-no-figures.md \
  docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.docx
```

Expected: `git diff --check` is clean; the Markdown source is unchanged; the user's pre-existing modified Word document, lock file, and unrelated untracked directory remain unstaged and untouched; `docs/patents/build/` is ignored by the existing root `build/` rule.

- [ ] **Step 7: Commit only any source corrections found during validation**

If Task 4 required LaTeX source or guide corrections, run:

```bash
git add docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.tex \
  docs/patents/latex-environment-guide.md
git commit -m "docs: verify LaTeX disclosure build"
```

Expected: the commit contains only reviewed source or guide corrections. If no corrections were required, do not create an empty commit.

---

### Task 5: Submit the completed patent source to the configured remote

**Files:**
- No new files created.
- Remote target: `origin` (`git@github.com:xlk18/ego_v2_with_comments.git`).
- Remote branch: `main`.

**Interfaces:**
- Consumes: all reviewed commits from Tasks 1–4 and the final whole-branch review.
- Produces: an updated `origin/main` containing the formal patent LaTeX source, environment configuration, usage guide, design, and implementation plan.

- [ ] **Step 1: Review exactly what will be pushed**

Run:

```bash
git fetch origin main
git log --oneline --decorate origin/main..HEAD
git diff --stat origin/main..HEAD
git status --short
```

Expected: the outgoing commit range includes the reviewed patent work; the user's modified Word document, lock file, and unrelated untracked directory remain outside commits.

- [ ] **Step 2: Push the reviewed branch**

Run:

```bash
git push origin main
```

Expected: Git reports that local `main` updates `origin/main` without a non-fast-forward error.

- [ ] **Step 3: Verify the remote branch**

Run:

```bash
test "$(git rev-parse HEAD)" = "$(git rev-parse origin/main)"
git status --short --branch
```

Expected: local `HEAD` and `origin/main` resolve to the same commit; only the pre-existing unstaged and untracked user files remain in the working tree.
