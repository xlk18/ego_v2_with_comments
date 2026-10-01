# LaTeX Patent TOC and Running-Header Removal Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the patent PDF table of contents and running section headers while retaining centered page numbers at the bottom of every page.

**Architecture:** Make the smallest possible change to the standalone XeLaTeX source by selecting the standard `plain` page style and removing the three TOC-only lines. Rebuild with the existing verified LaTeX Workshop-equivalent recipe, inspect representative pages and PDF text geometry, then push the reviewed commit to `origin/main`.

**Tech Stack:** CTeX/XeLaTeX, latexmk, Poppler (`pdfinfo`, `pdftotext`, `pdftoppm`), Git.

## Global Constraints

- Delete the table of contents completely.
- Delete every page's top running section title.
- Retain page numbers centered at the bottom.
- Do not restore any previously removed technical section.
- Do not modify prose, formulas, five pseudocode blocks, measured parameters, or the two result images.
- Do not modify the Markdown, Word documents, image files, or unrelated user worktree files.
- Keep the existing XeLaTeX/latexmk/`build/` workflow unchanged.
- Push only reviewed commits to `origin/main`.

---

### Task 1: Remove the table of contents and running headers

**Files:**
- Modify: `docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.tex`
- Generate but do not commit: `docs/patents/build/point-mass-trajectory-reconstruction-technical-disclosure.pdf`

**Interfaces:**
- Consumes: the reviewed standalone CTeX document at commit `0ae72ce` and the existing latexmk/XeLaTeX recipe.
- Produces: a native LaTeX document with no TOC or running headings and a rebuilt PDF using bottom-centered page numbers.

- [ ] **Step 1: Record content and preservation baselines**

Run:

```bash
tex=docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.tex
sha256sum "$tex" \
  docs/patents/point-mass-trajectory-reconstruction-technical-disclosure-no-figures.md \
  docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.docx \
  docs/patents/figures/test-pm-rviz-result.png \
  docs/patents/figures/test-pm-trajectory-curves.png
test "$(rg -c '^\\section\{' "$tex")" -eq 15
test "$(rg -c '^\\begin\{lstlisting\}' "$tex")" -eq 5
test "$(rg -c '^\\includegraphics' "$tex")" -eq 2
```

Expected: all hashes are recorded; the source contains 15 sections, five listings, and two images.

- [ ] **Step 2: Apply the minimal page-structure change**

Use `apply_patch` to make exactly these structural changes:

```tex
% Remove from the preamble:
\setcounter{tocdepth}{2}

% Change the document opening from:
\begin{document}
\maketitle
\tableofcontents
\clearpage

% To:
\begin{document}
\pagestyle{plain}
\maketitle
```

Expected: no `\tableofcontents`, TOC depth, or TOC-only forced page break remains; `\pagestyle{plain}` appears before the title.

- [ ] **Step 3: Verify the source change is content-neutral**

Run:

```bash
tex=docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.tex
! rg -n '\\tableofcontents|tocdepth' "$tex"
test "$(rg -c '^\\pagestyle\{plain\}$' "$tex")" -eq 1
test "$(rg -c '^\\section\{' "$tex")" -eq 15
test "$(rg -c '^\\begin\{lstlisting\}' "$tex")" -eq 5
test "$(rg -c '^\\includegraphics' "$tex")" -eq 2
rg -n '363|10\.86|0\.0226|-0\.6082|4\.6077' "$tex"
git diff --check -- "$tex"
```

Expected: only the intended page-structure lines changed and all technical-content invariants remain present.

- [ ] **Step 4: Force a clean-equivalent XeLaTeX rebuild**

Run:

```bash
cd docs/patents
latexmk -g -xelatex -synctex=1 -interaction=nonstopmode -file-line-error \
  -outdir=build point-mass-trajectory-reconstruction-technical-disclosure.tex
```

Expected: latexmk exits 0 and rewrites the PDF, log, auxiliary files, and SyncTeX data under `docs/patents/build/`.

- [ ] **Step 5: Check PDF text, images, logs, and page-number geometry**

Run:

```bash
pdf=docs/patents/build/point-mass-trajectory-reconstruction-technical-disclosure.pdf
log=docs/patents/build/point-mass-trajectory-reconstruction-technical-disclosure.log
pdfinfo "$pdf" | rg '^(Pages|Page size):'
pdftotext -layout "$pdf" /tmp/patent-no-toc.txt
! rg -n '^\s*目录\s*$' /tmp/patent-no-toc.txt
for term in '发明名称' '完整算法伪代码' '具体示例' '算法增益效果' '363' '10.86' '0.0226' '4.6077'; do
  rg -q -F "$term" /tmp/patent-no-toc.txt
done
test "$(pdfimages -list "$pdf" | awk 'NR>2 && $3=="image" {n++} END {print n+0}')" -ge 2
! rg -n '(^! |Undefined control sequence|LaTeX Error|Package .* Error|Missing character|Citation .* undefined|Reference .* undefined|Overfull|Underfull)' "$log"
pdftotext -bbox-layout "$pdf" /tmp/patent-no-toc-bbox.html
```

Parse `/tmp/patent-no-toc-bbox.html` and assert on every page after the first that at least one page-number word consists only of digits, has horizontal center within 20 points of the page center, and has vertical center `(yMin + yMax) / 2` within the bottom 60 points. Also confirm no section-title text appears in the top 50 points of representative body pages.

Expected: A4 PDF, no contents heading, all required content and images present, clean compiler log, and numeric page labels with horizontal centers within 20 points of the page center and vertical centers within the bottom 60 points, matching the user-approved standard `plain` footer; no section-title text in the top 50 points of representative body pages.

- [ ] **Step 6: Render and visually inspect representative pages**

Render the title/first-body page, one formula-heavy page, one pseudocode page, and the result-image page:

```bash
render_dir=$(mktemp -d /tmp/patent-no-header-render.XXXXXX)
pdf=docs/patents/build/point-mass-trajectory-reconstruction-technical-disclosure.pdf
page_count=$(pdfinfo "$pdf" | awk '/^Pages:/ {print $2}')
find_page() {
  term=$1
  for page in $(seq 1 "$page_count"); do
    if pdftotext -f "$page" -l "$page" -layout "$pdf" - | rg -q -F "$term"; then
      printf '%s\n' "$page"
      return 0
    fi
  done
  return 1
}
formula_page=$(find_page '端点方程与时间消元')
pseudocode_page=$(find_page '完整算法伪代码')
image_page=$(find_page '重构轨迹的可视化结果')
for page in 1 "$formula_page" "$pseudocode_page" "$image_page"; do
  pdftoppm -png -r 120 -f "$page" -singlefile \
    "$pdf" \
    "$render_dir/page-$page"
done
```

Open all four PNGs with the local image viewer.

Expected: no page has a top running title; page numbers are bottom-centered; title, formulas, listings, figures, and captions are unclipped and readable.

- [ ] **Step 7: Commit the source-only modification**

Run:

```bash
git diff --check
git diff --cached --quiet
git add docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.tex
git diff --cached --check
git diff --cached --name-only
git commit -m "docs: remove patent contents and running headers"
```

Expected: the commit contains only the `.tex` source; ignored PDF/build artifacts and all user-owned files remain outside the commit.

---

### Task 2: Review and push the adjusted patent

**Files:**
- No additional source files expected.
- Remote target: `origin/main`.

**Interfaces:**
- Consumes: Task 1's reviewed source commit and verified PDF.
- Produces: an updated `origin/main` containing the no-TOC, no-running-header LaTeX source.

- [ ] **Step 1: Review the outgoing commit and worktree state**

Run:

```bash
git fetch origin main
git log --oneline origin/main..HEAD
git diff --stat origin/main..HEAD
git status --short
```

Expected: the outgoing range contains the design, implementation plan, and reviewed source change only; the user's modified Word document, lock file, and unrelated untracked directory remain unstaged.

- [ ] **Step 2: Push to the existing remote branch**

Run:

```bash
git push origin main
```

Expected: the push fast-forwards `origin/main`.

- [ ] **Step 3: Verify direct remote equality**

Run:

```bash
local_sha=$(git rev-parse HEAD)
tracking_sha=$(git rev-parse origin/main)
remote_sha=$(git ls-remote origin refs/heads/main | awk '{print $1}')
test "$local_sha" = "$tracking_sha"
test "$local_sha" = "$remote_sha"
git status --short --branch
```

Expected: local, tracking, and direct remote SHAs are identical; only the preserved user worktree entries remain.
