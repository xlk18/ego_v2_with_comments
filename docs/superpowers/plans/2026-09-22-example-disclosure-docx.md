# Measured-Example Disclosure DOCX Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Generate a reproducible, editable Chinese DOCX from the current measured-example disclosure Markdown, with correct formulas, a navigable 15-section TOC, and both result images embedded.

**Architecture:** Add a variant-specific builder that imports the already reviewed Open XML formatting functions without changing the original builder. The new builder supplies the current Markdown, resource path, image sizing, and variant-specific package validation; it then performs the same deterministic restyling, TOC pagination, full-PDF equation scan, and ZIP normalization before producing a separately named DOCX.

**Tech Stack:** Python 3 standard library, Pandoc, OOXML/OMML, LibreOffice headless, Poppler, ImageMagick, SVG/PNG assets, Git.

## Global Constraints

- Input is `docs/patents/point-mass-trajectory-reconstruction-technical-disclosure-no-figures.md`.
- Output is `docs/patents/point-mass-trajectory-reconstruction-technical-disclosure-no-figures.docx`.
- Never write to `docs/patents/point-mass-trajectory-reconstruction-technical-disclosure.docx`.
- Do not modify `docs/patents/tools/build_disclosure_docx.py`; import and reuse its reviewed helpers.
- Preserve 15 ordered H2 sections, five pseudocode blocks, the measured example, and the complete algorithm-gain body.
- Embed exactly the two current result PNGs; do not link externally or rasterize formulas.
- Convert supported TeX math to editable OMML. LibreOffice-safe editable text fallback is permitted only through the existing compatibility function.
- Validate the formatting of `\widehat{\boldsymbol q}_i`, `\theta_i`, `M=363`, `(M-1)\times0.03=10.86\,\mathrm s`, superscript units, fractions, roots, intervals, and piecewise expressions.
- Use A4 portrait, 1440-twip margins, existing Chinese fonts/styles, 1.5 spacing, first-line indent, dynamic PAGE footer, and a populated/updateable 15-entry TOC.
- Scale images proportionally to the text width; do not crop or stretch them.
- Do not stage, restore, or modify the user-edited original DOCX, its Word lock file, Figure 1 assets, or `swarm-playground/main_ws/.github/`.

---

### Task 1: Add the variant-specific DOCX builder

**Files:**
- Create: `docs/patents/tools/build_example_disclosure_docx.py`
- Import: `docs/patents/tools/build_disclosure_docx.py`
- Read: `docs/patents/point-mass-trajectory-reconstruction-technical-disclosure-no-figures.md`

**Interfaces:**
- `build_with_resources(source: Path, output: Path) -> None`
- `fit_embedded_images(path: Path, maximum_width_emu: int = 5_400_000) -> None`
- `validate_example_docx(path: Path, source: Path) -> None`
- `main() -> int`
- Reused helpers: `restyle_docx`, `cache_toc_page_numbers`, `write_package`, `run`, `require`, namespace constants and `qn`.

- [ ] **Step 1: Implement resource-aware Pandoc conversion**

Invoke Pandoc with:

```text
pandoc SOURCE
--from=markdown+tex_math_dollars+tex_math_single_backslash
--to=docx
--toc
--number-sections
--metadata=lang:zh-CN
--resource-path=SOURCE_PARENT
--output=OUTPUT
```

Resolve all paths from `__file__`; do not depend on the caller's current directory.

- [ ] **Step 2: Implement proportional image sizing**

Open the generated DOCX package, parse `word/document.xml`, and for each `wp:inline` or `wp:anchor` drawing:

- read `wp:extent` width/height;
- if width exceeds 5,400,000 EMU, multiply both dimensions by the same ratio;
- apply the same dimensions to the drawing's matching `a:xfrm/a:ext`;
- preserve the source image bytes and relationship IDs;
- set `w:keepNext` on the explanatory paragraph immediately before an image paragraph and on the image paragraph when followed by a caption paragraph.

Repackage with the imported deterministic `write_package` helper.

- [ ] **Step 3: Implement variant-specific validation**

Validate all of the following:

```text
required Word parts and ZIP integrity
Chinese title
exact ordered H2 body headings: 15
具体示例 and 算法增益效果 present
具体实施例, 有益效果, 附图说明, 仿真验证与效果说明 absent
363, 10.86, 0.0226, -0.6082, 4.6077 present
five pseudocode titles present
A4 page size and 1440-twip margins
Normal/Title/Heading1/Heading2/Heading3/SourceCode styles
centered complex PAGE field
15 TOC hyperlinks/bookmarks/PAGEREF fields with numeric caches
at least one editable m:oMath element
no embedded OLE equation objects
exactly two image relationships and two PNG media parts
embedded media SHA-256 multiset equals the two source PNG SHA-256 values
every drawing width <= 5,400,000 EMU and has positive proportional dimensions
```

Confirm key mathematical operands are present in document text/OMML serialization, including `q`, `θ`, `M`, `363`, `0.03`, and `10.86`.

- [ ] **Step 4: Implement the build pipeline**

`main()` must run in this order:

```text
build_with_resources
base.restyle_docx
fit_embedded_images
base.cache_toc_page_numbers
validate_example_docx
print absolute output path
```

Any failed validation must exit nonzero and must not write the original full-disclosure DOCX.

- [ ] **Step 5: Run static and focused package tests**

Run:

```bash
python3 -m py_compile docs/patents/tools/build_example_disclosure_docx.py
python3 docs/patents/tools/build_example_disclosure_docx.py
unzip -t docs/patents/point-mass-trajectory-reconstruction-technical-disclosure-no-figures.docx
```

Expected: compilation succeeds, build prints the variant DOCX path, ZIP reports no errors, and the builder's complete validation passes.

- [ ] **Step 6: Verify byte reproducibility and commit Task 1**

Hash the generated DOCX, run the builder again, and require the second SHA-256 to match the first. Run `git diff --check` and confirm the original DOCX remains outside the staged paths. Commit only:

```bash
git add docs/patents/tools/build_example_disclosure_docx.py \
        docs/patents/point-mass-trajectory-reconstruction-technical-disclosure-no-figures.docx
git commit -m "docs: build measured-example disclosure DOCX"
```

### Task 2: Inspect formulas, images, and final package

**Files:**
- Verify: `docs/patents/point-mass-trajectory-reconstruction-technical-disclosure-no-figures.docx`
- Verify: `docs/patents/point-mass-trajectory-reconstruction-technical-disclosure-no-figures.md`
- Generate temporarily: `/tmp/example-disclosure-preview.pdf`

**Interfaces:**
- Consumes: the Task 1 builder and DOCX.
- Produces: semantic/package/rendering evidence and a completed plan.

- [ ] **Step 1: Verify source-to-DOCX correspondence**

Extract `word/document.xml` and verify all 15 H2 source headings appear in order after visible numbering. Confirm all five pseudocode titles and measured-example values appear. Confirm the DOCX contains exactly the two expected embedded image hashes and no other raster media.

- [ ] **Step 2: Inspect OMML formula structure**

Count editable `m:oMath`/`m:oMathPara` elements and require a positive count. Locate the paragraphs corresponding to representative formulas and verify their serialized operands, accents, subscripts and superscripts are complete. Specifically inspect:

```text
dot p = v and dot v = a
a_r^- < 0 < a_r^+
quadratic/root expressions
common-time lower bound
piecewise acceleration
widehat bold q_i and theta_i
M=363 and (M-1)*0.03=10.86 s
m/s^2
```

Allow existing intentional editable-text fallbacks, but reject missing operands, replacement characters, or formula images.

- [ ] **Step 3: Export and scan the complete PDF**

Use a unique LibreOffice profile to export the DOCX to PDF. Require positive page count and A4 page size. Extract every page with `pdftotext -layout` and fail on `¿`, `�`, `<?>`, missing measured values, or absent headings.

- [ ] **Step 4: Render and visually inspect representative pages**

Use `pdftoppm` to render at least:

```text
title/TOC page
single-axis formula page
common-time formula page
pseudocode page
具体示例 page
RViz image page
trajectory-curves image page
final 算法增益效果 page
```

Inspect at original detail for Chinese glyphs, formula completeness/alignment, image aspect ratio, image clarity, caption adjacency, page breaks, margins, heading hierarchy and page numbers. If any defect is found, correct the builder, rebuild, and repeat Tasks 1–2 checks.

- [ ] **Step 5: Run final isolation and reproducibility checks**

Run the builder once more, verify its SHA-256 matches the committed/generated artifact, run `git diff --check`, and confirm only the pre-existing user-modified original DOCX, Word lock, and unrelated `.github/` remain outside the committed scope.

- [ ] **Step 6: Mark the plan complete and commit**

Mark all Task 2 checkboxes complete only after the checks pass, then commit only the completed plan:

```bash
git add docs/superpowers/plans/2026-09-22-example-disclosure-docx.md
git commit -m "docs: complete measured-example DOCX verification"
```
