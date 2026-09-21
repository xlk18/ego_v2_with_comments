#!/usr/bin/env python3
"""Build the editable measured-example disclosure, including its two PNGs.

Requires Pandoc, LibreOffice and pdftotext, as does the base disclosure builder.
Paths are anchored to this script; the original disclosure is never rebuilt.
"""

from __future__ import annotations

from collections import Counter
from hashlib import sha256
from pathlib import Path, PurePosixPath
import os
import re
import struct
import tempfile
import zipfile
from xml.etree import ElementTree as ET

import build_disclosure_docx as base
from build_disclosure_docx import M_NS, R_NS, W_NS, qn, require, run, write_package


SOURCE = Path(__file__).resolve().parent.parent / (
    "point-mass-trajectory-reconstruction-technical-disclosure-no-figures.md"
)
OUTPUT = SOURCE.with_suffix(".docx")
WP_NS = "http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing"
A_NS = "http://schemas.openxmlformats.org/drawingml/2006/main"
PIC_NS = "http://schemas.openxmlformats.org/drawingml/2006/picture"
NS = {**base.NS, "wp": WP_NS, "a": A_NS, "pic": PIC_NS}
IMAGE_REL_TYPE = f"{R_NS}/image"
MAXIMUM_WIDTH_EMU = 5_400_000
TOC_INSTRUCTION = 'TOC \\o "2-2" \\h \\z \\u'
NORMALIZED_DIRECTION_RHS = (
    "=((sinθ_i,cosθ_i,1))/(√(sin²θ_i+cos²θ_i+1))"
    "=((sinθ_i,cosθ_i,1))/(√(2))."
)


def build_with_resources(source: Path, output: Path) -> None:
    """Convert the source and embed images resolved beside the Markdown."""
    source, output = source.resolve(), output.resolve()
    require(output != base.OUTPUT.resolve(), "cannot overwrite the original disclosure")
    output.parent.mkdir(parents=True, exist_ok=True)
    run([
        "pandoc", str(source),
        "--from=markdown+tex_math_dollars+tex_math_single_backslash",
        "--to=docx", "--toc", "--number-sections", "--metadata=lang:zh-CN",
        f"--resource-path={source.parent}", f"--output={output}",
    ])


def restrict_toc_scope(path: Path) -> None:
    """Keep Word's updated TOC limited to the 15 level-two section headings."""
    with zipfile.ZipFile(path) as package:
        files = {name: package.read(name) for name in package.namelist()}
    document = ET.fromstring(files["word/document.xml"])
    instructions = [node for node in document.findall(".//w:sdtContent//w:instrText", NS)
                    if (node.text or "").strip().startswith("TOC ")]
    require(len(instructions) == 1, "expected one complex TOC instruction")
    instructions[0].text = f" {TOC_INSTRUCTION} "
    files["word/document.xml"] = ET.tostring(document, encoding="utf-8", xml_declaration=True)
    write_package(path, files)


def drawings(document: ET.Element) -> list[ET.Element]:
    return [element for element in document.iter()
            if element.tag in {qn(WP_NS, "inline"), qn(WP_NS, "anchor")}]


def normalized_direction_paragraph(document: ET.Element) -> ET.Element:
    """Locate the example's equation by its preceding prose, not global operands."""
    body = document.find("w:body", NS)
    require(body is not None, "missing document body")
    children = list(body)
    introductions = [index for index, child in enumerate(children)
                     if child.tag == qn(W_NS, "p")
                     and base.paragraph_text(child).endswith("各圆形航点同时赋予归一化参考运动方向：")]
    require(len(introductions) == 1 and introductions[0] + 1 < len(children),
            "missing unique normalized-direction formula introduction")
    paragraph = children[introductions[0] + 1]
    require(paragraph.tag == qn(W_NS, "p"), "missing normalized-direction formula paragraph")
    return paragraph


def preserve_normalized_direction(path: Path) -> None:
    """Restore the fallback equation's hatted bold vector as safe editable OMML.

    The base LO 6.4 fallback drops accent/style properties when serializing an
    equation containing superscripts. Keep only this equation's left-hand side
    in OMML; its remaining expression stays editable linear Word text.
    """
    with zipfile.ZipFile(path) as package:
        files = {name: package.read(name) for name in package.namelist()}
    document = ET.fromstring(files["word/document.xml"])
    paragraph = normalized_direction_paragraph(document)
    text = base.paragraph_text(paragraph)
    require(re.sub(r"\s+", "", text).replace("^2", "²") == "q_i" + NORMALIZED_DIRECTION_RHS,
            "normalized-direction fallback differs from the expected source equation")
    for child in list(paragraph):
        if child.tag != qn(W_NS, "pPr"):
            paragraph.remove(child)
    equation = ET.SubElement(paragraph, qn(M_NS, "oMath"))
    subscript = ET.SubElement(equation, qn(M_NS, "sSub"))
    expression = ET.SubElement(subscript, qn(M_NS, "e"))
    accent = ET.SubElement(expression, qn(M_NS, "acc"))
    properties = ET.SubElement(accent, qn(M_NS, "accPr"))
    ET.SubElement(properties, qn(M_NS, "chr"), {qn(M_NS, "val"): "\u0302"})
    base_expression = ET.SubElement(accent, qn(M_NS, "e"))
    vector = ET.SubElement(base_expression, qn(M_NS, "r"))
    run_properties = ET.SubElement(vector, qn(M_NS, "rPr"))
    ET.SubElement(run_properties, qn(M_NS, "sty"), {qn(M_NS, "val"): "b"})
    # LO 6.4 ignores both m:sty and w:b on this imported math run. The Unicode
    # mathematical bold small q preserves its vector weight in Word and LO.
    ET.SubElement(vector, qn(M_NS, "t")).text = "𝐪"
    index = ET.SubElement(subscript, qn(M_NS, "sub"))
    index_run = ET.SubElement(index, qn(M_NS, "r"))
    ET.SubElement(index_run, qn(M_NS, "t")).text = "i"
    paragraph.append(base.word_text_run(text[len("q_i"):].replace("^2", "²")))
    files["word/document.xml"] = ET.tostring(document, encoding="utf-8", xml_declaration=True)
    write_package(path, files)


def set_keep_next(paragraph: ET.Element) -> None:
    properties = paragraph.find("w:pPr", NS)
    if properties is None:
        properties = ET.Element(qn(W_NS, "pPr"))
        paragraph.insert(0, properties)
    keep = properties.find("w:keepNext", NS)
    if keep is None:
        keep = ET.Element(qn(W_NS, "keepNext"))
        # CT_PPr puts keepNext after pStyle and before other paragraph options.
        properties.insert(1 if properties.find("w:pStyle", NS) is not None else 0, keep)
    keep.set(qn(W_NS, "val"), "1")


def image_neighbors(document: ET.Element):
    """Yield adjacent paragraphs without crossing table/container boundaries."""
    for parent in document.iter():
        children = list(parent)
        for index, child in enumerate(children):
            if child.tag != qn(W_NS, "p") or not drawings(child):
                continue
            previous = children[index - 1] if index else None
            following = children[index + 1] if index + 1 < len(children) else None
            yield previous, child, following


def is_explanation(paragraph: ET.Element | None) -> bool:
    return (paragraph is not None and paragraph.tag == qn(W_NS, "p")
            and bool(base.paragraph_text(paragraph).strip()) and not drawings(paragraph))


def is_caption(paragraph: ET.Element | None) -> bool:
    if paragraph is None or paragraph.tag != qn(W_NS, "p"):
        return False
    return (base.paragraph_text(paragraph).strip().startswith("图：")
            or paragraph.find("w:pPr/w:pStyle[@w:val='ImageCaption']", NS) is not None)


def caption_continuations(document: ET.Element):
    """Keep Pandoc's implicit caption attached to the source's explicit caption."""
    for parent in document.iter():
        children = list(parent)
        for current, following in zip(children, children[1:]):
            if is_caption(current) and is_caption(following):
                yield current


def fit_embedded_images(path: Path, maximum_width_emu: int = 5_400_000) -> None:
    """Fit drawings proportionally and keep explanatory text/captions adjacent."""
    require(maximum_width_emu > 0, "image width limit must be positive")
    with zipfile.ZipFile(path) as package:
        files = {name: package.read(name) for name in package.namelist()}
    document = ET.fromstring(files["word/document.xml"])
    for number, drawing in enumerate(drawings(document), 1):
        nonvisual = drawing.find("wp:docPr", NS)
        require(nonvisual is not None, "drawing has no nonvisual properties")
        nonvisual.set("id", str(number))
        for picture_properties in drawing.findall(".//pic:cNvPr", NS):
            picture_properties.set("id", str(number))
        extent = drawing.find("wp:extent", NS)
        require(extent is not None, "drawing has no extent")
        width, height = int(extent.get("cx", "0")), int(extent.get("cy", "0"))
        require(width > 0 and height > 0, "drawing dimensions must be positive")
        if width > maximum_width_emu:
            height = max(1, round(height * maximum_width_emu / width))
            width = maximum_width_emu
        transforms = drawing.findall(".//a:xfrm/a:ext", NS)
        require(len(transforms) == 1, "drawing must have one matching picture transform")
        for dimensions in [extent, *transforms]:
            dimensions.set("cx", str(width))
            dimensions.set("cy", str(height))
    for previous, paragraph, following in image_neighbors(document):
        if is_explanation(previous):
            set_keep_next(previous)
        if is_caption(following):
            set_keep_next(paragraph)
    for caption in caption_continuations(document):
        set_keep_next(caption)
    files["word/document.xml"] = ET.tostring(document, encoding="utf-8", xml_declaration=True)
    write_package(path, files)


def validate_example_docx(path: Path, source: Path) -> None:
    """Fail on missing content, broken navigation, or damaged embedded images."""
    required_files = {
        "[Content_Types].xml", "_rels/.rels", "docProps/core.xml",
        "word/document.xml", "word/styles.xml", "word/footer1.xml",
        "word/_rels/document.xml.rels",
    }
    with zipfile.ZipFile(path) as package:
        require(package.testzip() is None, "DOCX ZIP integrity failure")
        names = package.namelist()
        require(len(names) == len(set(names)), "duplicate DOCX package members")
        require(required_files <= set(names), "DOCX is missing required Word parts")
        files = {name: package.read(name) for name in names}
    document = ET.fromstring(files["word/document.xml"])
    styles = ET.fromstring(files["word/styles.xml"])
    footer = ET.fromstring(files["word/footer1.xml"])
    relationships = ET.fromstring(files["word/_rels/document.xml.rels"])
    content_types = ET.fromstring(files["[Content_Types].xml"])
    markdown = source.read_text(encoding="utf-8")
    text = "".join(document.itertext()).replace("−", "-")
    require("质点轨迹重构算法专利技术交底书" in text, "missing Chinese title")
    expected = re.findall(r"^## (.+)$", markdown, re.MULTILINE)
    headings = [p for p in document.findall("./w:body/w:p", NS)
                if p.find("w:pPr/w:pStyle[@w:val='Heading2']", NS) is not None]
    require(len(expected) == 15 and [base.paragraph_text(p) for p in headings]
            == [f"{number}. {title}" for number, title in enumerate(expected, 1)],
            "body must contain the 15 source H2 headings in order")
    require({"具体示例", "算法增益效果"} <= set(expected), "missing example/gain headings")
    for obsolete in ("具体实施例", "有益效果", "附图说明", "仿真验证与效果说明"):
        require(obsolete not in text, f"obsolete heading remains: {obsolete}")
    for value in ("363", "10.86", "0.0226", "-0.6082", "4.6077"):
        require(value in text, f"missing measured value: {value}")
    pseudocode_titles = re.findall(r"^算法[一二三四五]：.+$", markdown, re.MULTILINE)
    require(len(pseudocode_titles) == 5, "source must contain five pseudocode titles")
    for title in pseudocode_titles:
        require(title in text, f"missing pseudocode: {title}")

    footer_ids = {r.get("Id") for r in relationships
                  if r.get("Type") == base.FOOTER_REL_TYPE and r.get("Target") == "footer1.xml"}
    require(bool(footer_ids), "missing footer relationship")
    require(any(item.get("PartName") == "/word/footer1.xml"
                and item.get("ContentType") == base.FOOTER_CONTENT_TYPE
                for item in content_types), "missing footer content type")
    footer_paragraph = footer.find("w:p", NS)
    require(footer_paragraph is not None, "missing footer paragraph")
    center = footer_paragraph.find("w:pPr/w:jc", NS)
    require(center is not None and center.get(qn(W_NS, "val")) == "center",
            "PAGE footer must be centered")
    require([node.get(qn(W_NS, "fldCharType")) for node in footer.findall(".//w:fldChar", NS)]
            == ["begin", "separate", "end"]
            and any((node.text or "").strip() == "PAGE"
                    for node in footer.findall(".//w:instrText", NS)), "missing complex PAGE field")
    sections = document.findall(".//w:sectPr", NS)
    require(bool(sections), "missing section properties")
    for section in sections:
        size, margins = section.find("w:pgSz", NS), section.find("w:pgMar", NS)
        require(size is not None and size.get(qn(W_NS, "w")) == "11906"
                and size.get(qn(W_NS, "h")) == "16838"
                and size.get(qn(W_NS, "orient")) in (None, "portrait"), "missing A4 portrait size")
        require(margins is not None and all(margins.get(qn(W_NS, side)) == "1440"
                for side in ("top", "right", "bottom", "left")), "missing 1440-twip margins")
        reference = section.find("w:footerReference[@w:type='default']", NS)
        require(reference is not None and reference.get(qn(R_NS, "id")) in footer_ids,
                "missing default footer reference")
    style_ids = {item.get(qn(W_NS, "styleId")) for item in styles.findall("w:style", NS)}
    require({"Normal", "Title", "Heading1", "Heading2", "Heading3", "SourceCode"} <= style_ids,
            "missing required Word styles")
    for style_id, east_asia, ascii_font, half_points in (
        ("Normal", "宋体", "Times New Roman", "24"),
        ("Title", "黑体", "Arial", "36"),
        ("Heading1", "黑体", "Arial", "32"),
        ("Heading2", "黑体", "Arial", "28"),
        ("Heading3", "黑体", "Arial", "24"),
        ("SourceCode", "等线", "Courier New", "20"),
    ):
        style = styles.find(f"w:style[@w:styleId='{style_id}']", NS)
        fonts, size = style.find("w:rPr/w:rFonts", NS), style.find("w:rPr/w:sz", NS)
        require(fonts is not None and fonts.get(qn(W_NS, "eastAsia")) == east_asia
                and fonts.get(qn(W_NS, "ascii")) == ascii_font
                and size is not None and size.get(qn(W_NS, "val")) == half_points,
                f"incorrect font or size for {style_id}")
    normal = styles.find("w:style[@w:styleId='Normal']", NS)
    spacing, indent = normal.find("w:pPr/w:spacing", NS), normal.find("w:pPr/w:ind", NS)
    require(spacing is not None and spacing.get(qn(W_NS, "line")) == "360"
            and spacing.get(qn(W_NS, "lineRule")) == "auto"
            and indent is not None and indent.get(qn(W_NS, "firstLine")) == "480",
            "incorrect normal paragraph spacing or first-line indent")

    toc = document.find(".//w:sdtContent", NS)
    require(toc is not None, "missing TOC")
    instructions = [(node.text or "").strip()
                    for node in toc.findall(".//w:instrText", NS)]
    require(instructions == [TOC_INSTRUCTION],
            "TOC must update only Heading2 sections with the exact intended instruction")
    require([node.get(qn(W_NS, "fldCharType"))
             for node in toc.findall(".//w:fldChar", NS)] == ["begin", "separate", "end"],
            "TOC must retain valid complex-field boundaries")
    links = toc.findall(".//w:hyperlink", NS)
    require(len(links) == 15 and len(toc.findall(".//w:fldSimple", NS)) == 15,
            "TOC must contain 15 hyperlinks and PAGEREF fields")
    toc_bookmarks = [node for node in document.findall(".//w:bookmarkStart", NS)
                     if node.get(qn(W_NS, "name"), "").startswith("DisclosureSection")]
    require(len(toc_bookmarks) == 15, "missing TOC destination bookmarks")
    for number, (link, heading) in enumerate(zip(links, headings), 1):
        anchor = f"DisclosureSection{number}"
        bookmark = heading.find(f"w:bookmarkStart[@w:name='{anchor}']", NS)
        require(link.get(qn(W_NS, "anchor")) == anchor and bookmark is not None,
                "TOC link does not target its body heading")
        label = link.find("w:r", NS)
        require(label is not None and "".join(label.itertext()) == base.paragraph_text(heading),
                "TOC label differs from body heading")
        require(heading.find(f"w:bookmarkEnd[@w:id='{bookmark.get(qn(W_NS, 'id'))}']", NS)
                is not None, "TOC bookmark has no matching end")
        field = link.find("w:fldSimple", NS)
        require(field is not None and field.get(qn(W_NS, "instr"), "").strip()
                == f"PAGEREF {anchor} \\h", "missing TOC PAGEREF")
        cache = "".join(field.itertext())
        require(cache.isdigit() and int(cache) > 0, "missing positive cached TOC page number")

    equations = document.findall(".//m:oMath", NS)
    require(bool(equations), "missing editable OMML equations")
    math_text = text + "".join(base.serialize_omml(equation) for equation in equations)
    for operand in ("q", "θ", "M", "363", "0.03", "10.86"):
        require(operand in math_text, f"missing mathematical operand: {operand}")
    direction = normalized_direction_paragraph(document)
    subscript = direction.find("m:oMath/m:sSub", NS)
    require(subscript is not None, "normalized-direction vector has no editable subscript")
    accent = subscript.find("m:e/m:acc/m:accPr/m:chr", NS)
    vector = subscript.find("m:e/m:acc/m:e/m:r", NS)
    require(accent is not None and accent.get(qn(M_NS, "val")) == "\u0302",
            "normalized-direction vector has no hat accent")
    require(vector is not None and base.serialize_omml(vector) == "𝐪"
            and vector.find("m:rPr/m:sty[@m:val='b']", NS) is not None,
            "normalized-direction vector q must be bold")
    require(base.serialize_omml(subscript.find("m:sub", NS)) == "i",
            "normalized-direction vector must have subscript i")
    rhs = "".join(node.text or "" for node in direction.findall("w:r/w:t", NS))
    require(re.sub(r"\s+", "", rhs) == NORMALIZED_DIRECTION_RHS,
            "normalized-direction RHS operands or normalization denominator are incomplete")
    require(not document.findall(".//w:object", NS)
            and not any(node.tag.rsplit("}", 1)[-1] == "OLEObject" for node in document.iter())
            and not any(name.startswith("word/embeddings/") for name in names)
            and not any(r.get("Type", "").endswith("/oleObject") for r in relationships),
            "embedded OLE equation objects are not allowed")

    images = [r for r in relationships if r.get("Type") == IMAGE_REL_TYPE]
    media = {name: data for name, data in files.items() if name.startswith("word/media/")}
    require(len(images) == 2 and len(media) == 2 and all(name.endswith(".png") for name in media),
            "expected exactly two image relationships and two PNG media parts")
    image_sources = re.findall(r"!\[[^\]]*\]\(([^)]+)\)", markdown)
    require(len(image_sources) == 2, "source must reference exactly two PNGs")
    source_hashes = Counter(sha256((source.parent / name).read_bytes()).hexdigest()
                            for name in image_sources)
    require(Counter(sha256(data).hexdigest() for data in media.values()) == source_hashes,
            "embedded image hashes differ from source PNGs")
    targets = {}
    for relationship in images:
        require(relationship.get("TargetMode") != "External", "images must be embedded")
        target = str(PurePosixPath("word") / relationship.get("Target", ""))
        require(target in media, "image relationship has no media target")
        targets[relationship.get("Id")] = target
    require(len(targets) == 2 and set(targets.values()) == set(media), "invalid image relationship IDs")
    all_drawings = drawings(document)
    require(len(all_drawings) == 2, "expected two image drawings")
    drawing_ids = [node.get("id", "") for node in document.findall(".//wp:docPr", NS)]
    require(len(drawing_ids) == len(all_drawings)
            and all(value.isdigit() and int(value) > 0 for value in drawing_ids)
            and len({int(value) for value in drawing_ids}) == len(drawing_ids),
            "drawing nonvisual IDs must be unique positive integers")
    used_ids = []
    for drawing in all_drawings:
        nonvisual = drawing.find("wp:docPr", NS)
        require(nonvisual is not None, "drawing has no nonvisual properties")
        picture_properties = drawing.findall(".//pic:cNvPr", NS)
        require(len(picture_properties) == 1
                and picture_properties[0].get("id") == nonvisual.get("id"),
                "picture nonvisual ID differs from drawing ID")
        extent = drawing.find("wp:extent", NS)
        require(extent is not None, "drawing has no extent")
        width, height = int(extent.get("cx", "0")), int(extent.get("cy", "0"))
        require(0 < width <= MAXIMUM_WIDTH_EMU and height > 0, "invalid drawing dimensions")
        transforms = drawing.findall(".//a:xfrm/a:ext", NS)
        require(len(transforms) == 1 and transforms[0].get("cx") == str(width)
                and transforms[0].get("cy") == str(height), "picture transform differs from drawing extent")
        blips = drawing.findall(".//a:blip", NS)
        require(len(blips) == 1 and blips[0].get(qn(R_NS, "embed")) in targets,
                "drawing has no embedded PNG relationship")
        relation_id = blips[0].get(qn(R_NS, "embed"))
        used_ids.append(relation_id)
        png = media[targets[relation_id]]
        require(png[:8] == b"\x89PNG\r\n\x1a\n" and png[12:16] == b"IHDR", "invalid PNG header")
        pixel_width, pixel_height = struct.unpack(">II", png[16:24])
        require(pixel_width > 0 and pixel_height > 0
                and abs(width * pixel_height - height * pixel_width) <= pixel_width + pixel_height,
                "drawing aspect ratio differs from source PNG")
        require(not drawing.findall(".//a:srcRect", NS), "image must not be cropped")
    require(set(used_ids) == set(targets), "both images must be drawn")
    for previous, paragraph, following in image_neighbors(document):
        for candidate in ([previous] if is_explanation(previous) else []) + (
                [paragraph] if is_caption(following) else []):
            keep = candidate.find("w:pPr/w:keepNext", NS)
            require(keep is not None and keep.get(qn(W_NS, "val")) not in ("0", "false", "off"),
                    "image explanation/caption can separate across pages")
    for caption in caption_continuations(document):
        keep = caption.find("w:pPr/w:keepNext", NS)
        require(keep is not None and keep.get(qn(W_NS, "val")) not in ("0", "false", "off"),
                "implicit and explicit image captions can separate across pages")


def main() -> int:
    output = OUTPUT.resolve()
    require(output != base.OUTPUT.resolve(), "cannot overwrite the original disclosure")
    output.parent.mkdir(parents=True, exist_ok=True)
    descriptor, name = tempfile.mkstemp(prefix=f".{output.stem}-", suffix=".docx",
                                        dir=output.parent)
    os.close(descriptor)
    staging = Path(name)
    try:
        build_with_resources(SOURCE, staging)
        base.restyle_docx(staging)
        restrict_toc_scope(staging)
        preserve_normalized_direction(staging)
        fit_embedded_images(staging)
        base.cache_toc_page_numbers(staging)
        validate_example_docx(staging, SOURCE)
        staging.chmod(output.stat().st_mode & 0o777 if output.exists() else 0o644)
        os.replace(staging, output)
    finally:
        staging.unlink(missing_ok=True)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
