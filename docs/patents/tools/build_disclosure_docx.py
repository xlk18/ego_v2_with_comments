#!/usr/bin/env python3
"""Build an editable DOCX with a verified, navigable table of contents.

Requires Pandoc, LibreOffice and Poppler's pdftotext. LibreOffice measures the
current environment's pagination and checks all pages for equation-import error
markers; it does not rewrite the DOCX. Word can update the TOC/PAGEREF fields if
the recipient's fonts or pagination differ.
"""

from __future__ import annotations

import shutil
import subprocess
import tempfile
import zipfile
import re
from pathlib import Path
from xml.etree import ElementTree as ET


W_NS = "http://schemas.openxmlformats.org/wordprocessingml/2006/main"
M_NS = "http://schemas.openxmlformats.org/officeDocument/2006/math"
R_NS = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"
REL_NS = "http://schemas.openxmlformats.org/package/2006/relationships"
CT_NS = "http://schemas.openxmlformats.org/package/2006/content-types"

NS = {"w": W_NS, "m": M_NS, "r": R_NS, "rel": REL_NS, "ct": CT_NS}
ET.register_namespace("w", W_NS)
ET.register_namespace("r", R_NS)
ET.register_namespace("", REL_NS)

SOURCE = Path(__file__).resolve().parent.parent / (
    "point-mass-trajectory-reconstruction-technical-disclosure.md"
)
OUTPUT = SOURCE.with_suffix(".docx")
FOOTER_REL_TYPE = (
    "http://schemas.openxmlformats.org/officeDocument/2006/relationships/footer"
)
FOOTER_CONTENT_TYPE = (
    "application/vnd.openxmlformats-officedocument.wordprocessingml.footer+xml"
)
ZIP_TIMESTAMP = (1980, 1, 1, 0, 0, 0)
CORE_TIMESTAMP = "1980-01-01T00:00:00Z"
CORE_TIMESTAMP_PATTERN = re.compile(
    br"(<dcterms:(?:created|modified)\b[^>]*>)[^<]*(</dcterms:(?:created|modified)>)"
)


def qn(namespace: str, name: str) -> str:
    return f"{{{namespace}}}{name}"


def run(command: list[str]) -> None:
    """Run a build command and propagate a useful failure to the caller."""
    subprocess.run(command, check=True)


def build_with_pandoc(source: Path, output: Path) -> None:
    """Convert Markdown to DOCX while retaining TeX math as editable OMML."""
    output.parent.mkdir(parents=True, exist_ok=True)
    run(
        [
            "pandoc",
            str(source),
            "--from=markdown+tex_math_dollars+tex_math_single_backslash",
            "--to=docx",
            "--toc",
            "--number-sections",
            "--metadata=lang:zh-CN",
            f"--output={output}",
        ]
    )


def find_or_add(parent: ET.Element, tag: str) -> ET.Element:
    child = parent.find(tag)
    if child is None:
        child = ET.SubElement(parent, tag)
    return child


def style_by_id(styles: ET.Element, style_id: str) -> ET.Element:
    style = styles.find(f"w:style[@w:styleId='{style_id}']", NS)
    if style is not None:
        return style

    style = ET.SubElement(
        styles,
        qn(W_NS, "style"),
        {qn(W_NS, "type"): "paragraph", qn(W_NS, "styleId"): style_id},
    )
    ET.SubElement(style, qn(W_NS, "name"), {qn(W_NS, "val"): style_id})
    return style


def set_style(
    styles: ET.Element,
    style_id: str,
    east_asia: str,
    ascii_font: str,
    half_points: int,
    *,
    bold: bool = False,
    centered: bool = False,
) -> None:
    style = style_by_id(styles, style_id)
    run_properties = find_or_add(style, qn(W_NS, "rPr"))
    fonts = find_or_add(run_properties, qn(W_NS, "rFonts"))
    fonts.set(qn(W_NS, "eastAsia"), east_asia)
    fonts.set(qn(W_NS, "ascii"), ascii_font)
    size = find_or_add(run_properties, qn(W_NS, "sz"))
    size.set(qn(W_NS, "val"), str(half_points))
    if bold:
        find_or_add(run_properties, qn(W_NS, "b"))
    else:
        bold_tag = run_properties.find(qn(W_NS, "b"))
        if bold_tag is not None:
            run_properties.remove(bold_tag)
    if centered:
        paragraph_properties = find_or_add(style, qn(W_NS, "pPr"))
        justification = find_or_add(paragraph_properties, qn(W_NS, "jc"))
        justification.set(qn(W_NS, "val"), "center")


def set_normal_paragraph_style(styles: ET.Element) -> None:
    normal = style_by_id(styles, "Normal")
    paragraph_properties = find_or_add(normal, qn(W_NS, "pPr"))
    spacing = find_or_add(paragraph_properties, qn(W_NS, "spacing"))
    spacing.set(qn(W_NS, "line"), "360")
    spacing.set(qn(W_NS, "lineRule"), "auto")
    indent = find_or_add(paragraph_properties, qn(W_NS, "ind"))
    indent.set(qn(W_NS, "firstLine"), "480")


def normalize_style_order(styles: ET.Element) -> None:
    """Respect CT_Style order, including styles created without a reference."""
    order = ["name", "aliases", "basedOn", "next", "link", "autoRedefine",
             "hidden", "uiPriority", "semiHidden", "unhideWhenUsed", "qFormat",
             "locked", "personal", "personalCompose", "personalReply", "rsid",
             "pPr", "rPr", "tblPr", "trPr", "tcPr", "tblStylePr"]
    ranks = {qn(W_NS, tag): rank for rank, tag in enumerate(order)}
    for style in styles.findall("w:style", NS):
        style[:] = sorted(style, key=lambda child: ranks.get(child.tag, len(order)))


def footer_xml() -> bytes:
    footer = ET.Element(qn(W_NS, "ftr"))
    paragraph = ET.SubElement(footer, qn(W_NS, "p"))
    paragraph_properties = ET.SubElement(paragraph, qn(W_NS, "pPr"))
    ET.SubElement(
        paragraph_properties, qn(W_NS, "jc"), {qn(W_NS, "val"): "center"}
    )
    run = ET.SubElement(paragraph, qn(W_NS, "r"))
    ET.SubElement(run, qn(W_NS, "fldChar"), {qn(W_NS, "fldCharType"): "begin"})
    run = ET.SubElement(paragraph, qn(W_NS, "r"))
    instruction = ET.SubElement(run, qn(W_NS, "instrText"))
    instruction.set("{http://www.w3.org/XML/1998/namespace}space", "preserve")
    instruction.text = " PAGE "
    run = ET.SubElement(paragraph, qn(W_NS, "r"))
    ET.SubElement(run, qn(W_NS, "fldChar"), {qn(W_NS, "fldCharType"): "separate"})
    run = ET.SubElement(paragraph, qn(W_NS, "r"))
    ET.SubElement(run, qn(W_NS, "t")).text = "1"
    run = ET.SubElement(paragraph, qn(W_NS, "r"))
    ET.SubElement(run, qn(W_NS, "fldChar"), {qn(W_NS, "fldCharType"): "end"})
    return ET.tostring(footer, encoding="utf-8", xml_declaration=True)


def footer_relationship_id(relationships: ET.Element) -> str:
    for relationship in relationships.findall("rel:Relationship", NS):
        if relationship.get("Type") == FOOTER_REL_TYPE and relationship.get("Target") == "footer1.xml":
            return relationship.get("Id", "rIdFooter1")

    used_ids = {relationship.get("Id") for relationship in relationships}
    relation_id = "rIdFooter1"
    number = 1
    while relation_id in used_ids:
        number += 1
        relation_id = f"rIdFooter{number}"
    ET.SubElement(
        relationships,
        qn(REL_NS, "Relationship"),
        {"Id": relation_id, "Type": FOOTER_REL_TYPE, "Target": "footer1.xml"},
    )
    return relation_id


def set_sections(document: ET.Element, footer_relation_id: str) -> None:
    for section in document.findall(".//w:sectPr", NS):
        for reference in list(section.findall("w:footerReference", NS)):
            if reference.get(qn(W_NS, "type")) == "default":
                section.remove(reference)
        section.insert(
            0,
            ET.Element(
                qn(W_NS, "footerReference"),
                {
                    qn(W_NS, "type"): "default",
                    qn(R_NS, "id"): footer_relation_id,
                },
            ),
        )
        page_size = find_or_add(section, qn(W_NS, "pgSz"))
        page_size.set(qn(W_NS, "w"), "11906")
        page_size.set(qn(W_NS, "h"), "16838")
        page_size.attrib.pop(qn(W_NS, "orient"), None)
        margins = find_or_add(section, qn(W_NS, "pgMar"))
        for side in ("top", "right", "bottom", "left"):
            margins.set(qn(W_NS, side), "1440")


def add_footer_content_type(content_types: ET.Element) -> None:
    part_name = "/word/footer1.xml"
    for override in content_types.findall("ct:Override", NS):
        if override.get("PartName") == part_name:
            override.set("ContentType", FOOTER_CONTENT_TYPE)
            return
    ET.SubElement(
        content_types,
        qn(CT_NS, "Override"),
        {"PartName": part_name, "ContentType": FOOTER_CONTENT_TYPE},
    )


def normalize_core_properties(package_files: dict[str, bytes]) -> None:
    core_properties, replacements = CORE_TIMESTAMP_PATTERN.subn(
        lambda match: match.group(1)
        + CORE_TIMESTAMP.encode("ascii")
        + match.group(2),
        package_files["docProps/core.xml"],
    )
    if replacements != 2:
        raise ValueError("Pandoc core properties are missing creation timestamps")
    package_files["docProps/core.xml"] = core_properties


def serialize_omml(node: ET.Element | None) -> str:
    """Serialize an OMML node into editable, LibreOffice-safe equation text."""
    if node is None:
        return ""
    tag = node.tag.rsplit("}", 1)[-1]
    children = list(node)
    if tag == "t":
        return node.text or ""
    if tag in {"sSub", "sSup", "sSubSup"}:
        base = node.find(qn(M_NS, "e"))
        subscript = node.find(qn(M_NS, "sub"))
        superscript = node.find(qn(M_NS, "sup"))
        text = serialize_omml(base) if base is not None else ""
        if subscript is not None:
            text += "_" + serialize_omml(subscript)
        if superscript is not None:
            text += "^" + serialize_omml(superscript)
        return text
    if tag == "f":
        numerator = node.find(qn(M_NS, "num"))
        denominator = node.find(qn(M_NS, "den"))
        return f"({serialize_omml(numerator)})/({serialize_omml(denominator)})"
    if tag == "d":
        properties = node.find(qn(M_NS, "dPr"))
        expression = node.find(qn(M_NS, "e"))
        begin = properties.find(qn(M_NS, "begChr")) if properties is not None else None
        end = properties.find(qn(M_NS, "endChr")) if properties is not None else None
        begin_character = begin.get(qn(M_NS, "val"), "") if begin is not None else ""
        end_character = end.get(qn(M_NS, "val"), "") if end is not None else ""
        return begin_character + serialize_omml(expression) + end_character
    if tag == "m":
        rows = []
        for row in node.findall(qn(M_NS, "mr")):
            cells = [serialize_omml(cell) for cell in row.findall(qn(M_NS, "e"))]
            rows.append("; ".join(cells))
        return "; ".join(rows)
    if tag == "rad":
        degree = node.find(qn(M_NS, "deg"))
        expression = node.find(qn(M_NS, "e"))
        degree_text = serialize_omml(degree)
        root = "√" if not degree_text else f"√[{degree_text}]"
        return root + "(" + serialize_omml(expression) + ")"
    return "".join(serialize_omml(child) for child in children)


def word_text_run(text: str) -> ET.Element:
    run = ET.Element(qn(W_NS, "r"))
    content = ET.SubElement(run, qn(W_NS, "t"))
    if text.startswith(" ") or text.endswith(" "):
        content.set("{http://www.w3.org/XML/1998/namespace}space", "preserve")
    content.text = text
    return run


def contains_libreoffice_unsafe_math(math: ET.Element) -> bool:
    # LO 6.4's OMML importer misinterprets literal delimiter runs and a bare
    # operator subscript as StarMath syntax. Structural tags alone miss these.
    unsafe_tags = {"d", "m", "sSup", "sSubSup"}
    text = serialize_omml(math)
    operator_subscript = any(
        serialize_omml(sub).strip() in {"+", "−", "-"}
        for sub in math.findall(".//m:sub", NS)
    )
    return any(character in text for character in "∞|[]") or operator_subscript or any(
        element.tag.rsplit("}", 1)[-1] in unsafe_tags
        for element in math.iter()
    )


def replace_unsafe_math(document: ET.Element) -> None:
    """Replace only LibreOffice-unsafe OMML constructs with editable text."""
    for paragraph in document.findall(".//w:p", NS):
        for index, child in enumerate(list(paragraph)):
            if child.tag == qn(M_NS, "oMath") and contains_libreoffice_unsafe_math(child):
                paragraph.remove(child)
                paragraph.insert(index, word_text_run(serialize_omml(child)))
            elif child.tag == qn(M_NS, "oMathPara"):
                equations = child.findall(qn(M_NS, "oMath"))
                if not equations or not any(contains_libreoffice_unsafe_math(item) for item in equations):
                    continue
                paragraph_properties = find_or_add(paragraph, qn(W_NS, "pPr"))
                justification = find_or_add(paragraph_properties, qn(W_NS, "jc"))
                justification.set(qn(W_NS, "val"), "center")
                text = "".join(serialize_omml(item) for item in equations)
                paragraph.remove(child)
                paragraph.insert(index, word_text_run(text))


def paragraph_text(paragraph: ET.Element) -> str:
    return "".join(paragraph.itertext())


def prepend_paragraph_text(paragraph: ET.Element, prefix: str) -> None:
    for text in paragraph.findall(".//w:t", NS):
        text.text = prefix + (text.text or "")
        return
    paragraph.append(word_text_run(prefix))


def format_title_and_headings(document: ET.Element) -> list[str]:
    """Make the title distinct and materialize visible section numbering."""
    section_number = 0
    subsection_number = 0
    sections = []
    for paragraph in document.findall(".//w:p", NS):
        style = paragraph.find("w:pPr/w:pStyle", NS)
        if style is None:
            continue
        style_id = style.get(qn(W_NS, "val"))
        text = paragraph_text(paragraph)
        if style_id == "Heading1" and text == "质点轨迹重构算法专利技术交底书":
            style.set(qn(W_NS, "val"), "Title")
            properties = find_or_add(paragraph, qn(W_NS, "pPr"))
            justification = find_or_add(properties, qn(W_NS, "jc"))
            justification.set(qn(W_NS, "val"), "center")
        elif style_id == "Heading2":
            section_number += 1
            subsection_number = 0
            prefix = f"{section_number}. "
            if not text.startswith(prefix):
                prepend_paragraph_text(paragraph, prefix)
            sections.append(paragraph_text(paragraph))
        elif style_id == "Heading3":
            subsection_number += 1
            prefix = f"{section_number}.{subsection_number} "
            if not text.startswith(prefix):
                prepend_paragraph_text(paragraph, prefix)
    return sections


def paragraph_with_style(style_id: str, text: str) -> ET.Element:
    paragraph = ET.Element(qn(W_NS, "p"))
    properties = ET.SubElement(paragraph, qn(W_NS, "pPr"))
    ET.SubElement(properties, qn(W_NS, "pStyle"), {qn(W_NS, "val"): style_id})
    paragraph.append(word_text_run(text))
    return paragraph


def toc_field_boundary(field_type: str) -> ET.Element:
    """Create one boundary of an updateable TOC field with cached entries."""
    paragraph = ET.Element(qn(W_NS, "p"))
    if field_type == "begin":
        run = ET.SubElement(paragraph, qn(W_NS, "r"))
        ET.SubElement(run, qn(W_NS, "fldChar"), {qn(W_NS, "fldCharType"): "begin"})
        run = ET.SubElement(paragraph, qn(W_NS, "r"))
        instruction = ET.SubElement(run, qn(W_NS, "instrText"))
        instruction.set("{http://www.w3.org/XML/1998/namespace}space", "preserve")
        instruction.text = ' TOC \\o "1-3" \\h \\z \\u '
        run = ET.SubElement(paragraph, qn(W_NS, "r"))
        ET.SubElement(run, qn(W_NS, "fldChar"), {qn(W_NS, "fldCharType"): "separate"})
    elif field_type == "end":
        run = ET.SubElement(paragraph, qn(W_NS, "r"))
        ET.SubElement(run, qn(W_NS, "fldChar"), {qn(W_NS, "fldCharType"): "end"})
    else:
        raise ValueError(f"unsupported TOC field boundary: {field_type}")
    return paragraph


def populate_toc(document: ET.Element, sections: list[str]) -> None:
    """Keep an updateable TOC field while supplying visible cached entries."""
    body = document.find(qn(W_NS, "body"))
    if body is None:
        raise ValueError("missing document body")
    for index, child in enumerate(list(body)):
        if child.tag != qn(W_NS, "sdt"):
            continue
        content = child.find(qn(W_NS, "sdtContent"))
        if content is None:
            raise ValueError("missing table-of-contents content")
        for entry in list(content):
            content.remove(entry)
        entries = [
            paragraph_with_style("TOCHeading", "目录"),
            toc_field_boundary("begin"),
        ]
        headings = [p for p in document.findall(".//w:p", NS)
                    if p.find("w:pPr/w:pStyle[@w:val='Heading2']", NS) is not None]
        bookmark_ids = [int(item.get(qn(W_NS, "id"), "0"))
                        for item in document.findall(".//w:bookmarkStart", NS)]
        next_id = max(bookmark_ids, default=0) + 1
        for number, (text, heading) in enumerate(zip(sections, headings), 1):
            anchor = f"DisclosureSection{number}"
            bookmark_id = str(next_id + number)
            heading.insert(1, ET.Element(qn(W_NS, "bookmarkStart"),
                {qn(W_NS, "id"): bookmark_id, qn(W_NS, "name"): anchor}))
            heading.append(ET.Element(qn(W_NS, "bookmarkEnd"),
                                      {qn(W_NS, "id"): bookmark_id}))
            entry = paragraph_with_style("TOC1", "")
            entry.remove(entry[-1])
            properties = entry.find(qn(W_NS, "pPr"))
            tabs = ET.SubElement(properties, qn(W_NS, "tabs"))
            ET.SubElement(tabs, qn(W_NS, "tab"), {qn(W_NS, "val"): "right",
                qn(W_NS, "leader"): "dot", qn(W_NS, "pos"): "9026"})
            hyperlink = ET.SubElement(entry, qn(W_NS, "hyperlink"),
                {qn(W_NS, "anchor"): anchor, qn(W_NS, "history"): "1"})
            hyperlink.append(word_text_run(text))
            tab_run = ET.SubElement(hyperlink, qn(W_NS, "r"))
            ET.SubElement(tab_run, qn(W_NS, "tab"))
            page_ref = ET.SubElement(hyperlink, qn(W_NS, "fldSimple"),
                {qn(W_NS, "instr"): f" PAGEREF {anchor} \\h "})
            page_ref.append(word_text_run("00"))
            entries.append(entry)
        entries.append(toc_field_boundary("end"))
        for entry in entries:
            content.append(entry)
        return
    raise ValueError("missing Pandoc table of contents")


def restyle_docx(path: Path) -> None:
    """Apply the disclosure's Word styles, page setup, and PAGE footer."""
    with zipfile.ZipFile(path) as package:
        package_files = {
            name: package.read(name)
            for name in package.namelist()
            if not name.endswith("/")
        }

    styles = ET.fromstring(package_files["word/styles.xml"])
    set_style(styles, "Normal", "宋体", "Times New Roman", 24)
    set_style(styles, "Title", "黑体", "Arial", 36, bold=True, centered=True)
    set_style(styles, "Heading1", "黑体", "Arial", 32, bold=True)
    set_style(styles, "Heading2", "黑体", "Arial", 28, bold=True)
    set_style(styles, "Heading3", "黑体", "Arial", 24, bold=True)
    set_style(styles, "SourceCode", "等线", "Courier New", 20)
    set_normal_paragraph_style(styles)
    normalize_style_order(styles)
    package_files["word/styles.xml"] = ET.tostring(
        styles, encoding="utf-8", xml_declaration=True
    )

    document = ET.fromstring(package_files["word/document.xml"])
    sections = format_title_and_headings(document)
    replace_unsafe_math(document)
    populate_toc(document, sections)
    relationships = ET.fromstring(package_files["word/_rels/document.xml.rels"])
    footer_relation_id = footer_relationship_id(relationships)
    set_sections(document, footer_relation_id)
    package_files["word/document.xml"] = ET.tostring(
        document, encoding="utf-8", xml_declaration=True
    )
    ET.register_namespace("", REL_NS)
    package_files["word/_rels/document.xml.rels"] = ET.tostring(
        relationships, encoding="utf-8", xml_declaration=True
    )

    content_types = ET.fromstring(package_files["[Content_Types].xml"])
    add_footer_content_type(content_types)
    ET.register_namespace("", CT_NS)
    package_files["[Content_Types].xml"] = ET.tostring(
        content_types, encoding="utf-8", xml_declaration=True
    )
    package_files["word/footer1.xml"] = footer_xml()
    normalize_core_properties(package_files)

    write_package(path, package_files)


def write_package(path: Path, package_files: dict[str, bytes]) -> None:
    """Write stable ZIP metadata, including after page-reference updates."""

    with tempfile.NamedTemporaryFile(
        suffix=".docx", dir=path.parent, delete=False
    ) as temporary_file:
        temporary_path = Path(temporary_file.name)
    try:
        with zipfile.ZipFile(temporary_path, "w", zipfile.ZIP_DEFLATED) as package:
            for name, content in package_files.items():
                package_member = zipfile.ZipInfo(name, date_time=ZIP_TIMESTAMP)
                package_member.compress_type = zipfile.ZIP_DEFLATED
                package_member.create_system = 3
                package_member.external_attr = 0o600 << 16
                package.writestr(package_member, content)
        shutil.move(str(temporary_path), str(path))
    finally:
        if temporary_path.exists():
            temporary_path.unlink()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def validate_docx(path: Path) -> None:
    """Assert that the built package contains its required editable DOCX features."""
    required_files = {
        "[Content_Types].xml",
        "word/document.xml",
        "word/styles.xml",
        "word/footer1.xml",
        "word/_rels/document.xml.rels",
    }
    with zipfile.ZipFile(path) as package:
        names = set(package.namelist())
        require(required_files <= names, "DOCX package is missing required Word parts")
        document = ET.fromstring(package.read("word/document.xml"))
        styles = ET.fromstring(package.read("word/styles.xml"))
        footer = ET.fromstring(package.read("word/footer1.xml"))
        relationships = ET.fromstring(package.read("word/_rels/document.xml.rels"))
        content_types = ET.fromstring(package.read("[Content_Types].xml"))

    document_text = "".join(document.itertext())
    require("质点轨迹重构算法专利技术交底书" in document_text, "missing Chinese title")
    require(
        "仿真验证与效果说明（预留）" in document_text,
        "missing simulation-placeholder heading",
    )
    footer_rel_ids = {
        relationship.get("Id")
        for relationship in relationships.findall("rel:Relationship", NS)
        if relationship.get("Type") == FOOTER_REL_TYPE
        and relationship.get("Target") == "footer1.xml"
    }
    require(footer_rel_ids, "missing footer relationship")
    require(
        any(
            override.get("PartName") == "/word/footer1.xml"
            and override.get("ContentType") == FOOTER_CONTENT_TYPE
            for override in content_types.findall("ct:Override", NS)
        ),
        "missing footer content type",
    )
    instructions = footer.findall(".//w:r/w:instrText", NS)
    field_characters = [
        character.get(qn(W_NS, "fldCharType"))
        for character in footer.findall(".//w:fldChar", NS)
    ]
    require(
        any("PAGE" in (instruction.text or "") for instruction in instructions)
        and field_characters == ["begin", "separate", "end"],
        "missing PAGE field",
    )

    sections = document.findall(".//w:sectPr", NS)
    require(sections, "missing document section properties")
    for section in sections:
        page_size = section.find("w:pgSz", NS)
        margins = section.find("w:pgMar", NS)
        require(
            page_size is not None
            and page_size.get(qn(W_NS, "w")) == "11906"
            and page_size.get(qn(W_NS, "h")) == "16838",
            "missing A4 portrait page size",
        )
        require(
            page_size.get(qn(W_NS, "orient")) in (None, "portrait"),
            "page orientation is not portrait",
        )
        require(
            margins is not None
            and all(margins.get(qn(W_NS, side)) == "1440" for side in ("top", "right", "bottom", "left")),
            "missing 1440-twip margins",
        )
        footer_reference = section.find("w:footerReference[@w:type='default']", NS)
        require(
            footer_reference is not None
            and footer_reference.get(qn(R_NS, "id")) in footer_rel_ids,
            "missing default footer reference",
        )

    required_style_ids = {"Normal", "Title", "Heading1", "Heading2", "Heading3", "SourceCode"}
    present_style_ids = {
        style.get(qn(W_NS, "styleId")) for style in styles.findall("w:style", NS)
    }
    require(required_style_ids <= present_style_ids, "missing required Word styles")
    for style in styles.findall("w:style", NS):
        tags = [child.tag for child in style]
        if qn(W_NS, "pPr") in tags and qn(W_NS, "rPr") in tags:
            require(tags.index(qn(W_NS, "pPr")) < tags.index(qn(W_NS, "rPr")),
                    "style paragraph properties follow run properties")
    bookmarks = {item.get(qn(W_NS, "name"))
                 for item in document.findall(".//w:bookmarkStart", NS)}
    toc = document.find(".//w:sdtContent", NS)
    require(toc is not None, "missing updateable TOC")
    links = toc.findall(".//w:hyperlink", NS)
    require(len(links) == 18, "missing TOC hyperlinks")
    for link in links:
        anchor = link.get(qn(W_NS, "anchor"))
        require(anchor in bookmarks, "TOC hyperlink has no destination")
        field = link.find("w:fldSimple", NS)
        require(field is not None and f"PAGEREF {anchor}" in field.get(qn(W_NS, "instr"), ""),
                "missing TOC page reference field")
        require("".join(field.itertext()).isdigit(), "missing cached TOC page number")


def export_and_check_pdf(path: Path, directory: Path) -> list[str]:
    """Check every rendered page, not just selected formula screenshots."""
    directory.mkdir(parents=True, exist_ok=True)
    profile = directory / "lo-profile"
    run(["libreoffice", f"-env:UserInstallation={profile.as_uri()}", "--headless",
         "--convert-to", "pdf", "--outdir", str(directory), str(path)])
    pdf = directory / path.with_suffix(".pdf").name
    require(pdf.is_file(), "LibreOffice did not produce a PDF")
    result = subprocess.run(["pdftotext", "-layout", str(pdf), "-"],
                            check=True, capture_output=True, text=True)
    pages = result.stdout.split("\f")
    if not pages[-1].strip():
        pages.pop()
    require(bool(pages), "empty PDF")
    for number, page in enumerate(pages, 1):
        require(not any(marker in page for marker in ("¿", "�", "<?>")),
                f"equation rendering error marker on PDF page {number}")
    return pages


def cache_toc_page_numbers(path: Path) -> None:
    """Cache PAGEREF results using checked, convergent PDF pagination.

    The field remains updateable in Word. We keep Pandoc's original editable
    content, using LibreOffice only to measure pagination, never to rewrite it.
    Re-export verifies both full-document equation rendering and page stability.
    """
    with tempfile.TemporaryDirectory(prefix="disclosure-pagination-") as temporary:
        for attempt in range(3):
            pages = export_and_check_pdf(path, Path(temporary) / str(attempt))
            with zipfile.ZipFile(path) as package:
                files = {name: package.read(name) for name in package.namelist()}
            document = ET.fromstring(files["word/document.xml"])
            entries = document.findall(".//w:sdtContent/w:p/w:hyperlink", NS)
            changed = False
            for entry in entries:
                heading = "".join(entry.find("w:r", NS).itertext())
                compact_heading = re.sub(r"\s+", "", heading)
                matches = [number for number, page in enumerate(pages, 1)
                           if compact_heading in re.sub(r"\s+", "", page)]
                require(len(matches) >= 2, f"cannot locate body heading: {heading}")
                # The first occurrence is in the TOC; the last is the body heading.
                number = str(matches[-1])
                cache = entry.find("w:fldSimple/w:r/w:t", NS)
                if cache.text != number:
                    cache.text = number
                    changed = True
            if not changed:
                print(f"PDF PASS: {len(pages)} pages; all pages free of error markers; TOC page references stable")
                return
            files["word/document.xml"] = ET.tostring(document, encoding="utf-8", xml_declaration=True)
            write_package(path, files)
    raise ValueError("TOC pagination did not converge in three exports")


def main() -> int:
    build_with_pandoc(SOURCE, OUTPUT)
    restyle_docx(OUTPUT)
    cache_toc_page_numbers(OUTPUT)
    validate_docx(OUTPUT)
    print(OUTPUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
