#!/usr/bin/env python3
"""Build the technical disclosure as a consistently styled, editable DOCX."""

from __future__ import annotations

import shutil
import subprocess
import tempfile
import zipfile
import re
from pathlib import Path
from xml.etree import ElementTree as ET


W_NS = "http://schemas.openxmlformats.org/wordprocessingml/2006/main"
R_NS = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"
REL_NS = "http://schemas.openxmlformats.org/package/2006/relationships"
CT_NS = "http://schemas.openxmlformats.org/package/2006/content-types"

NS = {"w": W_NS, "r": R_NS, "rel": REL_NS, "ct": CT_NS}
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
    package_files["word/styles.xml"] = ET.tostring(
        styles, encoding="utf-8", xml_declaration=True
    )

    document = ET.fromstring(package_files["word/document.xml"])
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


def main() -> int:
    build_with_pandoc(SOURCE, OUTPUT)
    restyle_docx(OUTPUT)
    validate_docx(OUTPUT)
    print(OUTPUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
