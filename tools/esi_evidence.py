"""ESI schema v2：保留 XML 声明，给解析值附加来源；不作功能授权。"""

import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

SCHEMA_VERSION = 2
ADAPTER_VERSION = "cnc-hal-esi-evidence-1"


def number(text):
    if text is None or not text.strip():
        return None
    text = text.strip()
    return int(text[2:], 16) if text.lower().startswith(("#x", "0x")) else int(text, 10)


def raw_tree(node, path, paths):
    """保存属性、正文、顺序和定位；精确原始字节另以 SHA-256 标识。"""
    paths[id(node)] = path
    counts = {}
    children = []
    for child in node:
        counts[child.tag] = counts.get(child.tag, 0) + 1
        children.append(raw_tree(child, f"{path}/{child.tag}[{counts[child.tag]}]", paths))
    return {"tag": node.tag, "path": path, "attributes": dict(node.attrib),
            "text": node.text if node.text and node.text.strip() else None, "children": children}


def field(node, name, paths, parser_value=None, attribute=False, numeric=False):
    target = node if attribute else (node.find(name) if node is not None else None)
    raw = target.get(name) if target is not None and attribute else (
        target.text if target is not None else None)
    references = ([paths[id(target)] + (f"/@{name}" if attribute else "")]
                  if target is not None and (not attribute or name in target.attrib) else [])
    value = None if raw is None or not raw.strip() else raw.strip()
    if numeric and value is not None:
        value = number(value)
    return {"value": value, "origin": "xml" if value is not None else "unknown",
            "raw": raw, "references": references, "parser_value": parser_value}


def unknown(value, origin="unknown", references=None):
    return {"value": None, "origin": origin, "raw": None,
            "references": references or [], "parser_value": value}


def entry_evidence(entry, obj_node, dtype_nodes, paths, pdo_nodes, before):
    source = None
    origin = "unresolved"
    if obj_node is not None:
        type_name = obj_node.findtext("Type", "").strip()
        types = dtype_nodes.get(type_name, [])
        subitems = [sub for dtype in types for sub in dtype.findall("SubItem")]
        if (not subitems and entry["subindex"] == 0 and len(types) <= 1
                and not any(dtype.find("ArrayInfo") is not None for dtype in types)):
            source, origin = obj_node, "dictionary"
        else:
            matches = [sub for sub in subitems if number(sub.findtext("SubIdx")) == entry["subindex"]]
            # Array entries are expanded by the parser. Keep them unresolved here.
            if len(matches) == 1:
                sub_type = matches[0].findtext("Type", "").strip()
                if not any(dtype.find("ArrayInfo") is not None for dtype in dtype_nodes.get(sub_type, [])):
                    source, origin = matches[0], "dictionary_subitem"
    elif pdo_nodes:
        origin = "pdo_derived"

    fields = {"index": field(obj_node, "Index", paths, numeric=True)}
    for output, tag, numeric in (("bits", "BitSize", True), ("type", "Type", False),
                                  ("bit_offset", "BitOffs", True)):
        fields[output] = field(source, tag, paths, entry[output], numeric=numeric)
    flags = source.find("Flags") if source is not None else None
    fields["access"] = field(flags, "Access", paths, entry["access"])
    fields["pdo_mapping"] = field(flags, "PdoMapping", paths, entry["access"] & 192)
    access = flags.find("Access") if flags is not None else None
    for restriction in ("ReadRestrictions", "WriteRestrictions"):
        fields[restriction] = field(access, restriction, paths, attribute=True)
    sub_tag = "SubIdx" if origin == "dictionary_subitem" else "SubIndex"
    fields["subindex"] = field(source, sub_tag, paths, entry["subindex"], numeric=True)
    if source is obj_node and source is not None:
        fields["subindex"] = {"value": 0, "origin": "scalar_convention", "raw": None,
                              "references": [paths[id(source)]], "parser_value": entry["subindex"]}
    fields["default_data"] = field(source, "Info/DefaultData", paths, entry["default_data"])
    fields["default_value"] = field(source, "Info/DefaultValue", paths)

    # Only preserve a unique PDO declaration as field evidence; duplicates stay explicit.
    if origin == "pdo_derived":
        references = [paths[id(p)] for p in pdo_nodes]
        for key in fields:
            fields[key] = unknown(fields[key]["parser_value"], "inferred", references)
        if len(pdo_nodes) == 1:
            fields["index"] = field(pdo_nodes[0], "Index", paths, numeric=True)
            fields["index"]["origin"] = "pdo_declaration"
            for output, tag, numeric in (("bits", "BitLen", True), ("type", "DataType", False),
                                         ("subindex", "SubIndex", True)):
                fields[output] = field(pdo_nodes[0], tag, paths, entry[output], numeric=numeric)
                if fields[output]["origin"] == "xml":
                    fields[output]["origin"] = "pdo_declaration"
    elif source is None:
        origin = "parser_generated" if before is None and obj_node is None else "unresolved"

    if before is not None:
        for key in ("bits", "type", "access", "bit_offset", "default_data"):
            if entry[key] != before[key]:
                fields[key]["declared_value"] = fields[key]["value"]
                fields[key]["value"] = None
                fields[key]["origin"] = "parser_modified"
                fields[key]["parser_before"] = before[key]
    return {"origin": origin, "fields": fields,
            "pdo_declarations": [paths[id(p)] for p in pdo_nodes]}


def model_changes(before, after):
    def grouped(objects):
        result = {}
        for obj in objects:
            result.setdefault(obj["index"], []).append(obj)
        return result
    old, new = grouped(before), grouped(after)
    changes = []
    for index in sorted(old.keys() | new.keys()):
        if old.get(index) == new.get(index):
            continue
        code = "object_synthesized" if index not in old else (
            "object_removed" if index not in new else "object_rewritten")
        changes.append({"code": code, "index": index,
                        "before": old.get(index), "after": new.get(index)})
    return changes


def enrich(device, node, vendor):
    paths = {}
    declarations = {"vendor": raw_tree(vendor, "/Vendor", paths),
                    "device": raw_tree(node, "/Device", paths)}
    identity = {"vendor_id": field(vendor, "Id", paths, device["vendor_id"], numeric=True),
                "product_code": field(node.find("Type"), "ProductCode", paths,
                                      device["product_code"], attribute=True, numeric=True),
                "revision": field(node.find("Type"), "RevisionNo", paths,
                                  device["revision"], attribute=True, numeric=True)}
    for key, evidence in identity.items():
        if evidence["value"] is None or not 0 <= evidence["value"] <= 0xFFFFFFFF:
            raise ValueError(f"ESI 身份 {key} 未声明或超范围，拒绝用解析器默认值建索引")
        if evidence["value"] != device[key]:
            raise ValueError(f"ESI 身份 {key} 与解析结果不一致")

    diagnostics = list(device.pop("parser_diagnostics"))
    dictionary = node.find("Profile/Dictionary")
    raw_objects, dtypes = {}, {}
    if dictionary is not None:
        for obj in dictionary.findall("Objects/Object"):
            raw_objects.setdefault(number(obj.findtext("Index")), []).append(obj)
        for dtype in dictionary.findall("DataTypes/DataType"):
            dtypes.setdefault(dtype.findtext("Name", "").strip(), []).append(dtype)
    for index, objects in raw_objects.items():
        if len(objects) > 1:
            diagnostics.append({"code": "duplicate_object_index", "index": index,
                                "references": [paths[id(o)] for o in objects]})
    for name, types in dtypes.items():
        if len(types) > 1:
            diagnostics.append({"code": "duplicate_data_type", "name": name})

    pdo_targets = {}
    for tag in ("RxPdo", "TxPdo"):
        for pdo in node.findall(tag):
            for entry in pdo.findall("Entry"):
                key = (number(entry.findtext("Index")), number(entry.findtext("SubIndex")))
                pdo_targets.setdefault(key, []).append(entry)

    before_objects = device.pop("declared_dictionary")
    before_entries = {}
    for obj in before_objects:
        for entry in obj["entries"]:
            before_entries.setdefault((obj["index"], entry["subindex"]), []).append(entry)
    transformations = model_changes(before_objects, device["objects"])
    entries = []
    for obj in device["objects"]:
        raw = raw_objects.get(obj["index"], [])
        for entry in obj["entries"]:
            key = (obj["index"], entry["subindex"])
            before = before_entries.get(key, [])
            evidence = entry_evidence(entry, raw[0] if len(raw) == 1 else None, dtypes, paths,
                                      pdo_targets.get(key, []), before[0] if len(before) == 1 else None)
            if len(raw) > 1 or len(before) > 1:
                evidence["origin"] = "conflict"
                for name, value in evidence["fields"].items():
                    evidence["fields"][name] = unknown(value["parser_value"], "conflict")
            entries.append({"index": obj["index"], "subindex": entry["subindex"], **evidence})

    def attributes(element, names):
        return {name: field(element, name, paths, attribute=True) for name in names}

    # Missing attributes remain null, never inherit the normalized model's false/zero.
    coe = node.find("Mailbox/CoE")
    constraints = {
        "coe": attributes(coe, ("PdoAssign", "PdoConfig", "PdoUpload", "SdoInfo", "CompleteAccess")),
        "sm": [attributes(sm, ("Enable", "StartAddress", "MinSize", "MaxSize", "DefaultSize",
                                "ControlByte", "Virtual", "OpOnly")) for sm in node.findall("Sm")],
        "pdos": [{"direction": tag, "reference": paths[id(pdo)],
                  "fields": attributes(pdo, ("Sm", "Su", "Fixed", "Mandatory", "Virtual", "OSFac",
                                             "OSMin", "OSMax", "OSIndexInc", "PdoOrder"))}
                 for tag in ("RxPdo", "TxPdo") for pdo in node.findall(tag)],
    }
    if any(node.find(tag) is not None for tag in ("Slots", "ModuleGroups")):
        diagnostics.append({"code": "unsupported_modular_composition", "reference": "/Device"})
    if len(node.findall("Profile")) > 1:
        diagnostics.append({"code": "multiple_profiles_not_normalized", "reference": "/Device"})
    device["evidence"] = {
        "identity": identity, "entries": entries, "constraints": constraints,
        "declarations": declarations, "transformations": transformations,
        "diagnostics": diagnostics,
        "coverage": {"dictionary_declared": dictionary is not None,
                     "dictionary_completeness": "unknown",
                     "configuration_authorization": "not_evaluated",
                     "unresolved_entries": sum(e["origin"] in ("unresolved", "conflict") for e in entries),
                     "raw_declarations_preserved": True,
                     "normalized_model_authoritative": False},
    }
    return device


def import_file(path, extractor):
    path = Path(path).resolve()
    data = path.read_bytes()
    try:
        root = ET.fromstring(data)
    except ET.ParseError as error:
        raise ValueError(f"无效 XML：{path}：{error}") from error
    if root.tag != "EtherCATInfo":
        raise ValueError("ESI 根元素必须是 EtherCATInfo（当前不支持命名空间变体）")
    vendor = root.find("Vendor")
    nodes = root.findall("Descriptions/Devices/Device")
    if vendor is None or not nodes:
        raise ValueError("ESI 缺少 Vendor 或 Device")
    # Parse exactly the bytes being hashed; an edited source cannot mix two versions.
    with tempfile.TemporaryDirectory(prefix="cnc-hal-esi-") as temp:
        frozen = Path(temp) / "source.xml"
        frozen.write_bytes(data)
        result = subprocess.run([str(extractor), str(frozen)], capture_output=True, text=True)
    if result.returncode:
        raise ValueError(f"解析失败：{path}\n{result.stderr.strip()}")
    parsed = json.loads(result.stdout)
    if parsed.get("schema_version") != SCHEMA_VERSION or len(parsed["devices"]) != len(nodes):
        raise ValueError("提取器版本或设备数量不匹配；请重新构建 ESI 提取器")
    parsed["source"] = str(path)
    parsed["sha256"] = hashlib.sha256(data).hexdigest()
    parsed["adapter_version"] = ADAPTER_VERSION
    parsed["import_diagnostics"] = ([{"code": "parser_stderr", "message": result.stderr}]
                                    if result.stderr.strip() else [])
    parsed["devices"] = [enrich(device, node, vendor) for device, node in zip(parsed["devices"], nodes)]
    return parsed
