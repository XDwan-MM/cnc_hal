#!/usr/bin/env python3
"""ESI v2 与计划 PDO 的离线需求审核；不执行配置或发布运行能力。"""
import argparse
import hashlib
import json
import os
import pathlib
import re
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parent.parent
TYPE_NAMES = {"SINT": "SINT", "INTEGER8": "SINT", "INT": "INT", "INTEGER16": "INT",
              "DINT": "DINT", "INTEGER32": "DINT", "USINT": "USINT", "UNSIGNED8": "USINT",
              "UINT": "UINT", "UNSIGNED16": "UINT", "UDINT": "UDINT", "UNSIGNED32": "UDINT",
              "REAL": "REAL", "REAL32": "REAL"}


def checked_int(value, label, maximum=0xFFFFFFFF):
    if isinstance(value, str) and re.fullmatch(r"(?:0[xX][0-9a-fA-F]+|[0-9]+)", value):
        value = int(value, 16 if value.lower().startswith("0x") else 10)
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= maximum:
        raise ValueError(f"{label} 需要 0..{maximum} 整数")
    return value


def identity(item):
    return tuple(checked_int(item[k], k) for k in ("vendor_id", "product_code", "revision"))


class Rules:
    """Freeze the dictionary once; validation and lookup use the production C parser."""
    def __init__(self, path, helper=None):
        self.data = path.read_bytes()
        self.sha256 = hashlib.sha256(self.data).hexdigest()
        self.helper = pathlib.Path(helper or os.environ.get("CNC_HAL_ESI_RULES", ROOT / "build/esi_rules"))
        self.contract = self.query([])

    def query(self, keys):
        with tempfile.TemporaryDirectory(prefix="esi-rules-") as temp:
            path = pathlib.Path(temp) / "devices.json"
            path.write_bytes(self.data)
            try:
                proc = subprocess.run([str(self.helper), str(path),
                                       *(str(v) for key in keys for v in key)],
                                      capture_output=True, text=True)
            except FileNotFoundError as error:
                raise ValueError("缺少 C 字典适配器，请先运行 bash tools/build_esi_rules.sh") from error
        if proc.returncode:
            raise ValueError(f"设备字典读取失败：{proc.stderr.strip()}")
        result = json.loads(proc.stdout)
        if result.get("contract_version") != 1 or len(result["matches"]) != len(keys):
            raise ValueError("C 字典适配器契约不匹配，请重新构建")
        if hasattr(self, "contract") and result["roles"] != self.contract["roles"]:
            raise ValueError("审核期间角色契约变化")
        return result


def load_rules(path, helper=None):
    return Rules(pathlib.Path(path), helper)


def checked_entries(slave, field):
    mapping = slave.get(field)
    if mapping is None:
        return None
    if not isinstance(mapping, dict) or set(mapping) != {"rx", "tx"}:
        raise ValueError(f"{field} 必须同时含 rx 和 tx 数组")
    out = {}
    for direction in ("rx", "tx"):
        if not isinstance(mapping[direction], list):
            raise ValueError(f"{field}.{direction} 必须是数组")
        out[direction] = []
        for entry in mapping[direction]:
            item = {"index": checked_int(entry["index"], "index", 0xFFFF),
                    "subindex": checked_int(entry.get("subindex", 0), "subindex", 0xFF),
                    "bits": checked_int(entry["bits"], "bits", 255)}
            if item["bits"] == 0:
                raise ValueError("PDO 位宽不能为 0")
            if "pdo_index" in entry:
                item["pdo_index"] = checked_int(entry["pdo_index"], "pdo_index", 0xFFFF)
            out[direction].append(item)
    return out


def children(node, tag):
    return [c for c in node.get("children", []) if c["tag"] == tag]


def child_text(node, tag):
    matches = children(node, tag)
    return matches[0].get("text") if len(matches) == 1 else None


def xml_number(text):
    if text is None:
        return None
    return checked_int(text.strip().replace("#x", "0x").replace("#X", "0x"), "XML 整数")


def type_name(value):
    return TYPE_NAMES.get(value.strip().upper()) if isinstance(value, str) else None


def check_role(device, plan, spec):
    key = (spec["index"], spec["subindex"])
    refs = []

    def result(state, reason):
        return {"state": state, "reason": reason, "references": refs,
                "enabled": False, "object": spec}

    if plan is None:
        return result("plan_missing", "未提供本次 planned_pdo_entries；诊断快照不能代替计划")
    mapped = [e for e in plan[spec["direction"]] if (e["index"], e["subindex"]) == key]
    if len(mapped) > 1:
        return result("duplicate_mapping", "计划内此方向出现重复对象，与 C 句柄绑定要求冲突")
    if not mapped:
        return result("not_in_plan", "本次计划未映射该角色")
    if mapped[0]["bits"] != spec["bits"]:
        return result("bits_mismatch", "计划位宽与角色要求不符")

    evidence = device.get("evidence", {})
    if evidence.get("diagnostics"):
        return result("evidence_diagnostic", "ESI 存在解析警告或未支持结构，需人工解决后重新导入")
    entries = [e for e in evidence.get("entries", []) if (e["index"], e["subindex"]) == key]
    if len(entries) > 1 or any(e["origin"] == "conflict" for e in entries):
        return result("evidence_conflict", "对象声明无法唯一对应")

    fields = entries[0]["fields"] if entries else {}
    od = entries and entries[0]["origin"] in ("dictionary", "dictionary_subitem")
    explicit = {}
    if od:
        for name in ("bits", "type", "pdo_mapping"):
            field = fields[name]
            refs.extend(field.get("references", []))
            if field["origin"] == "parser_modified":
                return result("evidence_conflict", "解析器修正了所需对象字段")
            explicit[name] = field["value"] if field["origin"] == "xml" else None
        if explicit["bits"] is not None and explicit["bits"] != spec["bits"]:
            return result("bits_mismatch", "对象声明位宽与角色要求不符")
        declared_type = type_name(explicit["type"])
        if explicit["type"] is not None and declared_type is None:
            return result("type_unknown", "间接类型或未支持类型尚未取得明确语义")
        if declared_type is not None and declared_type != spec["type"]:
            return result("type_mismatch", "对象类型（含有符号性）与角色要求不符")

    # Direct PDO declarations can establish a fixed PDO role without SDO permission.
    tree = evidence.get("declarations", {}).get("device", {})
    candidates = []
    for pdo in children(tree, "RxPdo" if spec["direction"] == "rx" else "TxPdo"):
        pdo_index = xml_number(child_text(pdo, "Index"))
        if "pdo_index" in mapped[0] and pdo_index != mapped[0]["pdo_index"]:
            continue
        for entry in children(pdo, "Entry"):
            if (xml_number(child_text(entry, "Index")), xml_number(child_text(entry, "SubIndex"))) == key:
                candidates.append(entry)
    if candidates:
        refs.extend(e["path"] for e in candidates)
        mapping = explicit.get("pdo_mapping")
        allowed = ("R", "RT") if spec["direction"] == "rx" else ("T", "RT")
        if mapping is not None and mapping not in allowed:
            return result("evidence_conflict", "对象 PdoMapping 与此方向的 PDO 声明冲突")
        values = [(xml_number(child_text(e, "BitLen")), type_name(child_text(e, "DataType")))
                  for e in candidates]
        if len(set(values)) > 1:
            return result("evidence_conflict", "PDO 候选声明不同，需明确选择且解决冲突")
        bits, dtype = values[0]
        if bits is not None and bits != spec["bits"]:
            return result("bits_mismatch", "PDO 声明位宽不符")
        if dtype is not None and dtype != spec["type"]:
            return result("type_mismatch", "PDO 声明类型不符")
        if bits == spec["bits"] and (dtype == spec["type"] or
                (all(child_text(e, "DataType") is None for e in candidates) and od and
                 type_name(explicit.get("type")) == spec["type"])):
            return result("static_ok", "角色与计划及明确 PDO 声明匹配；配置约束尚未授权")
    if od and explicit.get("bits") == spec["bits"] and type_name(explicit.get("type")) == spec["type"]:
        mapping = explicit.get("pdo_mapping")
        allowed = ("R", "RT") if spec["direction"] == "rx" else ("T", "RT")
        if mapping in allowed and not candidates:
            return result("static_ok", "对象明确声明此方向可映射；新增映射约束待规划阶段审核")
    return result("evidence_insufficient", "缺少明确对象类型、位宽或 PDO 证据；不等于设备不支持")


def audit(catalog, topology, rules):
    if catalog.get("schema_version") != 2 or topology.get("schema_version") != 1:
        raise ValueError("需要 v2 ESI 索引和 v1 拓扑；旧索引请从 XML 重建")
    if not isinstance(topology.get("slaves"), list):
        raise ValueError("拓扑快照缺少 slaves 数组")
    devices = {}
    for item in catalog["devices"]:
        key = identity(item["identity"])
        if key in devices:
            raise ValueError(f"ESI 索引身份重复：{key}")
        if identity(item["device"]) != key:
            raise ValueError("ESI 索引和设备身份不一致")
        ev_id = item["device"].get("evidence", {}).get("identity", {})
        if any(ev_id.get(k, {}).get("origin") != "xml" or ev_id[k]["value"] != key[i]
               for i, k in enumerate(("vendor_id", "product_code", "revision"))):
            raise ValueError("ESI 缺少明确身份声明证据")
        devices[key] = item
    keys = [identity(s) for s in topology["slaves"]]
    exported = rules.query(keys)
    roles_contract = exported["roles"]
    results, positions = [], set()
    for slave, key, rule in zip(topology["slaves"], keys, exported["matches"]):
        pos = checked_int(slave["slave_pos"], "slave_pos")
        if pos in positions:
            raise ValueError(f"从站位置重复：{pos}")
        positions.add(pos)
        use = slave.get("use", "unassigned")
        if use not in ("position", "spindle", "unassigned"):
            raise ValueError(f"从站 {pos} 的 use 无效")
        plan = checked_entries(slave, "planned_pdo_entries")
        for field in ("active_pdo_entries", "pre_download_pdo_entries"):
            checked_entries(slave, field)
        item = devices.get(key)
        result = {"slave_pos": pos, "identity": dict(zip(("vendor_id", "product_code", "revision"), key)),
                  "esi": "matched" if item else "missing", "esi_source": item["source"] if item else None,
                  "esi_sha256": item.get("sha256") if item else None,
                  "plan_sha256": hashlib.sha256(json.dumps(plan, sort_keys=True).encode()).hexdigest()
                                 if plan is not None else None,
                  "device_rule": "matched" if rule else "missing", "profile": rule["profile"] if rule else None,
                  "roles": {}, "features": {}, "alarm_control": "undefined_rule",
                  "runtime_ready": False, "configuration_authorization": "not_evaluated"}
        if not item or not rule:
            result.update(decision="blocked" if use != "unassigned" else "pending",
                          reason="缺少精确 ESI 或设备规则，证据不足")
        elif rule["type"] != "servo" or rule["profile"] not in ("ds402", "custom"):
            result.update(decision="not_audited", reason="多轴及 IO 尚无审核规则")
        else:
            required = {n for n, s in roles_contract.items() if s["base"] or (use == "spindle" and s["speed"])}
            for name in roles_contract:
                spec = rule["objects"].get(name)
                value = (check_role(item["device"], plan, spec) if spec else
                         {"state": "rule_missing", "reason": "设备规则未定义该角色", "enabled": False,
                          "references": []})
                value["required"] = name in required
                result["roles"][name] = value
            for feature, names in {
                "position": [n for n, s in roles_contract.items() if s["base"]],
                "speed": [n for n, s in roles_contract.items() if s["base"] or s["speed"]],
                "error_code": ["error_code"],
            }.items():
                result["features"][feature] = {
                    "state": "static_ok" if all(result["roles"][n]["state"] == "static_ok" for n in names)
                             else "unavailable", "dependencies": names, "enabled": False}
            states = [result["roles"][n]["state"] for n in required]
            if plan is None or use == "unassigned":
                result["decision"] = "pending"
            else:
                result["decision"] = "requirements_satisfied" if all(s == "static_ok" for s in states) else "blocked"
        results.append(result)
    if positions != set(range(len(positions))):
        raise ValueError("从站位置应从 0 连续排列")
    results.sort(key=lambda r: r["slave_pos"])
    return {"schema_version": 2, "rules_sha256": rules.sha256, "contract_version": 1,
            "runtime_ready": False, "configuration_authorization": "not_evaluated", "slaves": results,
            "summary": {"count": len(results), **{s: sum(r["decision"] == s for r in results)
                        for s in ("requirements_satisfied", "blocked", "pending", "not_audited")}}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("catalog", type=pathlib.Path)
    parser.add_argument("topology", type=pathlib.Path)
    parser.add_argument("--rules", type=pathlib.Path, default=ROOT / "src/Greemaster/devices.json")
    parser.add_argument("--rules-helper", type=pathlib.Path)
    parser.add_argument("--output", type=pathlib.Path)
    args = parser.parse_args()
    try:
        result = audit(json.loads(args.catalog.read_text(encoding="utf-8")),
                       json.loads(args.topology.read_text(encoding="utf-8")),
                       load_rules(args.rules, args.rules_helper))
        data = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
        if args.output:
            args.output.write_text(data, encoding="utf-8")
        else:
            print(data, end="")
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(1, f"审核失败：{error}\n")


if __name__ == "__main__":
    main()
