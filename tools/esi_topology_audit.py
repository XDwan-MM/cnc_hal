#!/usr/bin/env python3
"""用拓扑快照、ESI 索引和现有设备字典做离线静态审核。"""

import argparse
import json
import pathlib


DS402 = {
    "status_word": (0x6041, 0, 16, "tx"),
    "actual_pos": (0x6064, 0, 32, "tx"),
    "error_code": (0x603F, 0, 16, "tx"),
    "mode_display": (0x6061, 0, 8, "tx"),
    "actual_speed": (0x606C, 0, 32, "tx"),
    "actual_torque": (0x6077, 0, 16, "tx"),
    "control_word": (0x6040, 0, 16, "rx"),
    "target_pos": (0x607A, 0, 32, "rx"),
    "target_speed": (0x60FF, 0, 32, "rx"),
    "op_mode": (0x6060, 0, 8, "rx"),
}
REQUIRED = {"status_word", "actual_pos", "mode_display", "control_word", "target_pos", "op_mode"}


def checked_int(value, label, maximum=0xFFFFFFFF):
    if isinstance(value, bool):
        raise ValueError(f"{label} 需要非负整数")
    if isinstance(value, str):
        try:
            value = int(value, 0)
        except ValueError as error:
            raise ValueError(f"{label} 需要非负整数") from error
    if not isinstance(value, int) or not 0 <= value <= maximum:
        raise ValueError(f"{label} 需要 0..{maximum} 整数")
    return value


def identity(item):
    return tuple(checked_int(item[k], k) for k in ("vendor_id", "product_code", "revision"))


def load_rules(path):
    # 当前 devices.json 的注释均为整行 //；保留该文件作为唯一设备角色定义来源。
    text = "\n".join(line for line in path.read_text(encoding="utf-8").splitlines()
                     if not line.lstrip().startswith("//"))
    data = json.loads(text)
    if data.get("version") != 1 or not isinstance(data.get("devices"), list):
        raise ValueError("设备字典版本或 devices 字段无效")
    return data["devices"]


def find_rule(rules, key):
    exact = [rule for rule in rules if checked_int(rule["vendor_id"], "vendor_id") == key[0]
             and checked_int(rule["product_code"], "product_code") == key[1]
             and rule.get("revision") != "*"
             and checked_int(rule["revision"], "revision") == key[2]]
    wildcard = [rule for rule in rules if checked_int(rule["vendor_id"], "vendor_id") == key[0]
                and checked_int(rule["product_code"], "product_code") == key[1]
                and rule.get("revision") == "*"]
    matches = exact or wildcard
    if len(matches) > 1:
        raise ValueError(f"设备字典身份重复：{key}")
    return matches[0] if matches else None


def object_roles(rule):
    if rule["profile"] == "ds402":
        return DS402
    if rule["profile"] == "custom":
        roles = {}
        for name, data in rule["objects"].items():
            if name not in DS402:
                raise ValueError(f"未知设备角色：{name}")
            standard = DS402[name]
            roles[name] = (checked_int(data["index"], "index", 0xFFFF),
                           checked_int(data.get("sub", 0), "sub", 0xFF),
                           checked_int(data.get("bits", standard[2]), "bits", 255),
                           data.get("dir", standard[3]))
        return roles
    return None


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
        out[direction] = {
            (checked_int(e["index"], "index", 0xFFFF),
             checked_int(e.get("subindex", 0), "subindex", 0xFF),
             checked_int(e["bits"], "bits", 255)) for e in mapping[direction]
        }
    return out


def check_role(device, active, pre_download, specification):
    index, subindex, bits, direction = specification
    if direction not in ("rx", "tx"):
        raise ValueError(f"对象方向无效：{direction}")
    objects = [obj for obj in device["objects"] if obj["index"] == index]
    if not objects:
        return "object_missing", "ESI 未列出对象"
    entries = [entry for obj in objects for entry in obj["entries"]
               if entry["subindex"] == subindex]
    if not entries:
        return "subindex_missing", "ESI 未列出子索引"
    if not any(entry["bits"] == bits for entry in entries):
        return "bits_mismatch", "ESI 位宽与设备规则不符"
    access_mask = 64 if direction == "rx" else 128
    if not any(entry["bits"] == bits and entry["access"] & access_mask for entry in entries):
        return "esi_mapping_unknown", "ESI 未确认此方向可映射"
    operation_mask = (8 | 16 | 32) if direction == "rx" else (1 | 2 | 4)
    if not any(entry["bits"] == bits and entry["access"] & operation_mask for entry in entries):
        return "esi_access_unknown", "ESI 未确认此方向可读写"
    if active is None:
        if pre_download is not None:
            if (index, subindex, bits) in pre_download[direction]:
                return "pdo_pre_download", "条目在下发前方案中；尚未确认启用"
            return "not_in_pre_download_pdo", "下发前方案中没有此条目"
        return "pdo_unverified", "尚无当前 PDO 条目快照"
    if (index, subindex, bits) not in active[direction]:
        return "not_in_current_pdo", "当前 PDO 中没有此条目"
    return "static_ok", "ESI 与当前 PDO 快照均匹配；仍需实机启动验证"


def audit(catalog, topology, rules):
    if catalog.get("schema_version") != 1 or topology.get("schema_version") != 1:
        raise ValueError("ESI 索引或拓扑快照版本无效")
    if not isinstance(topology.get("slaves"), list):
        raise ValueError("拓扑快照缺少 slaves 数组")
    devices = {}
    for item in catalog["devices"]:
        key = identity(item["identity"])
        if key in devices:
            raise ValueError(f"ESI 索引身份重复：{key}")
        devices[key] = item
    results = []
    positions = set()
    for slave in topology["slaves"]:
        pos = checked_int(slave["slave_pos"], "slave_pos")
        if pos in positions:
            raise ValueError(f"从站位置重复：{pos}")
        positions.add(pos)
        key = identity(slave)
        item = devices.get(key)
        rule = find_rule(rules, key)
        result = {"slave_pos": pos, "identity": dict(zip(
            ("vendor_id", "product_code", "revision"), key)),
            "esi": "matched" if item else "missing",
            "esi_source": item["source"] if item else None,
            "device_rule": "matched" if rule else "missing",
            "profile": rule["profile"] if rule else None,
            "roles": {}, "alarm_control": "undefined_rule"}
        if not item or not rule:
            if slave.get("use") in ("position", "spindle"):
                result["decision"] = "blocked"
            else:
                result["decision"] = "not_audited" if not rule else "pending"
        else:
            roles = object_roles(rule)
            if roles is None:
                result["decision"] = "not_audited"
            else:
                use = slave.get("use", "unassigned")
                if use not in ("position", "spindle", "unassigned"):
                    raise ValueError(f"从站 {pos} 的 use 无效")
                active = checked_entries(slave, "active_pdo_entries")
                pre_download = checked_entries(slave, "pre_download_pdo_entries")
                required = set(REQUIRED)
                if use == "spindle":
                    required.add("actual_speed")
                for name, specification in roles.items():
                    state, reason = check_role(item["device"], active, pre_download,
                                               specification)
                    result["roles"][name] = {"required": name in required,
                                              "state": state, "reason": reason,
                                              "object": {"index": specification[0],
                                                         "subindex": specification[1],
                                                         "bits": specification[2],
                                                         "direction": specification[3]}}
                required_states = [value["state"] for value in result["roles"].values()
                                   if value["required"]]
                if len(required_states) < len(required):
                    result["decision"] = "blocked"
                elif any(state in ("object_missing", "subindex_missing", "bits_mismatch",
                                   "not_in_current_pdo") for state in required_states):
                    result["decision"] = "blocked"
                elif any(state != "static_ok" for state in required_states):
                    result["decision"] = "pending"
                elif use == "unassigned":
                    result["decision"] = "pending"
                else:
                    result["decision"] = "static_eligible"
        results.append(result)
    if positions != set(range(len(positions))):
        raise ValueError("从站位置应从 0 连续排列")
    results.sort(key=lambda value: value["slave_pos"])
    return {"schema_version": 1, "slaves": results,
            "summary": {"count": len(results),
                        "static_eligible": sum(r["decision"] == "static_eligible" for r in results),
                        "blocked": sum(r["decision"] == "blocked" for r in results),
                        "pending": sum(r["decision"] == "pending" for r in results),
                        "not_audited": sum(r["decision"] == "not_audited" for r in results)}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("catalog", type=pathlib.Path)
    parser.add_argument("topology", type=pathlib.Path)
    parser.add_argument("--rules", type=pathlib.Path, default=pathlib.Path(__file__).resolve().parent.parent /
                        "src/Greemaster/devices.json")
    parser.add_argument("--output", type=pathlib.Path)
    args = parser.parse_args()
    try:
        result = audit(json.loads(args.catalog.read_text(encoding="utf-8")),
                       json.loads(args.topology.read_text(encoding="utf-8")),
                       load_rules(args.rules))
        data = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
        if args.output:
            args.output.write_text(data, encoding="utf-8")
        else:
            print(data, end="")
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(1, f"审核失败：{error}\n")


if __name__ == "__main__":
    main()
