#!/usr/bin/env python3
"""Offline evidence and production-rule conformance regressions."""
import copy
import json
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
from esi_topology_audit import audit, load_rules
from esi_evidence import raw_tree

KEY = (477, 271601776, 33818120)
# Independent expected protocol contract: do not derive these assertions from the exporter.
EXPECTED = {
    "status_word": (0x6041, 16, "tx", "UINT"),
    "actual_pos": (0x6064, 32, "tx", "DINT"),
    "error_code": (0x603F, 16, "tx", "UINT"),
    "mode_display": (0x6061, 8, "tx", "SINT"),
    "actual_speed": (0x606C, 32, "tx", "DINT"),
    "actual_torque": (0x6077, 16, "tx", "INT"),
    "control_word": (0x6040, 16, "rx", "UINT"),
    "target_pos": (0x607A, 32, "rx", "DINT"),
    "target_speed": (0x60FF, 32, "rx", "DINT"),
    "op_mode": (0x6060, 8, "rx", "SINT"),
}


def fixture(pdo_only=False):
    ident = dict(zip(("vendor_id", "product_code", "revision"), KEY))
    plan = {"rx": [], "tx": []}
    entries = []
    xml = ET.Element("Device")
    for name, (index, bits, direction, dtype) in EXPECTED.items():
        plan[direction].append({"index": index, "subindex": 0, "bits": bits})
        def field(value, origin="xml"):
            return {"value": value, "origin": origin, "references": ["/Device/test"], "parser_value": 999}
        entries.append({"index": index, "subindex": 0,
                        "origin": "pdo_derived" if pdo_only else "dictionary",
                        "fields": {"bits": field(None if pdo_only else bits),
                                   "type": field(None if pdo_only else dtype),
                                   "pdo_mapping": field(None if pdo_only else ("R" if direction == "rx" else "T")),
                                   "access": field(None, "unknown")}})
        if pdo_only:
            pdo = ET.SubElement(xml, "RxPdo" if direction == "rx" else "TxPdo")
            ET.SubElement(pdo, "Index").text = str(0x1600 if direction == "rx" else 0x1A00)
            entry = ET.SubElement(pdo, "Entry")
            for tag, value in (("Index", index), ("SubIndex", 0), ("BitLen", bits), ("DataType", dtype)):
                ET.SubElement(entry, tag).text = str(value)
    evidence = {"entries": entries, "identity": {k: {"origin": "xml", "value": v} for k, v in ident.items()},
                "diagnostics": [], "declarations": {"device": raw_tree(xml, "/Device", {})}}
    catalog = {"schema_version": 2, "devices": [{"identity": ident, "source": "fixture.xml",
               "device": {**ident, "objects": [], "evidence": evidence}}]}
    topology = {"schema_version": 1, "slaves": [{"slave_pos": 0, **ident, "use": "position",
                                               "planned_pdo_entries": plan}]}
    return catalog, topology


def ev(catalog, index):
    return next(e for e in catalog["devices"][0]["device"]["evidence"]["entries"] if e["index"] == index)


class AuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rules = load_rules(ROOT / "src/Greemaster/devices.json")

    def result(self, catalog, topology):
        report = audit(catalog, topology, self.rules)
        self.assertFalse(report["runtime_ready"])
        self.assertEqual(report["configuration_authorization"], "not_evaluated")
        return report["slaves"][0]

    def test_contract_and_no_runtime_authorization(self):
        exported = self.rules.query([KEY])
        roles = exported["matches"][0]["objects"]
        self.assertEqual(set(roles), set(EXPECTED))
        for name, (index, bits, direction, dtype) in EXPECTED.items():
            self.assertEqual((roles[name]["index"], roles[name]["bits"], roles[name]["direction"],
                              roles[name]["type"]), (index, bits, direction, dtype))
        result = self.result(*fixture())
        self.assertEqual(result["decision"], "requirements_satisfied")
        self.assertFalse(result["runtime_ready"])
        self.assertTrue(all(not role["enabled"] for role in result["roles"].values()))

    def test_each_speed_dependency(self):
        for index, direction in ((0x60FF, "rx"), (0x606C, "tx")):
            c, t = fixture()
            plan = t["slaves"][0]["planned_pdo_entries"]
            plan[direction] = [e for e in plan[direction] if e["index"] != index]
            self.assertEqual(self.result(c, t)["decision"], "requirements_satisfied")
            self.assertEqual(self.result(c, t)["features"]["speed"]["state"], "unavailable")
            t["slaves"][0]["use"] = "spindle"
            self.assertEqual(self.result(c, t)["decision"], "blocked")

    def test_diagnostics_do_not_replace_plan(self):
        for field in ("active_pdo_entries", "pre_download_pdo_entries"):
            c, t = fixture()
            t["slaves"][0][field] = t["slaves"][0].pop("planned_pdo_entries")
            result = self.result(c, t)
            self.assertEqual(result["decision"], "pending")
            self.assertEqual(result["roles"]["control_word"]["state"], "plan_missing")

    def test_duplicate_mapping_same_or_different_bits(self):
        for bits in (16, 32):
            c, t = fixture()
            t["slaves"][0]["planned_pdo_entries"]["rx"].append({"index": 0x6040, "subindex": 0, "bits": bits})
            self.assertEqual(self.result(c, t)["roles"]["control_word"]["state"], "duplicate_mapping")

    def test_type_mismatch_including_signedness(self):
        for dtype in ("REAL32", "UDINT"):
            c, t = fixture()
            ev(c, 0x607A)["fields"]["type"]["value"] = dtype
            result = self.result(c, t)
            self.assertEqual(result["roles"]["target_pos"]["state"], "type_mismatch")
            self.assertEqual(result["decision"], "blocked")

    def test_unknown_type(self):
        c, t = fixture()
        ev(c, 0x607A)["fields"]["type"]["value"] = "CUSTOM_ALIAS"
        self.assertEqual(self.result(c, t)["roles"]["target_pos"]["state"], "type_unknown")

    def test_pdo_only_does_not_require_sdo_access(self):
        result = self.result(*fixture(pdo_only=True))
        self.assertEqual(result["decision"], "requirements_satisfied")
        self.assertEqual(result["roles"]["control_word"]["state"], "static_ok")

    def test_synthesized_permissions_are_not_evidence(self):
        c, t = fixture()
        e = ev(c, 0x6040)
        e["origin"] = "pdo_derived"
        self.assertEqual(self.result(c, t)["roles"]["control_word"]["state"], "evidence_insufficient")

    def test_missing_dictionary_is_insufficient_not_unsupported(self):
        c, t = fixture()
        c["devices"][0]["device"]["evidence"]["entries"] = []
        result = self.result(c, t)
        self.assertEqual(result["decision"], "blocked")
        self.assertEqual(result["roles"]["control_word"]["state"], "evidence_insufficient")

    def test_optional_evidence_gap(self):
        c, t = fixture()
        ev(c, 0x603F)["fields"]["pdo_mapping"]["origin"] = "unknown"
        result = self.result(c, t)
        self.assertEqual(result["decision"], "requirements_satisfied")
        self.assertEqual(result["features"]["error_code"]["state"], "unavailable")

    def test_parser_modified_and_duplicate_evidence(self):
        c, t = fixture()
        ev(c, 0x6040)["fields"]["bits"]["origin"] = "parser_modified"
        self.assertEqual(self.result(c, t)["roles"]["control_word"]["state"], "evidence_conflict")
        c, t = fixture()
        c["devices"][0]["device"]["evidence"]["entries"].append(copy.deepcopy(ev(c, 0x6040)))
        self.assertEqual(self.result(c, t)["roles"]["control_word"]["state"], "evidence_conflict")

    def test_warning_prevents_promotion(self):
        c, t = fixture()
        c["devices"][0]["device"]["evidence"]["diagnostics"] = [{"code": "pdo_mapping_retargeted"}]
        self.assertEqual(self.result(c, t)["roles"]["control_word"]["state"], "evidence_diagnostic")

    def test_raw_pdo_type_conflict(self):
        c, t = fixture(pdo_only=True)
        tree = c["devices"][0]["device"]["evidence"]["declarations"]["device"]
        entry = tree["children"][0]["children"][1]
        next(x for x in entry["children"] if x["tag"] == "DataType")["text"] = "INT"
        self.assertEqual(self.result(c, t)["roles"]["status_word"]["state"], "type_mismatch")

    def test_revision_and_use_validation(self):
        c, t = fixture()
        t["slaves"][0]["revision"] += 1
        self.assertEqual(self.result(c, t)["decision"], "blocked")
        t["slaves"][0]["use"] = "wrong"
        with self.assertRaisesRegex(ValueError, "use"):
            self.result(c, t)

    def test_topology_and_identity_validation(self):
        c, t = fixture()
        t["slaves"].append(copy.deepcopy(t["slaves"][0]))
        with self.assertRaisesRegex(ValueError, "重复"):
            self.result(c, t)
        t["slaves"][1]["slave_pos"] = 2
        with self.assertRaisesRegex(ValueError, "连续"):
            self.result(c, t)
        c, t = fixture()
        c["devices"][0]["device"]["revision"] += 1
        with self.assertRaisesRegex(ValueError, "身份"):
            self.result(c, t)

    def test_v1_requires_reimport(self):
        c, t = fixture()
        c["schema_version"] = 1
        with self.assertRaisesRegex(ValueError, "重建"):
            self.result(c, t)

    def test_same_model_slaves_have_independent_results(self):
        c, t = fixture()
        second = copy.deepcopy(t["slaves"][0])
        second.update(slave_pos=1, use="spindle")
        second["planned_pdo_entries"]["rx"] = [e for e in second["planned_pdo_entries"]["rx"]
                                                 if e["index"] != 0x60FF]
        t["slaves"].append(second)
        report = audit(c, t, self.rules)
        self.assertEqual([s["decision"] for s in report["slaves"]], ["requirements_satisfied", "blocked"])

    def test_pdo_and_object_mapping_conflict(self):
        c, t = fixture(pdo_only=True)
        entry = ev(c, 0x6040)
        entry["origin"] = "dictionary"
        for name, value in (("bits", 16), ("type", "UINT"), ("pdo_mapping", "T")):
            entry["fields"][name].update(origin="xml", value=value)
        self.assertEqual(self.result(c, t)["roles"]["control_word"]["state"], "evidence_conflict")

    def test_unassigned_stays_pending(self):
        c, t = fixture()
        t["slaves"][0]["use"] = "unassigned"
        self.assertEqual(self.result(c, t)["decision"], "pending")

    def test_production_parser_defaults_comments_and_custom_torque(self):
        base = {"vendor_id": KEY[0], "product_code": KEY[1], "type": "servo", "name": "test",
                "profile": "ds402"}
        custom = {**base, "revision": KEY[2], "profile": "custom",
                  "objects": {n: {"index": str(i)} for n, (i, _, _, _) in EXPECTED.items()}}
        custom["objects"]["target_torque"] = {"index": "0x6071"}
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "rules.json"
            path.write_text("/* inline */" + json.dumps({"version": 1, "devices": [base, custom]}) + "// end")
            rules = load_rules(path)
            matches = rules.query([KEY, (KEY[0], KEY[1], 99)])["matches"]
            self.assertEqual(matches[0]["objects"]["target_torque"]["type"], "INT")
            self.assertEqual(matches[1]["profile"], "ds402")
            custom["objects"]["status_word"]["bits"] = 32
            path.write_text(json.dumps({"version": 1, "devices": [custom]}))
            with self.assertRaises(ValueError):
                load_rules(path)


if __name__ == "__main__":
    unittest.main()
