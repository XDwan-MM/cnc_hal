#!/usr/bin/env python3
"""无需从站的 ESI/拓扑功能审核回归。"""

import copy
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
from esi_topology_audit import DS402, audit, load_rules  # noqa: E402


KEY = (477, 271601776, 33818120)  # devices.json 中的台达身份


def fixture():
    objects = []
    active = {"rx": [], "tx": []}
    for index, subindex, bits, direction in DS402.values():
        objects.append({"index": index, "entries": [{"subindex": subindex,
                        "bits": bits, "access": (64 | 8) if direction == "rx" else (128 | 1)}]})
        active[direction].append({"index": index, "subindex": subindex, "bits": bits})
    catalog = {"schema_version": 1, "devices": [{
        "identity": dict(zip(("vendor_id", "product_code", "revision"), KEY)),
        "source": "fixture.xml", "device": {"objects": objects}}]}
    topology = {"schema_version": 1, "slaves": [{
        "slave_pos": 0, **catalog["devices"][0]["identity"],
        "use": "position", "active_pdo_entries": active}]}
    return catalog, topology


class AuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rules = load_rules(ROOT / "src/Greemaster/devices.json")

    def result(self, catalog, topology):
        return audit(catalog, topology, self.rules)["slaves"][0]

    def test_ready_requires_esi_and_current_pdo(self):
        catalog, topology = fixture()
        result = self.result(catalog, topology)
        self.assertEqual(result["decision"], "static_eligible")
        self.assertEqual(result["alarm_control"], "undefined_rule")

        del topology["slaves"][0]["active_pdo_entries"]
        result = self.result(catalog, topology)
        self.assertEqual(result["decision"], "pending")
        self.assertEqual(result["roles"]["control_word"]["state"], "pdo_unverified")

    def test_missing_required_and_optional(self):
        catalog, topology = fixture()
        topology["slaves"][0]["active_pdo_entries"]["rx"] = [
            e for e in topology["slaves"][0]["active_pdo_entries"]["rx"]
            if e["index"] != 0x6040]
        result = self.result(catalog, topology)
        self.assertEqual(result["decision"], "blocked")
        self.assertEqual(result["roles"]["control_word"]["state"], "not_in_current_pdo")

        catalog, topology = fixture()
        catalog["devices"][0]["device"]["objects"] = [
            o for o in catalog["devices"][0]["device"]["objects"]
            if o["index"] != 0x603F]
        result = self.result(catalog, topology)
        self.assertEqual(result["decision"], "static_eligible")
        self.assertEqual(result["roles"]["error_code"]["state"], "object_missing")

    def test_esi_gaps_and_revision_mismatch(self):
        catalog, topology = fixture()
        catalog["devices"][0]["device"]["objects"][0]["entries"][0]["access"] = 0
        result = self.result(catalog, topology)
        self.assertEqual(result["decision"], "pending")
        self.assertEqual(result["roles"]["status_word"]["state"], "esi_mapping_unknown")

        catalog, topology = fixture()
        topology["slaves"][0]["revision"] += 1
        result = self.result(catalog, topology)
        self.assertEqual(result["esi"], "missing")
        self.assertEqual(result["decision"], "blocked")

    def test_position_and_spindle_requirements_differ(self):
        catalog, topology = fixture()
        topology["slaves"][0]["active_pdo_entries"]["tx"] = [
            e for e in topology["slaves"][0]["active_pdo_entries"]["tx"]
            if e["index"] != 0x606C]
        self.assertEqual(self.result(catalog, topology)["decision"], "static_eligible")
        topology["slaves"][0]["use"] = "spindle"
        self.assertEqual(self.result(catalog, topology)["decision"], "blocked")

    def test_topology_order_and_duplicates(self):
        catalog, topology = fixture()
        second = copy.deepcopy(topology["slaves"][0])
        second["slave_pos"] = 2
        topology["slaves"].append(second)
        with self.assertRaisesRegex(ValueError, "连续"):
            audit(catalog, topology, self.rules)


if __name__ == "__main__":
    unittest.main()
