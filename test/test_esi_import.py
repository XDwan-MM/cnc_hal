#!/usr/bin/env python3
"""ESI 提取及精确身份索引的离线回归。"""

import json
import hashlib
import pathlib
import subprocess
import sys
import tempfile


ROOT = pathlib.Path(__file__).resolve().parent.parent
FIXTURE = ROOT / "test/fixtures/kickcat_multi_device.xml"
CATALOG = ROOT / "tools/esi_catalog.py"
sys.path.insert(0, str(ROOT / "tools"))
from esi_evidence import import_file


def run(*args, expect=0):
    if args[0] == sys.executable:
        args = (args[0], "-B", *args[1:])
    result = subprocess.run(args, text=True, capture_output=True)
    assert result.returncode == expect, (args, result.stdout, result.stderr)
    return result


def main(extractor):
    parsed = json.loads(run(extractor, FIXTURE).stdout)
    assert parsed["schema_version"] == 2
    assert [(d["vendor_id"], d["product_code"], d["revision"])
            for d in parsed["devices"]] == [
                (0xCAFE, 0x1111, 1), (0xCAFE, 0x2222, 1), (0xCAFE, 0x2222, 2)]
    assert parsed["devices"][0]["objects_may_be_synthesized"]

    with tempfile.TemporaryDirectory() as temp:
        temp = pathlib.Path(temp)
        (temp / "devices.xml").write_bytes(FIXTURE.read_bytes())
        output = temp / "catalog.json"
        run(sys.executable, CATALOG, "build", temp, output, "--extractor", extractor)
        catalog = json.loads(output.read_text(encoding="utf-8"))
        assert catalog["schema_version"] == 2
        assert catalog["parser"]["patch"] == "cnc-hal-evidence-1"
        assert catalog["adapter_version"] == "cnc-hal-esi-evidence-1"
        assert len(catalog["devices"]) == 3
        matched = json.loads(run(sys.executable, CATALOG, "lookup", output,
                                 "0xcafe", "0x2222", "2").stdout)
        assert matched["device"]["name"] == "KickCAT Multi Device Beta r2"
        assert len(matched["sha256"]) == 64
        assert matched["sha256"] == hashlib.sha256(FIXTURE.read_bytes()).hexdigest()
        run(sys.executable, CATALOG, "lookup", output, "0xcafe", "0x2222", "3", expect=1)

        modified = FIXTURE.read_text(encoding="utf-8").replace(
            "KickCAT Multi Device Alpha", "Different Device Alpha")
        (temp / "conflict.xml").write_text(modified, encoding="utf-8")
        result = run(sys.executable, CATALOG, "build", temp, output,
                     "--extractor", extractor, expect=1)
        assert "描述冲突" in result.stderr

        (temp / "broken.xml").write_text("<bad", encoding="utf-8")
        run(extractor, temp / "broken.xml", expect=1)

    evidence_tests(extractor)
    print("ESI v2 提取、证据来源、精确匹配、冲突和错误处理通过")


def evidence_tests(extractor):
    fixture = ROOT / "test/fixtures/esi_evidence.xml"
    parsed = import_file(fixture, extractor)
    ev = parsed["devices"][0]["evidence"]
    entries = {(e["index"], e["subindex"]): e for e in ev["entries"]}
    assert entries[0x2000, 0]["fields"]["access"]["value"] == "rw"
    absent = entries[0x2001, 0]["fields"]["access"]
    assert absent["value"] is None and absent["origin"] == "unknown"
    assert absent["parser_value"] != 0  # Parser default must not become evidence.
    derived = entries[0x6FFF, 0]
    assert derived["origin"] == "pdo_derived"
    assert derived["fields"]["access"]["value"] is None
    assert derived["fields"]["bits"]["origin"] == "pdo_declaration"
    assert ev["identity"]["revision"]["value"] == 0
    assert ev["constraints"]["coe"]["PdoAssign"]["value"] == "0"
    assert ev["constraints"]["coe"]["PdoConfig"]["value"] is None
    assert ev["constraints"]["pdos"][0]["fields"]["Fixed"]["value"] == "0"
    assert ev["constraints"]["pdos"][1]["fields"]["Fixed"]["value"] is None
    assert any(d["code"] == "pdo_mapping_retargeted" for d in ev["diagnostics"])
    assert any(t["index"] == 0x1A00 for t in ev["transformations"])
    rewritten = next(t for t in ev["transformations"] if t["index"] == 0x1600)
    assert rewritten["code"] == "object_rewritten"
    changed = entries[0x1600, 1]["fields"]["default_data"]
    assert changed["origin"] == "parser_modified" and changed["value"] is None
    assert changed["declared_value"] == "20000020"
    # KickCAT does not load this DataType/SubItem default into its baseline.
    assert changed["parser_before"] is None
    assert changed["parser_value"] == "2000ff6f"
    assert ev["coverage"]["configuration_authorization"] == "not_evaluated"
    with tempfile.TemporaryDirectory() as temp:
        temp = pathlib.Path(temp)
        source = temp / "a.xml"
        original = fixture.read_text()
        source.write_text(original.replace(' RevisionNo="0"', ''))
        try:
            import_file(source, extractor)
            raise AssertionError("missing identity accepted")
        except ValueError as error:
            assert "身份" in str(error)
        source.write_text(original)
        duplicate = original.replace('</RxPdo>', '</RxPdo><RxPdo Sm="2"><Index>#x1600</Index><Name>Conflict</Name><Entry><Index>#x6FFF</Index><SubIndex>0</SubIndex><BitLen>16</BitLen><DataType>UINT</DataType></Entry></RxPdo>')
        source.write_text(duplicate)
        duplicate_ev = import_file(source, extractor)["devices"][0]["evidence"]
        assert any(d["code"] == "duplicate_pdo_index" for d in duplicate_ev["diagnostics"])
        target = next(e for e in duplicate_ev["entries"] if e["index"] == 0x6FFF)
        assert target["fields"]["bits"]["value"] is None
        assert len(target["pdo_declarations"]) == 2
        source.write_text(original)
        output = temp / "catalog.json"
        run(sys.executable, CATALOG, "build", temp, output, "--extractor", extractor)
        saved = output.read_bytes()
        # Constraint omitted by the old projection must now cause a conflict.
        (temp / "b.xml").write_text(original.replace('<RxPdo Sm="2"', '<RxPdo Virtual="1" Sm="2"'))
        result = run(sys.executable, CATALOG, "build", temp, output, "--extractor", extractor, expect=1)
        assert "描述冲突" in result.stderr
        assert output.read_bytes() == saved  # Failed rebuild is atomic.
        output.write_text('{"schema_version":1,"devices":[]}')
        result = run(sys.executable, CATALOG, "lookup", output, "1", "2", "3", expect=1)
        assert "重新导入" in result.stderr


if __name__ == "__main__":
    main(str(pathlib.Path(sys.argv[1]).resolve()))
