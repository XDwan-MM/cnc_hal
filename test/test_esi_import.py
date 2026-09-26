#!/usr/bin/env python3
"""ESI 提取及精确身份索引的离线回归。"""

import json
import pathlib
import subprocess
import sys
import tempfile


ROOT = pathlib.Path(__file__).resolve().parent.parent
FIXTURE = ROOT / "test/fixtures/kickcat_multi_device.xml"
CATALOG = ROOT / "tools/esi_catalog.py"


def run(*args, expect=0):
    result = subprocess.run(args, text=True, capture_output=True)
    assert result.returncode == expect, (args, result.stdout, result.stderr)
    return result


def main(extractor):
    parsed = json.loads(run(extractor, FIXTURE).stdout)
    assert parsed["schema_version"] == 1
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
        assert len(catalog["devices"]) == 3
        matched = json.loads(run(sys.executable, CATALOG, "lookup", output,
                                 "0xcafe", "0x2222", "2").stdout)
        assert matched["device"]["name"] == "KickCAT Multi Device Beta r2"
        assert len(matched["sha256"]) == 64
        run(sys.executable, CATALOG, "lookup", output, "0xcafe", "0x2222", "3", expect=1)

        modified = FIXTURE.read_text(encoding="utf-8").replace(
            "KickCAT Multi Device Alpha", "Different Device Alpha")
        (temp / "conflict.xml").write_text(modified, encoding="utf-8")
        result = run(sys.executable, CATALOG, "build", temp, output,
                     "--extractor", extractor, expect=1)
        assert "描述冲突" in result.stderr

        (temp / "broken.xml").write_text("<bad", encoding="utf-8")
        run(extractor, temp / "broken.xml", expect=1)

    print("ESI 提取、版本精确匹配、冲突和错误处理通过")


if __name__ == "__main__":
    main(str(pathlib.Path(sys.argv[1]).resolve()))
