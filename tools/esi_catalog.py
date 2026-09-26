#!/usr/bin/env python3
"""离线 ESI 目录：按 Vendor ID / Product Code / Revision 精确索引。"""

import argparse
import hashlib
import json
import os
import pathlib
import subprocess
import sys
import tempfile


def identity(device):
    return (device["vendor_id"], device["product_code"], device["revision"])


def build(directory, output, extractor):
    directory = directory.resolve()
    files = sorted(path for path in directory.rglob("*") if path.suffix.lower() == ".xml")
    if not files:
        raise ValueError(f"目录中没有 ESI XML：{directory}")

    catalog = {}
    for path in files:
        result = subprocess.run([str(extractor), str(path)], capture_output=True, text=True)
        if result.returncode:
            raise ValueError(f"解析失败：{path}\n{result.stderr.strip()}")
        parsed = json.loads(result.stdout)
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        for device in parsed["devices"]:
            key = identity(device)
            if key in catalog:
                previous = catalog[key]
                if previous["device"] != device:
                    raise ValueError(
                        f"相同身份的 ESI 描述冲突 {key}："
                        f"{previous['source']} 与 {path}"
                    )
                previous["sources"].append({"path": str(path), "sha256": digest})
            else:
                catalog[key] = {
                    "identity": dict(zip(("vendor_id", "product_code", "revision"), key)),
                    "source": str(path),
                    "sha256": digest,
                    "sources": [{"path": str(path), "sha256": digest}],
                    "device": device,
                }
    output.parent.mkdir(parents=True, exist_ok=True)
    document = json.dumps(
        {"schema_version": 1, "devices": [catalog[k] for k in sorted(catalog)]},
        ensure_ascii=False, indent=2,
    ) + "\n"
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=output.parent,
                                         prefix=".esi_catalog_", delete=False) as stream:
            temporary = pathlib.Path(stream.name)
            stream.write(document)
        os.replace(temporary, output)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    print(f"已索引 {len(catalog)} 个设备，来源 {len(files)} 个 ESI：{output}")


def lookup(catalog_path, vendor, product, revision):
    catalog = json.loads(catalog_path.read_text(encoding="utf-8"))
    key = (vendor, product, revision)
    for item in catalog["devices"]:
        if identity(item["identity"]) == key:
            print(json.dumps(item, ensure_ascii=False, indent=2))
            return
    raise ValueError(f"未找到精确匹配的 ESI：{key}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    create = commands.add_parser("build", help="扫描本地 ESI 目录并建立索引")
    create.add_argument("directory", type=pathlib.Path)
    create.add_argument("output", type=pathlib.Path)
    create.add_argument("--extractor", type=pathlib.Path,
                        default=pathlib.Path(__file__).resolve().parent.parent / "build/esi_extract")
    find = commands.add_parser("lookup", help="按 EEPROM 身份精确查询")
    find.add_argument("catalog", type=pathlib.Path)
    find.add_argument("vendor_id", type=lambda value: int(value, 0))
    find.add_argument("product_code", type=lambda value: int(value, 0))
    find.add_argument("revision", type=lambda value: int(value, 0))
    args = parser.parse_args()
    try:
        if args.command == "build":
            build(args.directory, args.output, args.extractor)
        else:
            lookup(args.catalog, args.vendor_id, args.product_code, args.revision)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        parser.exit(1, f"错误：{error}\n")


if __name__ == "__main__":
    main()
