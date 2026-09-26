#!/usr/bin/env python3
"""离线 ESI 目录：按 Vendor ID / Product Code / Revision 精确索引。"""

import argparse
import json
import os
import pathlib
import sys
import tempfile

from esi_evidence import ADAPTER_VERSION, SCHEMA_VERSION, import_file


def identity(device):
    return (device["vendor_id"], device["product_code"], device["revision"])


def build(directory, output, extractor):
    directory = directory.resolve()
    files = sorted(path for path in directory.rglob("*") if path.suffix.lower() == ".xml")
    if not files:
        raise ValueError(f"目录中没有 ESI XML：{directory}")

    catalog = {}
    parser_metadata = None
    import_diagnostics = []
    for path in files:
        parsed = import_file(path, extractor)
        if parser_metadata is not None and parser_metadata != parsed["parser"]:
            raise ValueError("导入期间解析器版本变化")
        parser_metadata = parsed["parser"]
        import_diagnostics.extend({"source": str(path), **d} for d in parsed["import_diagnostics"])
        digest = parsed["sha256"]
        for ordinal, device in enumerate(parsed["devices"]):
            source = {"path": str(path), "sha256": digest, "device_ordinal": ordinal}
            key = identity(device)
            if key in catalog:
                previous = catalog[key]
                if previous["device"] != device:
                    raise ValueError(
                        f"相同身份的 ESI 描述冲突 {key}："
                        f"{previous['source']} 与 {path}"
                    )
                previous["sources"].append(source)
            else:
                catalog[key] = {
                    "identity": dict(zip(("vendor_id", "product_code", "revision"), key)),
                    "source": str(path),
                    "sha256": digest,
                    "sources": [source],
                    "device": device,
                }
    output.parent.mkdir(parents=True, exist_ok=True)
    document = json.dumps(
        {"schema_version": SCHEMA_VERSION, "parser": parser_metadata,
         "adapter_version": ADAPTER_VERSION, "import_diagnostics": import_diagnostics,
         "devices": [catalog[k] for k in sorted(catalog)]},
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
    if catalog.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("索引格式已升级到 v2，请从原始 ESI 重新导入")
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
