#!/usr/bin/env python3
"""WSL/Linux: python3 test/run_driver_tests.py; only SDK headers are used."""
import argparse
import copy
import json
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description="Build or run HAL offline regression cases")
parser.add_argument("--build-only", action="store_true",
                    help="compile test executables without running them")
parser.add_argument("--output-dir", type=Path,
                    help="directory for executables, fixtures and build logs")
options = parser.parse_args()
OUT = options.output_dir.resolve() if options.output_dir else Path(tempfile.mkdtemp(prefix="hal-tests-"))
OUT.mkdir(parents=True, exist_ok=True)
BUILD_ONLY = options.build_only
ENV = dict(os.environ, ASAN_OPTIONS="detect_leaks=1:halt_on_error=1",
           UBSAN_OPTIONS="halt_on_error=1:print_stacktrace=1")
FLAGS = ["-g", "-O1", "-Wall", "-Wextra", "-Werror", "-fno-omit-frame-pointer",
         "-fno-pie", "-no-pie", "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
         "-ffunction-sections", "-fdata-sections", "-Isrc", "-Isrc/Greemaster"]
def run(name, args, input=None):
    result = subprocess.run(list(map(str, args)), cwd=ROOT, env=ENV, input=input,
                            text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    (OUT/(name+".log")).write_text(result.stdout)
    if result.returncode:
        print(result.stdout[-8000:])
        raise SystemExit(f"FAIL {name}: exit {result.returncode}; logs: {OUT}")
    return result.stdout

def build(name, sources, cpp=False, extra=()):
    exe = OUT/name
    run("build-"+name, ["g++" if cpp else "gcc", "-std=c++17" if cpp else "-std=gnu11",
        *FLAGS, *extra, *sources, "-Wl,--gc-sections", "-o", exe])
    return exe

for source in sorted((ROOT/"src/Greemaster").glob("*.c")) + list((ROOT/"src/common").glob("*.c")):
    run("syntax-"+source.stem, ["gcc", "-std=gnu11", "-Wall", "-Wextra", "-Werror",
        "-Isrc", "-Isrc/Greemaster", "-fsyntax-only", source])
for compiler, language, std in [("gcc","c","c11"),("g++","c++","c++17")]:
    run("abi-"+std, [compiler, "-std="+std, "-pedantic-errors", "-Wall", "-Wextra",
        "-Werror", "-Iinclude", "-x", language, "-fsyntax-only", "-"],
        '#include "hal_c_api.h"\n')
print("Source and public ABI checks passed")

config_test = build("config_regression", ["test/config_regression.c", "src/internal/hal_config.c"],
                    extra=["-Iinclude"])
if not BUILD_ONLY:
    print(run("config_regression", [config_test]).strip())

context_test = build("context_regression", ["test/context_regression.c",
                     "src/internal/hal_context.c", "src/internal/hal_config.c"],
                     extra=["-Iinclude", "-Wl,--wrap=calloc", "-lm", "-pthread"])
if not BUILD_ONLY:
    print(run("context_regression", [context_test]).strip())

driver = build("driver_regression", ["test/driver_regression.c",
    "src/Greemaster/main_demo.c", "src/Greemaster/slave_list.c", "src/Greemaster/servo_step.c",
    "src/Greemaster/entry_access.c", "src/Greemaster/device_table.c", "src/common/devdict.c"],
    extra=["-Wno-unused-parameter", "-Wl,--wrap=calloc", "-Wl,--wrap=clock_gettime"])
if not BUILD_ONLY:
    print(run("driver_regression", [driver]).strip().splitlines()[-1])

ds402 = build("ds402", ["test/ds402_test.c"])
if not BUILD_ONLY:
    print("DS402:", run("ds402", [ds402]).strip().splitlines()[-1])
base = build("device_base", ["test/device_base_test.cpp","src/internal/DeviceBase.cpp"], cpp=True)
if not BUILD_ONLY:
    print("DeviceBase:", run("device_base", [base]).strip().splitlines()[-1])

entry = dict(vendor_id=1, product_code=2, revision="*", type="servo", profile="ds402", name="Servo")
valid = dict(version=1, devices=[entry])
cases = []
def case(name, data, ok=False):
    path = OUT/(name+".json")
    path.write_text(data if isinstance(data,str) else json.dumps(data))
    cases.append(("+" if ok else "-")+str(path))
case("valid", valid, True)
case("comments", "// comment\n"+json.dumps(valid)+" /* comment */", True)
case("tail", json.dumps(valid)+" trailing")
case("comment_unclosed", json.dumps(valid)+" /*")
case("version_missing", dict(devices=[]))
case("version_bad", dict(version=999, devices=[]))
case("duplicate_top_key", '{"version":1,"version":1,"devices":[]}')
case("null_top", "null")
case("bad_json", '{"version":1,"devices":[}')
case("too_deep", "["*20+"0"+"]"*20)
for field, values in {
    "vendor_id":[-1, 4294967296, 1.5, "garbage", "0x", "1tail", "-1", "", None, True],
    "product_code":[-1, 4294967296],
    "revision":[-1, 4294967296, "junk", None, False],
    "name":["x"*64],
    "type":["unknown","servo"+"x"*50],
    "profile":["unknown","none"]
}.items():
    for i,value in enumerate(values):
        doc=copy.deepcopy(valid);doc["devices"][0][field]=value
        case(f"{field}-{i}",doc)
doc=copy.deepcopy(valid);doc["devices"][0]["vendro_id"]=1
case("unknown_field",doc)
case("duplicate_match",dict(version=1,devices=[entry,entry]))
for literal in ["01", "1e2", "1.0", "--1", "99999999999999999999999999999"]:
    case("number-"+str(len(cases)),json.dumps(valid).replace('"vendor_id": 1', '"vendor_id": '+literal))
case("bad_escape",json.dumps(valid).replace("Servo",r"\q"))
case("bad_unicode",json.dumps(valid).replace("Servo",r"\uD800"))
case("nul_string",json.dumps(valid).replace("Servo",r"\u0000"))
case("unicode",json.dumps(valid).replace("Servo",r"\u4e2d\u6587\uD83D\uDE00"),True)
doc=copy.deepcopy(valid);doc["devices"][0]["vendor_id"]="0xFFFFFFFF"
case("hex",doc,True)
doc=copy.deepcopy(valid);doc["devices"][0]["vendor_id"]="010"
case("decimal_string",doc,True)
custom=copy.deepcopy(valid)
custom["devices"][0]["profile"]="custom"
roles={"status_word":(0x6041,16,"tx"),"control_word":(0x6040,16,"rx"),
       "target_pos":(0x607A,32,"rx"),"actual_pos":(0x6064,32,"tx"),
       "target_speed":(0x60FF,32,"rx"),"actual_speed":(0x606C,32,"tx"),
       "error_code":(0x603F,16,"tx"),"op_mode":(0x6060,8,"rx"),
       "mode_display":(0x6061,8,"tx")}
custom["devices"][0]["objects"]={k:dict(index=hex(v[0]),bits=v[1],dir=v[2]) for k,v in roles.items()}
case("custom",custom,True)
for field,values in {"index":[0,65536,-1,"bad"],"sub":[256,-1,"bad"],
                      "bits":[24,264,0,32],"dir":["rx",0],"bitz":[16]}.items():
    for i,value in enumerate(values):
        doc=copy.deepcopy(custom)
        doc["devices"][0]["objects"]["status_word"][field]=value
        case(f"object-{field}-{i}",doc)
doc=copy.deepcopy(custom);del doc["devices"][0]["objects"]["mode_display"]
case("missing_mode_display",doc)
doc=copy.deepcopy(custom);doc["devices"][0]["objects"]["error_code"]["index"]="0x6041"
case("duplicate_object",doc)
cases.append("-"+str(OUT/"missing.json"))
dictionary=build("dictionary_regression",["test/dictionary_regression.c","src/common/devdict.c"], extra=["-Wl,--wrap=malloc"])
if not BUILD_ONLY:
    print(run("dictionary_regression",[dictionary,ROOT/"src/Greemaster/devices.json",*cases]).strip())
    print("ASan/UBSan and leak checks passed. Logs:", OUT)
else:
    print("Test executables compiled without running. Output:", OUT)
