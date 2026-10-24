#!/usr/bin/env python3
"""Exercise real S.BUS/WFLY/Input/RobotCmd sources without changing firmware presets."""
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[2]
tests = Path(__file__).resolve().parent
remote = root / "User_File/Device/Peripheral/Remote"
app = root / "User_File/Application/Input"
robot = root / "User_File/Application/RobotCmd"
center = root / "User_File/System/MessageCenter"
daemon = root / "User_File/System/Daemon"
includes = [f"-I{p}" for p in (tests / "stubs", remote, app, robot, center, daemon)]
flags = ["-g", "-O1", "-Wall", "-Wextra", "-Werror", "-fsanitize=address,undefined",
         "-fno-omit-frame-pointer", "-fno-pie"]
sources = [remote / "sbus.cpp", remote / "wfly.cpp", app / "remote_input.cpp",
           app / "input_state.cpp", app / "source_arbitration.cpp", robot / "RobotCmd.cpp",
           center / "message_center.cpp", daemon / "daemon.cpp", tests / "test_remote.cpp"]
for wfly in (0, 1):
    build = root / "build/tests/remote" / ("wfly" if wfly else "default")
    build.mkdir(parents=True, exist_ok=True)
    # Confirm the public adapter remains usable from C.
    subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", *includes,
                    "-x", "c", "-fsyntax-only", "-"],
                   input='#include "wfly.h"\n', text=True, check=True)
    binary = build / "remote_test"
    subprocess.run(["g++", "-std=c++17", *flags, f"-DH7_REMOTE_WFLY={wfly}", *includes,
                    *map(str, sources), "-no-pie", "-o", str(binary)], check=True)
    for case in ("init", "protocol", "mapping", "input", "recovery"):
        subprocess.run([str(binary), case], check=True)
