#!/usr/bin/env python3
"""capnpfork2pnw: build the REAL-log fixtures for test_capnp_fork_ordinals_pnw.py.

Each fixture is a real qlog segment recorded by the car BEFORE capnpfork2pnw, trimmed to the messages
the tests need (initData, every onroadEvents, and -- where asked -- pandaStates). Trimming keeps each
kept message's ORIGINAL BYTES exactly as the car wrote them; nothing is re-serialized.

Next to each fixture goes <name>.expected.json.zst: the same messages decoded with the WRITER'S OWN schema
(the cereal/ of the commit in its initData.gitCommit, read from git), which is the ground truth the
migration's output is compared against. That decode runs in a child process: two schemas that share
capnp IDs cannot be loaded into one process.

  python3 make_fixtures.py <name>=<source qlog.zst>[:pandaStates] ...
  python3 make_fixtures.py --snapshot <git ref> <out.json.zst>

--snapshot freezes the WIRE FACTS of log.capnp + legacy.capnp at <ref> (every struct and enum by its
64-bit node id: each member's ordinal, name, type, union discriminant and data offset) for the
whole-schema guard in test_capnp_fork_ordinals_pnw.py. <ref> is the upstream commit this fork is
built on; the snapshot is what "the fork added nothing to upstream's schema" is measured against.

Run with the openpilot venv's python from the repo root. It imports nothing from openpilot, only
pycapnp and zstandard, precisely so the NEW schema can never leak into the ground truth.
"""
import json
import os
import struct
import subprocess
import sys
import tempfile

import zstandard

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "../../../.."))
FLAGS = ("enable", "noEntry", "warning", "userDisable", "softDisable", "immediateDisable",
         "preEnable", "permanent", "overrideLateral", "overrideLongitudinal")


def split_messages(raw: bytes) -> list[bytes]:
  """Split a stream of capnp messages (standard framing) into the exact byte range of each."""
  out, i = [], 0
  while i < len(raw):
    nseg = struct.unpack_from("<I", raw, i)[0] + 1
    sizes = struct.unpack_from(f"<{nseg}I", raw, i + 4)
    header = 4 + 4 * nseg
    header += header % 8  # the segment table is padded to a word boundary
    end = i + header + 8 * sum(sizes)
    if end > len(raw):
      raise ValueError(f"truncated message at byte {i}")
    out.append(raw[i:end])
    i = end
  return out


def git(*args: str, cwd: str = REPO) -> bytes:
  return subprocess.run(["git", "-C", cwd, *args], capture_output=True, check=True).stdout


def extract_schema(commit: str, dest: str) -> None:
  """The writer's cereal/ at `commit`, with car.capnp resolved through its opendbc pin."""
  os.makedirs(os.path.join(dest, "include"))
  for f in ("log.capnp", "custom.capnp", "legacy.capnp", "include/c++.capnp"):
    with open(os.path.join(dest, f), "wb") as fh:
      fh.write(git("show", f"{commit}:cereal/{f}"))
  pin = git("rev-parse", f"{commit}:opendbc_repo").decode().strip()
  with open(os.path.join(dest, "car.capnp"), "wb") as fh:
    fh.write(git("show", f"{pin}:opendbc/car/car.capnp", cwd=os.path.join(REPO, "opendbc_repo")))


def _type(t) -> str:
  w = t.which()
  if w == "struct":
    return f"struct:{t.struct.typeId:#x}"
  if w == "enum":
    return f"enum:{t.enum.typeId:#x}"
  if w == "list":
    return f"list<{_type(t.list.elementType)}>"
  return w


def schema_nodes(*modules) -> dict:
  """Wire facts of every struct and enum reachable from the given pycapnp modules (nested types and
  group nodes included): {"<node id>": {"kind", "name", "members": {"<ordinal>": [name, type, disc, offset]}}}.
  Pure pycapnp -- it must run in a process that has loaded ONLY the schema being described."""
  out: dict = {}

  def visit(sch) -> None:
    node = sch.node
    key = f"{node.id:#x}"
    if key in out:
      return
    if node.which() == "enum":
      out[key] = {"kind": "enum", "name": node.displayName,
                  "members": {str(i): e.name for i, e in enumerate(node.enum.enumerants)}}
      return
    if node.which() != "struct":
      return
    members = {}
    out[key] = {"kind": "struct", "name": node.displayName, "members": members}
    for f in node.struct.fields:
      disc = None if f.discriminantValue == 65535 else f.discriminantValue
      if f.which() == "group":
        members[f"group:{f.name}"] = [f.name, f"group:{f.group.typeId:#x}", disc, None]
        visit(sch.fields[f.name].schema)  # a group is its own node, invisible as a module attribute
      else:
        members[str(f.ordinal.explicit)] = [f.name, _type(f.slot.type), disc, f.slot.offset]

  def walk(mod) -> None:
    for name in dir(mod):
      if name.startswith("_"):
        continue
      obj = getattr(mod, name)
      sch = getattr(obj, "schema", None)
      node = getattr(sch, "node", None)
      if node is None or f"{node.id:#x}" in out:
        continue
      visit(sch)
      if node.which() == "struct":
        walk(obj)

  for m in modules:
    walk(m)
  return out


def snapshot_child(schema_dir: str, out_path: str) -> None:
  import capnp
  imports = [os.path.join(schema_dir, "include"), schema_dir]
  log = capnp.load(os.path.join(schema_dir, "log.capnp"), imports=imports)
  legacy = capnp.load(os.path.join(schema_dir, "legacy.capnp"), imports=imports)
  nodes = {k: v for k, v in schema_nodes(log, legacy).items() if v["name"].startswith(("log.capnp:", "legacy.capnp:"))}
  with open(out_path, "wb") as fh:
    fh.write(zstandard.ZstdCompressor(level=19).compress(json.dumps(nodes, separators=(",", ":"), sort_keys=True).encode()))
  print(f"{len(nodes)} nodes")


def decode_child(mode: str, schema_dir: str, msgs_path: str) -> None:
  """Child process. mode "probe": which() and initData only (any schema agrees on those). mode "truth":
  decode everything with the WRITER's schema and print the ground truth."""
  import capnp
  log = capnp.load(os.path.join(schema_dir, "log.capnp"), imports=[os.path.join(schema_dir, "include"), schema_dir])
  with open(msgs_path, "rb") as fh:
    msgs = json.load(fh)
  out = []
  for hexmsg in msgs:
    with log.Event.from_bytes(bytes.fromhex(hexmsg)) as m:
      w = m.which()
      rec = {"logMonoTime": m.logMonoTime, "which": w, "valid": m.valid}
      if w == "initData":
        rec.update(gitCommit=m.initData.gitCommit, dirty=m.initData.dirty)
      elif mode == "probe":
        pass
      elif w == "onroadEvents":
        rec["events"] = [{"name": str(e.name), "raw": e.name.raw, **{f: getattr(e, f) for f in FLAGS}} for e in m.onroadEvents]
      elif w == "pandaStates":
        rec["pandas"] = [{"controlsAllowed": p.controlsAllowed, "controlsAllowedLateral": p.controlsAllowedLateral,
                          "madsDisengageReason": p.madsDisengageReason, "healthPacketMismatch": p.healthPacketMismatch}
                         for p in m.pandaStates]
      out.append(rec)
  json.dump(out, sys.stdout)


def build(name: str, src: str, keep_panda: bool) -> None:
  with open(src, "rb") as fh:
    raw = zstandard.ZstdDecompressor().stream_reader(fh).read()
  msgs = split_messages(raw)

  with tempfile.TemporaryDirectory() as td:
    all_path = os.path.join(td, "all.json")
    with open(all_path, "w") as fh:
      json.dump([m.hex() for m in msgs], fh)
    # Pass 1 finds the writer: which() and initData read the same under any of this fork's schemas.
    probe_dir = os.path.join(td, "probe")
    extract_schema("HEAD", probe_dir)
    probe = subprocess.run([sys.executable, __file__, "--decode", "probe", probe_dir, all_path], capture_output=True, text=True)
    if probe.returncode:
      raise SystemExit(f"probe decode failed: {probe.stderr}")
    decoded = json.loads(probe.stdout)
    inits = [d for d in decoded if d["which"] == "initData"]
    if len(inits) != 1:
      raise SystemExit(f"{src}: expected exactly one initData, found {len(inits)}")
    commit = inits[0]["gitCommit"]

    wanted = {"initData", "onroadEvents"} | ({"pandaStates"} if keep_panda else set())
    kept = [m for m, d in zip(msgs, decoded, strict=True) if d["which"] in wanted]

    writer_dir = os.path.join(td, "writer")
    extract_schema(commit, writer_dir)
    kept_path = os.path.join(td, "kept.json")
    with open(kept_path, "w") as fh:
      json.dump([m.hex() for m in kept], fh)
    truth = subprocess.run([sys.executable, __file__, "--decode", "truth", writer_dir, kept_path], capture_output=True, text=True)
    if truth.returncode:
      raise SystemExit(f"writer-schema decode failed: {truth.stderr}")

  with open(os.path.join(HERE, f"{name}.qlog.zst"), "wb") as fh:
    fh.write(zstandard.ZstdCompressor(level=19).compress(b"".join(kept)))
  expected = {"source": os.path.basename(src), "writerCommit": commit, "messages": json.loads(truth.stdout)}
  with open(os.path.join(HERE, f"{name}.expected.json.zst"), "wb") as fh:
    fh.write(zstandard.ZstdCompressor(level=19).compress(json.dumps(expected, separators=(",", ":")).encode()))
  print(f"{name}: {len(kept)}/{len(msgs)} messages kept, writer {commit[:12]}")


if __name__ == "__main__":
  if sys.argv[1] == "--decode":
    decode_child(sys.argv[2], sys.argv[3], sys.argv[4])
    sys.exit(0)
  if sys.argv[1] == "--snapshot-child":
    snapshot_child(sys.argv[2], sys.argv[3])
    sys.exit(0)
  if sys.argv[1] == "--snapshot":
    with tempfile.TemporaryDirectory() as td:
      extract_schema(sys.argv[2], td + "/s")
      sha = git("rev-parse", sys.argv[2]).decode().strip()
      r = subprocess.run([sys.executable, __file__, "--snapshot-child", td + "/s", sys.argv[3]], capture_output=True, text=True)
      if r.returncode:
        raise SystemExit(f"snapshot failed: {r.stderr}")
      print(f"snapshot of {sha}: {r.stdout.strip()} -> {sys.argv[3]}")
    sys.exit(0)
  for arg in sys.argv[1:]:
    name, rest = arg.split("=", 1)
    src, _, opt = rest.partition(":")
    build(name, src, opt == "pandaStates")
