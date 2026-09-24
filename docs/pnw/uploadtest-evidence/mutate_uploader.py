"""Mutation harness for the restored uploader suite.

Discipline this repo requires (CLAUDE.md Rule 2, and the five 'checks that could not fail' found
this week): every mutant is compile()-checked AND anchor-checked for EXACTLY ONE match before it is
allowed to count. A mutant that does not build, or whose anchor matched zero or many times, is
reported as NOT BUILT -- never silently as 'killed'."""
import ast, os, subprocess, sys, pathlib

SRC = pathlib.Path("system/loggerd/uploader.py")
TESTS = "system/loggerd/tests/test_uploader.py"
ORIG = SRC.read_text()

MUTANTS = [
  ("M1 dcamera leaks: drop the key-prefix tier that keeps driver-cam unpicked",
   "    for name, key, fn in upload_files:\n      if key.startswith(PNW_LOG_PREFIXES):\n        return name, key, fn",
   "    for name, key, fn in upload_files:\n      return name, key, fn"),

  ("M2 creation order broken: reverse the per-directory sort",
   "    paths = sorted(paths, key=get_directory_sort)",
   "    paths = sorted(paths, key=get_directory_sort, reverse=True)"),

  ("M3 SILENT DATA LOSS returns: mark uploaded regardless of success",
   "    if success:\n      # tag file as uploaded\n      try:\n        setxattr(fn, UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE)",
   "    if True:\n      # tag file as uploaded\n      try:\n        setxattr(fn, UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE)"),

  ("M4 duplicate uploads: never mark a file uploaded, so it is picked again",
   "        setxattr(fn, UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE)\n      except OSError:",
   "        pass\n      except OSError:"),

  ("M5 rlog silently dropped from the firehose set",
   'FIREHOSE_FILES = {"rlog", "rlog.zst", "fcamera.hevc", "ecamera.hevc"}',
   'FIREHOSE_FILES = {"fcamera.hevc", "ecamera.hevc"}'),

  ("M6 logs-before-video gate removed: pass 2 (HD) runs while pass-1 logs are still pending (cesarchive2pnw)",
   "at_home, onroad, parked, defer_hd) and p1 is None:",
   "at_home, onroad, parked, defer_hd):"),

  ("M14 boot/crash tier swapped below the qlog tier: boot stops going first",
   "    for name, key, fn in upload_files:\n      if any(f in fn for f in self.immediate_folders):\n        return name, key, fn\n\n    for name, key, fn in upload_files:\n      if name in self.immediate_priority:\n        return name, key, fn",
   "    for name, key, fn in upload_files:\n      if name in self.immediate_priority:\n        return name, key, fn\n\n    for name, key, fn in upload_files:\n      if any(f in fn for f in self.immediate_folders):\n        return name, key, fn"),

  # M7 was first written as PASS2_INTERLEAVE 4 -> 1 and SURVIVED. That mutant was mis-specified, not
  # a coverage hole: gen_sequence DERIVES the expectation from PASS2_INTERLEAVE, so mutating the
  # constant moves the contract and the expectation together. That is the documented, intended
  # property -- a constant bump should not fail the test, a reordering should. Rewritten below as
  # the ordering mutation actually meant: drop the rlog-first tier so video can precede rlog.
  ("M7 pass-2 priority dropped: video no longer waits behind rlog",
   "    for name, key, fn in self.list_upload_files(metered, pass2=True):\n      if name in PASS2_PRIORITY_FILES:\n        return name, key, fn\n      if fallback is None:\n        fallback = (name, key, fn)",
   "    for name, key, fn in self.list_upload_files(metered, pass2=True):\n      if fallback is None:\n        fallback = (name, key, fn)"),
]

def run_tests() -> tuple[bool, str]:
    """Report WHY a mutant died, not just that it did.

    `-q` truncates every assertion message to "Ass...", so a mutant killed by the wrong assertion
    (a TypeError, a fixture error, an unrelated test) is indistinguishable from one killed by the
    check it was written for. That is the exact defect this repo keeps finding in its own tests, so
    the harness must not reproduce it: widen COLUMNS and pull the first `E   AssertionError` line."""
    env = {**os.environ, "COLUMNS": "200"}
    r = subprocess.run([sys.executable, "-m", "pytest", TESTS, "-q", "-p", "no:cacheprovider", "-x"],
                       capture_output=True, text=True, timeout=900, env=env)
    out = r.stdout or ""
    why = next((ln.strip()[:150] for ln in out.splitlines()
                if ln.startswith("E ") and "Error" in ln), "")
    tail = out.strip().splitlines()[-1] if out.strip() else ""
    return r.returncode == 0, (f"{tail}\n           via: {why}" if why else tail)

ok, base = run_tests()
print(f"BASELINE: {'GREEN' if ok else 'RED'}  ({base})")
if not ok:
    print("baseline is red -- mutation results would be meaningless"); sys.exit(1)

killed = survived = notbuilt = 0
for name, old, new in MUTANTS:
    n = ORIG.count(old)
    if n != 1:
        print(f"NOT BUILT  {name}\n           anchor matched {n} times, must be exactly 1"); notbuilt += 1; continue
    mutated = ORIG.replace(old, new)
    try:
        ast.parse(mutated)
    except SyntaxError as e:
        print(f"NOT BUILT  {name}\n           mutant does not parse: {e}"); notbuilt += 1; continue
    SRC.write_text(mutated)
    try:
        passed, last = run_tests()
    finally:
        SRC.write_text(ORIG)
    if passed:
        print(f"*** SURVIVED  {name}\n              the suite does NOT detect this"); survived += 1
    else:
        print(f"KILLED     {name}\n           ({last})"); killed += 1

print(f"\n{len(MUTANTS)} mutants: {killed} KILLED, {survived} SURVIVED, {notbuilt} NOT BUILT")
assert SRC.read_text() == ORIG, "source not restored!"
print("source restored byte-identical: OK")
sys.exit(1 if (survived or notbuilt) else 0)
