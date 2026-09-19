# `uploadtest-evidence` — the mutation proof for the restored uploader suite

`system/loggerd/tests/test_uploader.py` had **4 of 39 tests red on the channel for two weeks**
(bisected: 1 red at `4bb6a2e23b` 08-14 → 4 red at `6ad65ca264` `uploadanywifi2pnw` 09-05, still 4 at
`718079e75a` 09-10). The code was right; the tests encoded stock's 2-file expectation while the fork
moves 4. Restoring them to green is only worth something if they still FAIL when they should, so:

```bash
cd <a pnw worktree at the channel tip>
PYTHONPATH=$PWD:$PWD/opendbc_repo <venv>/python docs/pnw/uploadtest-evidence/mutate_uploader.py
```

Each mutant is `compile()`-checked **and** anchor-checked for **exactly one match** before it counts;
anything else is reported `NOT BUILT`, never silently as killed. The harness restores `uploader.py`
and asserts it is byte-identical afterwards. It prints the **killing assertion** for each mutant
(`COLUMNS=200` + first `E …Error` line) because `-q` truncates messages to `Ass...`, which would make
a mutant killed by the wrong assertion indistinguishable from one killed by the right one — the exact
defect this repo has found in its own tests five times this week.

## Result: 8 mutants, 8 KILLED, 0 survived, 0 not built

| mutant | killed by |
|---|---|
| M1 dcamera leaks (key-prefix tier dropped) | `dcamera.hevc must never leave the device (driver-facing camera)` |
| M2 creation order reversed | `qlog.zst uploaded out of creation order` |
| M3 **mark uploaded regardless of success** | `412'd file was marked uploaded without reaching S3 (silent data loss)` |
| M4 never mark uploaded → duplicates | `a file was uploaded twice` |
| M5 rlog dropped from `FIREHOSE_FILES` | set difference |
| **M6 HD-interleave gate deleted** | `Files uploaded in wrong order` |
| **M14 boot tier swapped below qlog** | `Files uploaded in wrong order` |
| M7 pass-2 rlog-first priority dropped | `TestPass2Priority` |

**M6 and M14 are the reason this file exists.** My first version replaced stock's exact-sequence
assertion with a set-plus-per-kind check, on the stated grounds that the pass interleave made a global
sequence "a race". **That was wrong** — Fable re-measured the 24-key sequence five times and got
byte-identical output, and derived it from `PASS2_INTERLEAVE`: `main()` is single-threaded and the
tests disable every sleep. Both mutants move no file in or out of the set, so they survived the weaker
check; stock would have caught M14. `gen_sequence` rebuilds the exact order **from the constant**
rather than pasting a captured one, so an interleave-constant change updates the expectation while a
genuine reordering still fails — and both now die on it.

M7 was first written as `PASS2_INTERLEAVE 4 → 1` and survived. That was a **mis-specified mutant, not
a coverage hole**: it mutates the very constant `gen_sequence` derives from, so contract and
expectation move together — the documented, intended property. Rewritten as the ordering change it was
meant to be (drop the rlog-first tier), it dies.
