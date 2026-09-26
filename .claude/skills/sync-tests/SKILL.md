---
name: sync-tests
description: Make the Python tests match the user's markup in TESTS.md (Status column: keep / drop / change / wanted / question), run the suites, and reset the markup. Use when the user says "sync the tests" or has edited TESTS.md.
---

# Sync tests to TESTS.md

`TESTS.md` is the user's reviewable statement of what the tests check. The user
edits its Status column; this skill makes the code follow. Never edit a test
for a reason that is not in the file, and never change the file's meaning
without saying so.

## 1. Read the markup

Read `TESTS.md` in full. Collect every row whose Status is not `keep`, plus any
row whose "Proves" / "Would catch" text no longer matches what the named test
actually does (compare against the Python; treat a mismatch as `change`).

Also reconcile both directions:
- A test function in any `test/` folder with no row: add a row with a
  plain-language description and status `keep`, and tell the user it was
  missing.
- A row with no test function and status `keep`: report it as a gap; do not
  invent a test silently. Ask whether it should become `wanted` or be removed.

If there is nothing to do, say so and stop.

## 2. Plan before touching code

List each action in one line each: "drop X", "change Y: <what>", "write Z".
For `question:` rows, answer in chat right away and leave the row as is until
the user replaces the status.

For `wanted` rows, check the "Blocked on" column. If it names a `DECISIONS.md`
entry that has no answer, do not write the test; say which decision is needed.
If the test would need behaviour the code does not have yet, say so: a test is
not a way to sneak in a feature.

If any action touches `esp32_bridge`, `robot_model.yaml`, joint centres, or
kinematics limits, the `hardware-change` checklist applies on top of this.

## 3. Apply

- `drop`: delete the function. If a fixture or helper is now unused, remove it
  too. Remove the row.
- `change`: rewrite the test to match the user's words. Update "Proves" and
  "Would catch". Keep the test name honest; rename if the meaning changed.
- `wanted`: write the test in the package's `test/` folder following the
  nearest existing test's pattern (pure function → `test_kinematics.py` style;
  node → the stub-services style). Move the row from **Wanted** into the right
  section table with status `keep`.
- Never weaken a test to make it pass. If a `change` request would make a test
  pass without checking anything, say so and ask.

## 4. Verify

Run each package separately (they share test file names):
```bash
cd ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 -m pytest src/facadebot_description/test -q -p no:cacheprovider
python3 -m pytest src/esp32_bridge/test -q -k 'not flake8 and not pep257 and not copyright' -p no:cacheprovider
python3 -m pytest src/facade_control/test -q -k 'not flake8 and not pep257 and not copyright' -p no:cacheprovider
```
A failure in a new or changed test is reported with the output, not patched
around. A failure in an untouched test is a real regression; stop and report.

## 5. Close out

- Reset every handled Status to `keep`.
- Update the test counts in section headings and in each package `CLAUDE.md`.
- Update `STATUS.md` if a test count or coverage claim there changed.
- Report: what was dropped, changed, written; what was refused and why; the
  three suite results.
