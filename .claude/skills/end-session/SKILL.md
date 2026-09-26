---
name: end-session
description: Close out a FacadeBot work session. Updates STATUS.md and LAUNCH.md, separates what was verified on hardware from what was only tested, and leaves the tree uncommitted.
---

# End of session

Run this before the last message of any session that changed code, hardware state,
or the plan.

1. **Verification ledger.** For each change this session, one line stating one of:
   verified on the arm (what was run, date), verified in unit tests only, or not
   verified. This wording goes into both `STATUS.md` and the final message.
2. **`STATUS.md`.** Update the row for each affected item. Move resolved blockers to
   "Previously resolved" with the date. Rewrite "Next task" so a new session can
   start from it without the conversation. Convert relative dates to absolute.
3. **`LAUNCH.md`.** Confirm every new node, topic, service, script, or command has a
   step, and that changed procedures were updated. If an ESP32 address, port, or
   protocol changed, grep the docs for the old value.
4. **Package docs.** If a package's interfaces or invariants changed, its README and
   its `CLAUDE.md` both say so.
5. **Open decisions.** List any robotics or architecture decision that was raised
   and not settled, with the options as presented, so the user can answer later.
6. **Do not commit or push.** Leave everything in the working tree. `git status`
   to list what changed is fine and useful to include.
7. **Final message.** Lead with what works and what is still unverified. Then what
   the user has to do on hardware next, in `LAUNCH.md` step numbers.
