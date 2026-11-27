# Videos

Short videos of tests and demos running on the real hardware. A video shows
what a log cannot: the LED blinking, the motor turning, the robot reaching
the line. List **every** video in the table at the bottom of this page.

## What to record

- **Short:** 15–60 seconds. One test or one feature per video.
- **Say what it is.** Show or say the test ID (e.g. `S2-U03`, `INT-01`) or
  the demo name at the start.
- **Show the commit.** Before recording, run `git rev-parse --short HEAD` and
  keep the terminal in shot for a moment, or put it in the table below.
- **Show the result.** Keep both the hardware and, if it matters, the
  console output in view.
- **Landscape**, steady, good light. No music needed.

## How to share it: unlisted YouTube only

1. Upload the video to YouTube with visibility set to **Unlisted**: anyone
   with the link can watch it, but it does not show up in search. Not
   *Private* (only invited accounts can watch, so the link won't work for
   your team or instructor), and not *Public*.
2. Copy the video's link (e.g. `https://youtu.be/...`).

**Do not commit video files to this repo or attach them on GitHub.** A video
in the repo stays in the git history for good, even if deleted later, and
slows every clone for the whole team. Video files are ignored by
`.gitignore` for this reason.

Then add a row to the table below. If the video is evidence for a test, also
put its link in the *Log / evidence* column of that test's row in
`testing/.../TEST_LOG.md`.

## Video index

Newest at the bottom. Never delete a row.

| Date | Subsystem | Test ID / demo | Commit | Student ID | Link | Notes |
|---|---|---|---|---|---|---|
| 2026-11-25 | S3 / SYS | S3-I02, SYS-03 | `f4e94ad` | 2500708 | <https://youtu.be/GweIDDofwxg> | Barcode pass on the track. Dashboard shows Z decoded at 12:33:51, 29/29 valid elements, mapped to STRAIGHT. |
| 2026-11-25 | S4 / SYS | SYS-04, S4-I02 | `f4e94ad` | 2500708 | <https://youtu.be/4vyPeXknS8E> | Car crossing the hump. Dashboard hump peak 12 mm (HUMP event 12:33:58 in the MQTT capture). No ruler reference in shot. |
| 2026-11-25 | S5 | S5-I09 (trials 1–2), S5-I01 | `f4e94ad` | 2500708 | <https://youtu.be/9nU9wnezGOc> | Round bottle (~70 mm) on the line, 12:35–12:36. Car stops, scans and drives the bypass. Object outside the S5-I09 box spec. |
