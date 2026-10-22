---
name: PStop Meeting Notes
description: Draft a Protective Stop meeting record from a meeting transcript, following docs/process/meeting_procedure.md. Use when the user shares a PStop meeting transcript or asks for meeting notes, a meeting record, or action items from a PStop documentation, design or review meeting.
---

<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# PStop Meeting Notes

Turn a meeting transcript into a public meeting record under `docs/process/meetings/`.

`docs/process/meeting_procedure.md` is the source of truth. Read it in full before drafting.
Where this skill and the procedure disagree, follow the procedure and tell the user about
the mismatch.

## Inputs

- The transcript, as a file path or pasted text. Never copy it into the repository.
- The meeting date. Take it from the transcript header if it has one, otherwise ask.
- The invite, if the user has it: agenda, required and optional attendees, pre-reads.

## Steps

1. **Read the procedure and the latest records.** Read `docs/process/meeting_procedure.md`.
   List `docs/process/meetings/` and read the most recent record to collect agenda items
   that weren't covered and action items that are still open. These go in the new record's
   carried-over section.

2. **Read the whole transcript before writing.** Room microphones often label everyone in
   the room with one speaker's name, and automatic transcripts mishear names and terms.
   Work out who said what from context, and keep a list of every attribution you're unsure
   of.

3. **Extract:**
   - **Agenda**: take it from the invite or from what was stated at the start, then give
     each item an outcome: covered, partly covered, or not covered (carried to the next
     meeting). If there was no agenda, say so in the record.
   - **Attendees**: required or optional, who was absent, and whether quorum was met. The
     invite decides who is required. Without the invite, ask the user.
   - **Decisions**: what was agreed, with the reason when it isn't obvious. Provisional if
     quorum wasn't met.
   - **Action items**: one owner each, and a due date given as an ETA or a DFD (date for a
     date). If the transcript gives no date, write `DFD needed` and ask the user.
   - **Review findings** and their disposition, for review meetings.
   - **Readiness progress**: readiness items the meeting moved forward, even if no status
     changed.

4. **Apply the public filter.** Records are public. Use the procedure's *What goes in the
   public record* table. Leave out customers and sites, commercial terms, credentials,
   internal hostnames and product names kept out of the public tree, personnel matters,
   off-topic discussion, and third-party proprietary material such as an assessor's
   checklist wording or IDs.

5. **Write the record** at `docs/process/meetings/YYYYMMDD_<topic>.md` using the procedure's
   record template. The title names the topic in Title Case with no date. Link pull requests
   and issues by URL. Link other docs pages by relative filename.

6. **Check it.**
   - `npm run build` in `website/` (run `npm ci` first if `node_modules` is missing). Broken
     links fail the build.
   - `pre-commit run --files <record>`.

7. **Report back** with:
   - the record path,
   - every uncertain attribution, attendee role or owner, as questions,
   - action items without a due date,
   - anything left out by the public filter that the user may want tracked internally.

   Don't open the pull request or file issues until the user has answered. Then follow
   *After the meeting* in the procedure: one issue per action item, and a pull request
   requesting review from the required attendees only.
