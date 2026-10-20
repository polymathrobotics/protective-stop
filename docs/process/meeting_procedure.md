---
title: Meeting Procedure
sidebar_position: 1
---

<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Meeting Procedure

*Date: 2026-10-20.*

This procedure covers Protective Stop meetings whose outcome belongs in the safety case:
documentation syncs, design and requirements reviews, test plan reviews, and any meeting
that changes a status in the [Safety Certification Readiness](safety_certification_readiness.md)
matrix. Each meeting produces a short public record under Meeting Records in this section,
merged by pull request, so the record, the people who agreed to it and the date are all in Git
history. The [Documentation Kickoff](meetings/documentation_kickoff.md) record is an example.

IEC 61508 asks for reviews to be recorded with their participants and results, and for actions
to be followed up to resolution (Part 1 §5.2.6, §6.2.5). This procedure is how we meet that.

## Roles

| Role | Responsibility |
|---|---|
| Organizer | Sends the invite with the agenda, runs the meeting, and makes sure a record is written. |
| Note-taker | Writes the record and opens its pull request. Defaults to the organizer. |
| Attendees | Review the record pull request and confirm it reflects what was agreed. |

## Before the meeting

1. Put the agenda in the invite: the topic, the matrix IDs it covers (for example `RM-REQ-14`),
   and the decision or review the meeting needs.
2. For a review, link the document under review at a fixed commit, for example
   `docs/safety/safety_requirements_specification.md@<commit>`, so everyone reviews the same
   revision.
3. Mark people whose input is optional as optional.

## Opening the meeting (first two minutes)

Say these out loud so they are in the notes and any transcript:

1. Who is attending, and in what role.
2. The topic, and the matrix IDs or IEC 61508 clauses it relates to.
3. What the meeting must produce: a status update, a decision, or a review outcome.
4. Whether the meeting is being transcribed. Transcripts are a note-taking aid only and are
   not committed.

## Closing the meeting (last five minutes)

1. Read back each decision.
2. Read back each action item with its owner. Every action item gets one named owner.
3. Read back any matrix status the meeting agreed to change.
4. Confirm who writes the record.

## After the meeting

Within two working days:

1. **File action items.** Open a GitHub issue for each action item, assigned to its owner,
   and link the meeting record. Action items that only concern internal work outside this
   repository are tracked internally and listed in the record without detail.
2. **Write the record.** Copy the [template](#record-template) to
   `docs/process/meetings/<topic>.md` and fill it in. Follow the naming rules in
   [`AGENTS.md`](/AGENTS.md): the filename and title name the topic, the date goes in the
   italic line under the title, and `sidebar_position` is the next number in sequence so
   records list in order.
3. **Open a pull request** with the record, and request review from the attendees. An
   attendee's approval is their confirmation that the record is accurate. Merge once at
   least one attendee other than the note-taker has approved.
4. **Update the matrix.** A status change agreed in the meeting is made in a pull request
   that links its evidence. That can be the same pull request as the record, or a separate
   one linked from the record.

At the start of each meeting, the organizer reviews the open action items from earlier
records and notes their state in the new record.

## What goes in the public record

Records are public. Write a summary, not a transcript.

| Include | Leave out |
|---|---|
| Attendee names and roles | Video, audio and full transcripts |
| The topic and the matrix IDs or clauses covered | Customer names, sites and commercial terms |
| Decisions, with the reason when it is not obvious | Credentials, keys and internal hostnames |
| Action items with owners and issue links | Third-party proprietary material, such as an assessor's checklists |
| For reviews: the document and commit reviewed, the findings and how each was disposed of | Personnel matters and off-topic discussion |

## Review meetings

A meeting that reviews a work product, such as the safety requirements, the architecture or
a test plan, also records:

- The document and the commit reviewed.
- Who wrote the document. At least one reviewer must not be an author of it.
- Each finding, and whether it was accepted, rejected or deferred, with the reason.
- The outcome: approved, approved with changes, or not approved.

## Cadence

The documentation sync runs every two weeks while the safety case is being built up.
Attendance is optional for anyone not needed for that session's agenda. Review meetings are
scheduled as needed.

## Record template

```markdown
---
title: <Topic in Title Case>
sidebar_position: <next number>
---

<!-- SPDX-FileCopyrightText: <year> Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# <Topic in Title Case>

*Date: YYYY-MM-DD.*

| | |
|---|---|
| Type | Documentation sync / Review / Decision |
| Attendees | Name (role), Name (role) |
| Organizer | Name |
| Note-taker | Name |
| Covers | RM-XXX-nn, IEC 61508 Part n §x.y |
| Document reviewed | path@commit (review meetings only) |

## Summary

Two to five sentences on what was discussed and why.

## Open action items from earlier meetings

| Item | Owner | State |
|---|---|---|

## Decisions

1. Decision, and the reason when it is not obvious.

## Review findings

Review meetings only.

| Finding | Disposition | Reason |
|---|---|---|

## Action items

| Action | Owner | Issue |
|---|---|---|

## Matrix status changes

| ID | From | To | Pull request |
|---|---|---|---|
```
