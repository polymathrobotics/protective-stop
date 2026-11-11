---
title: Meeting Procedure
sidebar_position: 1
---

<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Meeting Procedure

*Date: 2026-10-22.*

This procedure covers Protective Stop meetings whose outcome belongs in the safety case:
documentation syncs, design and requirements reviews, test plan reviews, and any meeting
that moves an item in the [Safety Certification Readiness](safety_certification_readiness.md)
matrix forward, whether or not its status changes.
Each meeting produces a short public record under Meeting Records in this section, merged
by pull request, so the record, the people who agreed to it and the date are all in Git
history. The [Documentation Kickoff](meetings/20260930_documentation_kickoff.md) record is
an example.

IEC 61508 asks for reviews to be recorded with their participants and results, and for actions
to be followed up to resolution (Part 1 §5.2.6, §6.2.5). This procedure is how we meet that.

## Roles

| Role | Responsibility |
|---|---|
| Organizer | Sends the invite, runs the meeting, and makes sure a record is written. |
| Note-taker | Writes the record and opens its pull request. Defaults to the organizer. |
| Required attendees | The people needed to make the meeting's decisions. They review the record. |
| Optional attendees | Anyone else. They may comment on the record but are not asked to approve it. |

## The meeting invite

Every invite carries:

1. **Title**: `PStop <type>: <topic>`, for example `PStop review: safety requirements`.
2. **Agenda**: the items to cover, each with a time box and the outcome it needs (a
   decision, a review outcome, or information only).
3. **Required and optional attendees**, marked as such. Required attendees are the people
   without whom an agenda decision can't be made.
4. **Related readiness items or IEC 61508 clauses**, when the meeting has them. Not every
   meeting maps to a clause. A general sync may not, and the invite says *None*.
5. **Pre-reads**, linked at a fixed commit, for example
   `docs/safety/safety_requirements_specification.md@<commit>`, so everyone reviews the same
   revision.
6. **Carried-over items**: agenda items not covered last time and action items due.
7. **Note-taker**.

## Quorum

The meeting has quorum when every required attendee, or a delegate they named, is present.

If quorum isn't met at the start, the organizer either:

- **reschedules**, preferably within a week, or
- **proceeds as a discussion only.** Anything decided is recorded as *provisional* and becomes
  final when the absent required attendees approve the record pull request.

The record lists absent required attendees either way.

## Opening the meeting

In the first two minutes:

1. Recap the agenda from the invite and add any late items.
2. Confirm who is attending and whether quorum is met.
3. Go through carried-over agenda items and action items that are due.

## Closing the meeting

In the last five minutes:

1. Read back each decision.
2. Read back each action item with its single owner and its due date: an ETA, or a DFD
   (date for a date) when the owner can't commit to an ETA yet.
3. Name any agenda item that wasn't covered. It carries over to the next meeting's agenda.
4. Note any readiness item the meeting moved forward, even if its status doesn't change.
5. Confirm who writes the record.

## After the meeting

Within two working days:

1. **File action items.** Open a GitHub issue for each action item, assigned to its owner,
   with its due date, and link the meeting record. Action items that only concern internal
   work outside this repository are tracked internally and listed in the record without
   detail.
2. **Write the record.** Copy the [template](#record-template) to
   `docs/process/meetings/YYYYMMDD_<topic>.md`, where `YYYYMMDD` is the meeting date, and
   fill it in. The date prefix keeps records in date order and easy to find.
3. **Open a pull request** with the record and request review from the required attendees.
   Don't request review from optional attendees, however many there are.
4. **Merge** once at least one required attendee other than the note-taker has approved
   and no required attendee has asked for changes. Required attendees who don't respond
   within two working days are taken to have no objection. The exception is provisional
   decisions (see [Quorum](#quorum)), which need the absent required attendees to approve.
5. **Update the readiness tracking.** A status change agreed in the meeting is made in a pull
   request that links its evidence. That can be the same pull request as the record, or a
   separate one linked from the record.

## What goes in the public record

Records are public. Write a summary, not a transcript.

| Include | Leave out |
|---|---|
| Attendee names and roles | Video, audio and full transcripts |
| The agenda and what was and wasn't covered | Customer names, sites and commercial terms |
| Decisions, with the reason when it is not obvious | Credentials, keys and internal hostnames |
| Action items with owners, due dates and issue links | Third-party proprietary material, such as an assessor's checklists |
| For reviews: the document and commit reviewed, the findings and how each was disposed of | Personnel matters and off-topic discussion |

## Review meetings

A meeting that reviews a work product, such as the safety requirements, the architecture or
a test plan, also records:

- The document and the commit reviewed.
- Who wrote the document. At least one reviewer must not be an author of it.
- Each finding, and whether it was accepted, rejected or deferred, with the reason.
- The outcome: approved, approved with changes, or not approved.

## Record template

```markdown
---
title: <Topic in Title Case>
---

<!-- SPDX-FileCopyrightText: <year> Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# <Topic in Title Case>

*Date: YYYY-MM-DD.*

| | |
|---|---|
| Type | Documentation sync / Review / Decision |
| Required attendees | Name (role), Name (role) |
| Optional attendees | Name (role) |
| Absent | Name (required or optional) |
| Quorum | Met / Not met: decisions below are provisional |
| Organizer | Name |
| Note-taker | Name |
| Related items | Readiness IDs or IEC 61508 clauses, or None |
| Document reviewed | path@commit (review meetings only) |

## Summary

Two to five sentences on what was discussed and why.

## Agenda

| Item | Outcome |
|---|---|
| Item from the invite | Covered / Partly covered / Not covered: carried to next meeting |

## Carried-over action items

| Action | Owner | Due | State |
|---|---|---|---|

## Decisions

1. Decision, and the reason when it is not obvious.

## Review findings

Review meetings only.

| Finding | Disposition | Reason |
|---|---|---|

## Action items

| Action | Owner | Due (ETA or DFD) | Issue |
|---|---|---|---|

## Readiness progress

| ID | Progress or status change | Pull request |
|---|---|---|
```
