---
title: Documentation Kickoff
---

<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Documentation Kickoff

*Date: 2026-09-30.*

| | |
|---|---|
| Type | Documentation sync |
| Required attendees | Raj Madhivanan (organizer, remote), David Tarazi, Ilia Baranov, Emerson |
| Optional attendees | None |
| Absent | None |
| Quorum | Met |
| Organizer | Raj Madhivanan |
| Note-taker | Raj Madhivanan (record written 2026-10-22 from the meeting transcript) |
| Related items | IEC 61508 process and documentation obligations in general |

## Summary

This was the first meeting of the documentation series. The team went through the external
gap-review checklist, which lists the process and documentation evidence needed for an
IEC 61508 certification, area by area. The aim was to choose which areas matter for day-to-day
development now, rather than working through the checklist from top to bottom. The team is
not seeking certification yet, so the goal is to build the safety case in the way the
project already works and to justify each choice to the assessor when asked.

## Agenda

The invite carried no agenda or pre-reads, so attendees met the checklist for the first time
in the meeting. The agenda below was stated at the start. Sending the agenda and pre-reads
with the invite is now part of the [Meeting Procedure](../meeting_procedure.md).

| Item | Outcome |
|---|---|
| Overview of the gap-review checklist and its areas | Covered |
| Choose the first areas to prioritize | Covered. See decisions 2 to 4. |
| Agree how these meetings are recorded and how often they run | Covered. See decisions 5 and 6. |
| Walk through the coding standard area in detail | Not covered. Carried to the next meeting. |

## Decisions

1. Use the checklist as a guide and prioritize the areas that support development. The
   tools are our choice, as long as we can explain how each one meets the requirement. For
   example, GitHub already records who made each change, when, and why.
2. The modification procedure, already in progress, is a good first area because the
   existing tools can implement it.
3. Publish the documentation as a static site with search, so it can be explored without
   reading the directory tree. Generate API reference from the header docstrings.
4. Track the checklist in source control as a page on the site. Keep an ID, status, summary
   requirement, action item and resolution for each entry. Leave out the assessor's
   compliance argument and evidence columns. Reference IEC 61508 clause numbers instead of
   the assessor's own IDs and groupings, which remain the assessor's property.
5. Record these meetings as public summaries in the repository, naming the attendees. Don't
   commit video or full transcripts.
6. Hold the documentation sync every two weeks. Attendance is optional for anyone not needed
   for that session.

Firmware release automation was also discussed. No decision was made, and reliability of
fielded units stays the higher priority.

## Action items

State as of 2026-10-22.

| Action | Owner | Due (ETA or DFD) | State |
|---|---|---|---|
| Publish the documentation site | David Tarazi | Done 2026-10-14 | Done in [#192](https://github.com/polymathrobotics/protective-stop/pull/192) |
| Generate API reference from header docstrings, and require docstrings on public interfaces | David Tarazi | Done 2026-10-15 | Done in [#201](https://github.com/polymathrobotics/protective-stop/pull/201) and [#202](https://github.com/polymathrobotics/protective-stop/pull/202) |
| Write the [Meeting Procedure](../meeting_procedure.md) | Raj Madhivanan | ETA: next documentation sync | In review in [#205](https://github.com/polymathrobotics/protective-stop/pull/205) |
| Put the checklist status on the site: David provides the page framework, Raj supplies the content and decides what is public | David Tarazi, Raj Madhivanan | ETA: next documentation sync | In review in [#206](https://github.com/polymathrobotics/protective-stop/pull/206) |
| Schedule the biweekly documentation sync | Raj Madhivanan | DFD: next documentation sync | Tracked internally |
| Apply the shared agent instructions to this repository, and consider a template for open-source repositories | To be confirmed | DFD: next documentation sync | Open. [`AGENTS.md`](/AGENTS.md) was added in [#194](https://github.com/polymathrobotics/protective-stop/pull/194) |

## Readiness progress

The checklist-to-site work (decision 4) is in review in
[#206](https://github.com/polymathrobotics/protective-stop/pull/206). No item's status changed
in this meeting.
