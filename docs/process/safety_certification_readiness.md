---
title: Safety Certification Readiness
sidebar_position: 2
---

<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Safety Certification Readiness

*Date: 2026-10-22.*

This matrix tracks the Protective Stop against the process and documentation obligations
of IEC 61508:2010 for the SIL 3 design target. It shows what is in place, what is in
review, and what is still open. It is a working tracker, not a certification claim. The
Protective Stop is not functional-safety certified.

The obligations came out of an external gap review held in 2026. The grouping, IDs and
wording are ours. Clause numbers point to IEC 61508:2010 so a reader who has the standard
can check the mapping. The standard's text is not reproduced here.

## How to read and update this page

| Status | Meaning |
|---|---|
| **Done** | Evidence is on `main` and linked in the Resolution column. |
| **In review** | Evidence exists in an open pull request, linked in the Resolution column. |
| **Partial** | Some evidence exists. The Action item column says what is missing. |
| **Not started** | No evidence yet. |
| **Not applicable** | The obligation doesn't apply. The Resolution column says why. |

A status changes only through a pull request that links the new evidence. When a
documentation meeting moves an item forward, its record links the pull request
that makes the change (see the [Meeting Procedure](meeting_procedure.md)).

Clause references are given as part and clause, for example *Part 3: 7.4.4.13* for
IEC 61508-3 clause 7.4.4.13. Letters such as *A.4.5* refer to the annex tables.

## Summary

| Area | Done | In review | Partial | Not started | Not applicable |
|---|---|---|---|---|---|
| [Functional Safety Management](#functional-safety-management) | 0 | 1 | 5 | 4 | 1 |
| [Documentation](#documentation) | 2 | 0 | 0 | 0 | 0 |
| [Configuration Management](#configuration-management) | 3 | 1 | 2 | 0 | 0 |
| [Safety Requirements](#safety-requirements) | 0 | 0 | 11 | 3 | 0 |
| [Architecture](#architecture) | 1 | 0 | 9 | 4 | 1 |
| [Detailed Design](#detailed-design) | 0 | 0 | 3 | 2 | 0 |
| [Coding Standard and Implementation](#coding-standard-and-implementation) | 1 | 0 | 6 | 1 | 0 |
| [Support Tools and Languages](#support-tools-and-languages) | 0 | 0 | 2 | 3 | 0 |
| [Verification](#verification) | 1 | 0 | 4 | 2 | 0 |
| [Integration Testing](#integration-testing) | 0 | 0 | 3 | 2 | 0 |
| [Validation](#validation) | 0 | 0 | 3 | 1 | 0 |
| [Modification](#modification) | 0 | 6 | 2 | 0 | 0 |
| [Safety Manual and User Documentation](#safety-manual-and-user-documentation) | 0 | 0 | 6 | 2 | 0 |
| [Traceability](#traceability) | 0 | 1 | 3 | 1 | 0 |
| **Total** | 8 | 9 | 59 | 25 | 2 |

## Functional Safety Management

Planning, roles, competence, follow-up of actions and field incidents, and assessment (Part 1 §6, Part 3 §6).

| ID | Obligation | IEC 61508 clauses | Status | Action item | Resolution |
|---|---|---|---|---|---|
| RM-MGT-01 | A functional safety plan defines the safety lifecycle, its activities and work products, and the strategy for achieving functional safety, and is communicated to the team. | Part 1: 5.2.2, 6.2.2, 6.2.4, 6.2.12.a, 6.2.12.c, 6.2.18; Part 2: 7.1.3.1, 7.1.3.2, 7.1.3.3, 7.4.6.2.c; Part 3: 6.2.2, 7.1.2.2 | **Partial** | Write a functional safety management plan under the Process tab that names the lifecycle phases and links the work product for each (Raj). | The safety work products exist under the Safety tab ([System Definition](../safety/system_definition.md), [Hazard Analysis and Risk Assessment](../safety/hazard_analysis_and_risk_assessment.md), [Safety Requirements Specification](../safety/safety_requirements_specification.md), [Requirements Traceability](../safety/requirements_traceability.md), [Open Items](../safety/open_items.md)). No single plan ties them together yet. |
| RM-MGT-02 | The software lifecycle phases are defined with the inputs and outputs of each phase. | Part 3: 7.1.2.3, 7.1.2.4, 7.1.2.5, 7.1.2.6 | **Not started** | Add the software phases with their inputs and outputs to the safety plan (Raj). | — |
| RM-MGT-03 | Key people and their project roles are named, and any division of responsibility between organizations is stated. | Part 1: 5.2.2, 6.2.3; Part 3: 6.2.2, 7.4.2.1, 7.4.5.1, 7.7.2.3 | **Partial** | Publish the role list with GitHub handles in the safety plan (Raj). | A personnel plan is kept internally. Maintainers with review authority are listed in [`CODEOWNERS`](/.github/CODEOWNERS). `pstop_c` is developed on its own certification track. |
| RM-MGT-04 | Competence requirements are defined per role, matched against the people filling each role, with training planned for gaps. | Part 1: 5.2.2, 6.2.13, 6.2.14.a, 6.2.14.b, 6.2.14.c, 6.2.14.d, 6.2.14.e, 6.2.14.f, 6.2.14.g, 6.2.14.h, 6.2.14.i, 6.2.14.j, 6.2.14.k, 6.2.15 | **Not started** | Create a competency matrix per role (Raj). | — |
| RM-MGT-05 | Action items and recommendations from reviews, meetings and analyses are tracked to resolution. | Part 1: 5.2.2, 6.2.5.c, 6.2.5.d, 6.2.5.e, 6.2.5.f; Part 2: 7.2.3.3.g; Part 3: 6.2.3.a | **In review** | Review open action items at each biweekly documentation meeting (Raj). | The [Meeting Procedure](meeting_procedure.md) requires every action item to be filed as a GitHub issue with an owner and reviewed at the next meeting, in review in [#205](https://github.com/polymathrobotics/protective-stop/pull/205). |
| RM-MGT-06 | Dangerous failures reported from the field are captured, analysed and used to prevent recurrence. | Part 1: 5.2.2, 6.2.5.f, 6.2.6 | **Partial** | Write a field failure reporting and corrective action procedure, including a template for the analysis (Raj). | A public reporting channel exists in the [Security](../developing/security.md) policy. There is no procedure for analysing reported dangerous failures yet. |
| RM-MGT-07 | A quality management system is in place and suppliers are qualified. | Part 1: 5.2.2, 6.2.17; Part 3: 6.2.2 | **Not started** | Decide how quality management and supplier qualification are argued for an open hardware product (Raj). | — |
| RM-MGT-08 | Every lifecycle phase has planned verification (criteria, techniques, tools) and its results are documented. | Part 1: 7.18.2.1, 7.18.2.2, 7.18.2.3, 7.18.2.4; Part 2: 7.1.3.3, 7.9.2.1, 7.9.2.2, 7.9.2.3, 7.9.2.4.a, 7.9.2.4.b, 7.9.2.4.c, 7.9.2.4.d, 7.9.2.6.a, 7.9.2.6.b, 7.9.2.6.c | **Not started** | Write a verification plan and keep phase verification results in one place (John, Ilia). | — |
| RM-MGT-09 | An independent functional safety assessment is planned, the assessor role is assigned and assessment activities are monitored. | Part 1: 5.2.2, 6.2.1, 6.2.7.a, 6.2.7.b, 6.2.7.c, 6.2.12.b, 6.2.16; Part 2: B.5.5; Part 3: 6.2.2, 8.2, A.10.6 | **Partial** | — | An external gap review against IEC 61508 was held in 2026. This matrix tracks the outcome. A formal assessment hasn't been commissioned. |
| RM-MGT-10 | The procedure for loading software into fielded units is defined and tested. | Part 3: 6.2.3.e | **Partial** | Write the field update procedure as a tested procedure with pass criteria. | First flash and over-the-air updates through the authenticated admin API are documented in the [Quickstart](../guides/quickstart.md) and [Secure Boot Guide](../guides/secure_boot_guide.md). |
| RM-MGT-11 | Accurate hazard event information is maintained for the protected process. | Part 1: 6.2.9 | **Not applicable** | — | This applies to the owner of the machine the Protective Stop is installed on. Integrator guidance belongs in the safety manual (RM-MAN-05). |

## Documentation

How safety documents are structured, controlled and published (Part 1 §5).

| ID | Obligation | IEC 61508 clauses | Status | Action item | Resolution |
|---|---|---|---|---|---|
| RM-DOC-01 | A document management system controls how safety documents are changed, reviewed and approved, and the latest revision can be identified. | Part 1: 5.2.11; Part 2: 7.1.3.4, 7.1.3.5 | **Done** | — | Safety documents live in this repository. Changes merge only by pull request with code-owner approval ([Contributing](../developing/contributing.md)). The site is published from `main`, so the published page is always the latest approved revision. |
| RM-DOC-02 | Safety documents have a title stating their scope, a table of contents, a revision history and are searchable. | Part 1: 5.2.7, 5.2.9, 5.2.10, 5.2.11; Part 2: 7.1.3.4, 7.1.3.5 | **Done** | — | Every page on this site has a title, an on-page table of contents and full-text search. Revision history is the Git history of each file, reachable from the edit link on every page. |

## Configuration Management

Identification, control and release of every configuration item (Part 1 §6.2.10, Part 3 §6.2.3).

| ID | Obligation | IEC 61508 clauses | Status | Action item | Resolution |
|---|---|---|---|---|---|
| RM-CFG-01 | The point at which formal configuration control starts is defined, and every configuration item is under that control. | Part 1: 6.2.10.a | **Done** | — | Firmware, host software, hardware design files, tests and safety documents are in one repository under pull-request review ([`CODEOWNERS`](/.github/CODEOWNERS)). The intended branch protection settings are written up in [#136](https://github.com/polymathrobotics/protective-stop/pull/136). |
| RM-CFG-02 | The configuration of each released product, including every hardware and software version, is uniquely identified. | Part 1: 6.2.10.b; Part 3: 6.2.3.a, 6.2.3.b, 6.2.3.c | **Done** | — | Releases are tagged and published as [GitHub Releases](https://github.com/polymathrobotics/protective-stop/releases). The safety traceability lint checks that cited requirement, code and test identifiers resolve. A wire-format signature check is in review in [#138](https://github.com/polymathrobotics/protective-stop/pull/138). |
| RM-CFG-03 | Each software release is documented in release notes that give the release number, the changes and the open issues. | Part 3: 6.2.3.f | **In review** | Adopt the release record template for the next release. | Releases are published with notes on [GitHub Releases](https://github.com/polymathrobotics/protective-stop/releases). A release record template that fixes the content is in review in [#136](https://github.com/polymathrobotics/protective-stop/pull/136). |
| RM-CFG-04 | Only approved items are delivered to users. | Part 1: 6.2.10.c; Part 3: 6.2.3.d | **Partial** | Decide when the change-control checks move from warn to enforce mode. | Merges require code-owner review and CI. `tools/release_guard.sh` rejects release binaries built with credentials ([Contributing](../developing/contributing.md)). A check that each pull request links an authorized change request is in review in [#139](https://github.com/polymathrobotics/protective-stop/pull/139), in warn mode. |
| RM-CFG-05 | Master copies of the software and its documentation are kept for the operational lifetime of each release. | Part 3: 6.2.3.g | **Partial** | Move the remaining internally held process documents (validation test plan, approved versions list, release notes template) into this repository. | The repository is the master copy. A migration map for documents moved out of internal tools is in review in [#136](https://github.com/polymathrobotics/protective-stop/pull/136). |
| RM-CFG-06 | Each released firmware image can be uniquely identified, for example by a cryptographic hash. | Part 3: 6.2.3.g | **Done** | — | Firmware builds are reproducible and checked in CI ([Contributing](../developing/contributing.md)). Releases ship a `SHA256SUMS` file ([Quickstart](../guides/quickstart.md)), and secure-boot profiles sign the bootloader and app ([Secure Boot Guide](../guides/secure_boot_guide.md)). |

## Safety Requirements

Content, structure and review of the system and software safety requirements (Part 1 §7.10, Part 2 §7.2, Part 3 §7.2).

| ID | Obligation | IEC 61508 clauses | Status | Action item | Resolution |
|---|---|---|---|---|---|
| RM-SRQ-01 | All safety functions needed to reach and hold the safe state are specified, including fault detection and safety communication. | Part 1: 7.10.2.5, 7.10.2.6.a; Part 2: 7.2.3.1, 7.2.3.2.a, 7.2.3.2.d; Part 3: 7.2.2.5.a, 7.2.2.10.a.1, 7.2.2.10.a.3, 7.2.2.10.a.7, 7.2.2.10.a.11 | **Partial** | Formal review under RM-SRQ-14. | The safety functions and 40 safety requirements are specified in the [Safety Requirements Specification](../safety/safety_requirements_specification.md). |
| RM-SRQ-02 | Software safety requirements are derived from the safety requirements, made available to developers and reviewed by them. | Part 1: 5.2.5; Part 2: 7.4.2.6; Part 3: 7.2.2.2, 7.2.2.5.a, 7.2.2.5.b, 7.2.2.5.c, 7.2.2.5.d, 7.2.2.5.e, 7.2.2.5.f, 7.2.2.6, 7.9.2.7.a | **Partial** | Design and document how the Protective Stop integrates with the machine's software stack (Raj). | Software allocations are recorded per requirement in the [Safety Requirements Specification](../safety/safety_requirements_specification.md) and [System Definition](../safety/system_definition.md). |
| RM-SRQ-03 | The target SIL, demand mode, target failure measure, hardware fault tolerance and any fault exclusions are specified and consistent with the FMEDA. | Part 1: 7.6.2.9, 7.10.2.2, 7.10.2.5, 7.10.2.6.f, 7.10.2.7.b; Part 2: 7.2.3.2.a, 7.2.3.3.a, 7.4.4.1.1.a, 7.4.4.1.1.b, 7.4.4.1.1.c, 7.4.9.4.m; Part 3: 7.2.2.10.b.1, 7.2.2.10.b.2 | **Partial** | Quantify the FMEDA (see Open Items). | SIL 3 is the allocated target. The [FMEDA](../safety/fmeda.md) isn't quantified yet ([Open Items](../safety/open_items.md)). |
| RM-SRQ-04 | Response time, diagnostic test interval and the constraints for high-demand operation are specified. | Part 1: 7.10.2.6.b, 7.10.2.6.g; Part 2: 7.2.3.2.c, 7.2.3.2.g, 7.4.4.1.4, 7.4.4.1.5; Part 3: 7.2.2.10.a.4, 7.2.2.10.a.9 | **Partial** | Reconcile the stop-on-silence timing in the requirements with the shipped defaults. | Heartbeat and stop-on-silence timing are specified in the [Safety Requirements Specification](../safety/safety_requirements_specification.md). |
| RM-SRQ-05 | Modes of operation and start-up and restart behaviour are specified. | Part 1: 7.10.2.6.f; Part 2: 7.2.3.2.g, 7.2.3.2.j; Part 3: 7.2.2.6 | **Partial** | — | Arming, re-arming and start-in-stopped behaviour are described in the [Failover and Arming Design](../design/failover_and_arming_design.md) and the [Safety Requirements Specification](../safety/safety_requirements_specification.md). |
| RM-SRQ-06 | System, operator and external data interfaces are specified, including how invalid, out-of-range or late data is handled. | Part 1: 7.10.2.6.c; Part 2: 7.2.3.2.e, 7.2.3.2.f; Part 3: 7.2.2.10.a.10, 7.2.2.12.a, 7.2.2.12.b, 7.2.2.12.c, 7.2.2.12.d, 7.2.2.12.e | **Partial** | — | Interfaces are documented in the [HTTP API](../guides/http_api.md) guide and the generated API reference. |
| RM-SRQ-07 | Reasonably foreseeable misuse through system and operator interfaces is handled safely. | Part 3: 7.2.2.5.f | **Not started** | Add misuse cases to the HARA and the requirements. | — |
| RM-SRQ-08 | Non-safety functions and their interfaces to safety functions are identified, with the safety boundary shown. | Part 2: 7.4.2.3, B.1.3; Part 3: 7.2.2.9, 7.2.2.10.a.8, 7.4.2.8 | **Partial** | — | Functions declared non-safety are marked in the [System Definition](../safety/system_definition.md). |
| RM-SRQ-09 | Requirements cover self-monitoring of program and data flow, diagnostics and proof testing. | Part 2: 7.2.3.3.b, 7.2.3.3.d, 7.4.9.4.g, A.15.1, A.16.5; Part 3: 7.2.2.8.a, 7.2.2.8.b, 7.2.2.8.c, 7.2.2.8.d, 7.2.2.8.e, 7.2.2.10.a.2, 7.2.2.10.a.5, 7.2.2.10.a.6 | **Partial** | — | Diagnostics are specified in the [Safety Requirements Specification](../safety/safety_requirements_specification.md) and analysed in the [FMEA](../safety/fmea.md). |
| RM-SRQ-10 | Hardware/software constraints, subsystem integration, capacity and other limiting conditions are specified. | Part 1: 7.10.2.7.g; Part 2: 7.2.3.2.b, 7.2.3.2.i, 7.4.9.4.f; Part 3: 7.2.2.5.e, 7.2.2.7, 7.2.2.10.a.9, 7.2.2.12.c | **Partial** | — | Capacity limits are documented in the multi-remote design pages under the Design tab. |
| RM-SRQ-11 | Operational parameters are protected against invalid values, unauthorized change and corruption. | Part 3: 7.2.2.13.a, 7.2.2.13.b, 7.2.2.13.c | **Partial** | — | Timing configuration is validated against a floor (`tools/test_config_floor.py`). Configuration changes need the authenticated admin API ([Secure Boot Guide](../guides/secure_boot_guide.md)). |
| RM-SRQ-12 | Environmental and electromagnetic limits are specified. | Part 1: 7.10.2.7.e, 7.10.2.7.f; Part 2: 7.2.3.3.e, 7.2.3.3.f, 7.4.9.4.e, A.16.3 | **Not started** | Specify the operating environment and EMC immunity levels. | — |
| RM-SRQ-13 | The specification is structured hierarchically, uses semi-formal notation where it helps, is tool-supported and detailed enough to reach the target SIL. | Part 1: 5.2.5; Part 2: 7.2.2.2.b, 7.2.3.1, 7.2.3.3.a, B.1.2, B.1.4, B.1.6, B.1.8; Part 3: 7.2.2.3, A.1.1.a, A.1.4 | **Partial** | — | Requirements derive from HARA safety goals with stable IDs. CI checks the structure with the safety traceability lint ([Requirements Traceability](../safety/requirements_traceability.md)). |
| RM-SRQ-14 | The safety requirements are reviewed in a recorded review, and review findings are tracked to resolution. | Part 1: 5.2.1, 5.2.2, 5.2.3, 5.2.4, 5.2.5, 5.2.6, 5.2.7, 7.10.2.4.a; Part 2: 7.2.2.2.a, 7.2.3.4, 7.4.2.7.a, 7.4.2.7.b, 7.4.2.7.c, 7.9.2.8.c, B.1.1, B.1.2, B.1.5, B.1.6, B.1.7, B.5.5; Part 3: 7.9.2.6.a, 7.9.2.7.a, 7.9.2.8.a, 7.9.2.8.b, 7.9.2.8.c, B.7 | **Not started** | Schedule the requirements review and record it under the [Meeting Procedure](meeting_procedure.md) (Raj). | — |

## Architecture

System and software architecture, fault behaviour, communication and architecture review (Part 2 §7.4, Part 3 §7.4.3).

| ID | Obligation | IEC 61508 clauses | Status | Action item | Resolution |
|---|---|---|---|---|---|
| RM-ARC-01 | The system is partitioned into subsystems with defined interfaces, and the important hardware/software interactions are described. | Part 2: 7.2.3.2.h, 7.2.3.3.a, 7.4.2.1, 7.4.2.10, 7.4.2.11, 7.4.6.2.a, 7.4.6.2.b, 7.4.9.4.m | **Partial** | — | Function decomposition is in the [System Definition](../safety/system_definition.md). Subsystem designs are under the Design tab. |
| RM-ARC-02 | The integrity of each component is identified, independence is shown where integrity differs, and on-chip redundancy meets Part 2 Annex E. | Part 2: 7.4.2.3, 7.4.2.5.a, 7.4.2.5.b, 7.4.3.2, E.1.a, E.1.b | **Partial** | Argue the Annex E requirements for the dual-core lockstep on a single ESP32-S3. | Diversity between the two cores is argued in [DU-3 Object Code Diversity](../safety/du_3_object_code_diversity.md). |
| RM-ARC-03 | The behaviour on a detected fault is to reach or hold the safe state. | Part 2: 7.2.3.2.g, 7.2.3.3.c, 7.4.2.2.d, 7.4.8.1.a, 7.4.8.1.b, 7.4.8.2.a, 7.4.8.2.b, 7.4.8.3 | **Partial** | Formal review under RM-ARC-10. | Detected faults stop transmission, and the machine stops on silence ([Safety Chain](../safety/safety_chain.md)). |
| RM-ARC-04 | Safety-critical communication interfaces are identified and analysed against Part 2 §7.4.11. | Part 1: 7.10.2.6.e; Part 2: 7.4.2.2.e, 7.4.11.1, 7.4.11.2, A.15.5, A.16.12; Part 3: 7.9.2.13.e | **Partial** | — | The transport is treated as an untrusted black channel ([FMEDA](../safety/fmeda.md)). Protocol safety improvements are in review in [#196](https://github.com/polymathrobotics/protective-stop/pull/196). |
| RM-ARC-05 | Configuration changes are protected against unauthorized modification. | Part 2: A.17.1 | **Done** | — | Configuration and firmware updates need the authenticated admin API, and deployed units run secure boot with flash encryption ([Secure Boot Guide](../guides/secure_boot_guide.md)). |
| RM-ARC-06 | Measures that prevent systematic faults in system design are applied, including semi-formal notation, design checklists, well-tried components and project management. | Part 2: 7.2.3.5, 7.4.2.2.c, 7.4.6.1, B.1.6, B.2.1, B.2.2, B.2.3, B.2.4, B.2.5, B.2.6, B.2.7, B.2.8, B.2.9, B.2.10, B.2.11, B.2.12, B.3.5, B.4.4 | **Partial** | Define a design review checklist. | Design pages use sequence and state diagrams. |
| RM-ARC-07 | Maintainability, testability, human factors and a limited number of operating modes are considered in the design. | Part 2: 7.2.3.3.b, 7.4.6.3, 7.4.7.1.c, 7.4.7.2, 7.4.7.3, B.4.2, B.4.3, B.4.6, B.4.7, B.4.8 | **Partial** | — | Operator interaction is limited to the stop switch and the arming gesture ([Failover and Arming Design](../design/failover_and_arming_design.md)). |
| RM-ARC-08 | Measures against environmental and systematic failures, on-line monitoring and the Part 2 Annex A/B techniques are applied, with any gaps justified. | Part 2: 7.4.2.8, 7.4.2.9, 7.4.7.1.a, 7.4.7.1.b, A.15.2, A.16.9, A.16.10, A.16.11, A.16.12, A.16.13, A.16.14, A.17.2, A.17.3, A.17.4 | **Not started** | Fill in the Part 2 Annex A/B technique tables. | — |
| RM-ARC-09 | Custom ICs (ASIC or user-programmable logic) in the design meet Part 2 Annex E. | Part 2: 7.4.2.2.b | **Not applicable** | — | The design uses no custom or user-programmable ICs. |
| RM-ARC-10 | The system architecture is reviewed in a recorded review that confirms it is testable and maintainable. | Part 1: 5.2.6; Part 2: 7.4.7.2, 7.9.2.8.a, 7.9.2.8.b, 7.9.2.8.c | **Not started** | Hold the architecture review and record it under the [Meeting Procedure](meeting_procedure.md) (Raj). | — |
| RM-ARC-11 | The software architecture describes each component, says whether it is new, reused or third-party (and how reused parts are qualified), its integrity, and every hardware/software interface. | Part 2: 7.4.2.3; Part 3: 7.2.2.4, 7.4.2.8, 7.4.2.9, 7.4.3.2.a, 7.4.3.2.b, 7.4.3.2.c, A.2.7 | **Partial** | — | `pstop_c` is vendored from its certification track. Networking uses the third-party `microlink` component. |
| RM-ARC-12 | The software architecture uses unambiguous, semi-formal notation (block, state, sequence and data-flow diagrams, decision tables). | Part 2: A.16.15; Part 3: 7.4.3.2.d, A.2.11.a, A.2.11.b, A.2.11.c, A.2.11.d, B.7.1, B.7.2, B.7.3, B.7.4.a, B.7.4.b, B.7.5, B.7.6, B.7.7, B.7.8 | **Partial** | Add the missing diagram types (John, Ilia). | Some design pages include Mermaid sequence and state diagrams. |
| RM-ARC-13 | The software architecture specifies fault detection, graceful degradation or retry, deterministic flow, a memory allocation strategy and fault control, backed by a software failure analysis. | Part 2: 7.4.7.1.d, A.15.1; Part 3: A.2.1, A.2.3.a, A.2.3.b, A.2.3.c, A.2.3.d, A.2.3.e, A.2.3.f, A.2.3.g, A.2.4.a, A.2.4.b, A.2.13.a, A.2.13.b, A.2.13.c, A.2.14, A.10.3, A.10.4, B.4.3 | **Partial** | — | Lockstep comparison, watchdogs and clock cross-checks are designed in ([Watchdog and Clock Cross-Check](../safety/watchdog_and_clock_cross_check.md)). Failure modes are analysed in the [FMEA](../safety/fmea.md). |
| RM-ARC-14 | The software architecture uses no artificial intelligence or dynamic reconfiguration. | Part 3: 7.9.2.9.c, A.2.5, A.2.6 | **Not started** | State this explicitly in the architecture. | — |
| RM-ARC-15 | The software architecture is reviewed against the safety requirements in a recorded review. | Part 1: 5.2.6, 6.2.5.c; Part 3: 7.4.2.3, 7.4.3.2.d, 7.9.2.6.a, 7.9.2.7.b, 7.9.2.9.a, 7.9.2.9.c, 7.9.2.9.d, 7.9.2.13.b | **Not started** | Hold the software architecture review and record it under the [Meeting Procedure](meeting_procedure.md) (Raj). | — |

## Detailed Design

Module-level design, diagnostics and design review (Part 3 §7.4.5).

| ID | Obligation | IEC 61508 clauses | Status | Action item | Resolution |
|---|---|---|---|---|---|
| RM-DES-01 | Each architecture component has a module-level design that follows a design standard and favours simplicity and modularity. | Part 3: 7.4.2.2.a, 7.4.2.2.b, 7.4.2.2.c, 7.4.2.2.d, 7.4.2.2.e, 7.4.2.6, 7.4.5.4, A.4.4, A.4.5 | **Partial** | Update the design pages to cover every component (Ilia, John). | Design pages and the generated API reference describe the main modules. |
| RM-DES-02 | The design describes how data integrity is maintained, how program flow is monitored, and the diagnostics for control flow, data flow and hardware faults. | Part 3: 7.4.2.7, 7.4.3.2.c, 7.4.3.2.e | **Partial** | — | See [Watchdog and Clock Cross-Check](../safety/watchdog_and_clock_cross_check.md) and the [FMEA](../safety/fmea.md). |
| RM-DES-03 | Design uses structured or semi-formal methods, unambiguous notation and computer-aided tools. | Part 3: 7.4.2.5, 7.4.5.4, A.2.12, A.4.1.a, A.4.1.b, A.4.1.c, A.4.2, B.7 | **Partial** | Add the diagrams that don't exist today (John). | — |
| RM-DES-04 | The inputs to the detailed design phase are defined. | Part 3: 7.4.5.2 | **Not started** | Add to the safety plan (Raj). | — |
| RM-DES-05 | Detailed design is reviewed with a checklist covering clarity, modularity, testability, maintainability, coverage of the architecture, data structures and interface failures. | Part 1: 5.2.6; Part 3: 7.4.2.4, 7.4.5.3, 7.4.5.4, 7.9.2.6.a, 7.9.2.6.b, 7.9.2.7.c, 7.9.2.7.e, 7.9.2.10.a, 7.9.2.10.c, 7.9.2.10.d, 7.9.2.11.a, 7.9.2.11.c, 7.9.2.11.d, 7.9.2.13.a, 7.9.2.13.b, 7.9.2.13.d, B.8.8 | **Not started** | Write the review checklist and hold the review under the [Meeting Procedure](meeting_procedure.md) (Raj). | — |

## Coding Standard and Implementation

Coding rules, language subset and implementation practices (Part 3 §7.4.4, §7.4.6).

| ID | Obligation | IEC 61508 clauses | Status | Action item | Resolution |
|---|---|---|---|---|---|
| RM-COD-01 | A coding standard exists for each language, promotes good practice, readability and testability, and is followed. | Part 3: 7.4.4.11, 7.4.4.12, 7.4.4.13, A.4.5, B.1.1 | **Partial** | Confirm the safety code follows the standard and publish the written standard (John). | The Polymath code standard runs as pre-commit hooks in CI ([Contributing](../developing/contributing.md)). The [Commenting Rules](../developing/commenting_rules.md) apply to all new code. |
| RM-COD-02 | A language subset is defined that excludes undefined, unsafe and error-prone features, implicit conversions and unstructured control flow, and is checked automatically. | Part 3: 7.4.4.10.b, 7.4.4.11, 7.4.4.13, A.3.2, A.3.3, A.4.6, B.1.7, B.1.8, B.8.3 | **Partial** | Make the CI MISRA step fail on findings not in the deviation register. | MISRA C:2012 is the subset. `tools/misra_check.sh` runs in CI as an advisory step that doesn't fail the build. Deviations are recorded individually in the [MISRA C Compliance Review](../safety/misra_c_compliance_review.md). |
| RM-COD-03 | Source code documentation covers the legal entity, module and function descriptions, inputs and outputs, and version history. | Part 3: 7.4.4.13, 7.4.4.13.a, 7.4.4.13.b, 7.4.4.13.c, 7.4.4.13.d | **Done** | — | SPDX headers are enforced by pre-commit. Public interfaces carry Doxygen docstrings and Doxygen warnings fail the docs build ([Commenting Rules](../developing/commenting_rules.md)). Version history is in Git. |
| RM-COD-04 | Defensive programming is applied. | Part 3: A.4.3 | **Partial** | Write the defensive-programming invariants into the coding standard. | The timing configuration envelope is checked by `tools/test_config_floor.py`. |
| RM-COD-05 | Dynamic memory, interrupts, pointers and recursion are restricted by policy, and the policy is followed. | Part 3: B.1.2, B.1.3.a, B.1.3.b, B.1.4, B.1.5, B.1.6 | **Partial** | Verify and state the policy (Raj, John). | MISRA rules cover part of this ([MISRA C Compliance Review](../safety/misra_c_compliance_review.md)). |
| RM-COD-06 | Module size, complexity and parameter count are limited, and functions have one entry and one exit. | Part 2: 7.4.6.1; Part 3: A.4.4, B.9.1, B.9.2, B.9.4, B.9.5 | **Not started** | Decide on limits and how single exit is interpreted, then measure them in CI. | — |
| RM-COD-07 | Modules hide their internals and interact only through fully defined, documented interfaces. | Part 3: 7.4.2.4, B.9.3, B.9.6 | **Partial** | — | Public headers are documented and published as the API reference. |
| RM-COD-08 | Trusted or verified software components are used where available. | Part 3: A.2.8, A.4.7 | **Partial** | — | The protocol and machine-safety logic come from `pstop_c`, which follows its own certification track. |

## Support Tools and Languages

Inventory, classification and qualification of tools and languages (Part 3 §7.4.4).

| ID | Obligation | IEC 61508 clauses | Status | Action item | Resolution |
|---|---|---|---|---|---|
| RM-TLS-01 | Every off-line support tool is listed with its vendor, version and use, and classified T1, T2 or T3. | Part 3: 7.4.4.2, 7.4.4.4, 7.4.4.15.a, 7.7.2.8 | **Not started** | List and classify the tools: hosting, CI, compilers, flashing and test scripts (Raj). | — |
| RM-TLS-02 | Each T2 and T3 tool has a manual or specification, an assessment of how its failures could affect the software, recorded configuration and evidence it behaves as specified. | Part 2: T3; Part 3: 7.4.4.4, 7.4.4.5, 7.4.4.6, 7.4.4.7, 7.4.4.8, 7.4.4.15.b, 7.4.4.15.c, A.3.4.a, A.3.4.b | **Not started** | Follows RM-TLS-01. | — |
| RM-TLS-03 | Tool selection, justification, compatibility, upgrades and configuration management are defined, and someone is responsible for tool qualification. | Part 3: 7.4.3.1, 7.4.4.2, 7.4.4.3, 7.4.4.9, 7.4.4.16, 7.4.4.17, 7.4.4.18, 7.4.4.18.a, 7.4.4.18.b, 7.4.4.19 | **Partial** | — | The ESP-IDF version and component lock file are pinned, and builds are reproducible in CI ([Contributing](../developing/contributing.md)). |
| RM-TLS-04 | On-line tools that can affect the running system are developed to IEC 61508. | Part 3: 7.4.4.1 | **Not started** | Determine whether any on-line tools exist. | — |
| RM-TLS-05 | The programming language and translators are suitable: compilers are treated as T3, and the language supports modularity and automatic error detection. | Part 3: 7.4.4.10.a, 7.4.4.10.c, 7.4.4.10.d, 7.4.4.10.e, 7.4.4.11, 7.4.4.14, A.3.1, A.3.2, A.9.3, B.8.3, B.8.4 | **Partial** | — | Safety code is C with the MISRA subset and static analysis ([MISRA C Compliance Review](../safety/misra_c_compliance_review.md)). |

## Verification

Verification planning, static analysis, code inspection, module testing and coverage (Part 1 §7.18, Part 3 §7.9).

| ID | Obligation | IEC 61508 clauses | Status | Action item | Resolution |
|---|---|---|---|---|---|
| RM-VER-01 | A software verification plan covers module testing, integration testing, design verification and validation, with tools and criteria. | Part 3: 7.9.2.1, 7.9.2.2.a, 7.9.2.2.b, 7.9.2.2.c, 7.9.2.2.d, 7.9.2.2.e, 7.9.2.7.c, 7.9.2.7.d | **Not started** | Write the verification plan (Raj). | — |
| RM-VER-02 | Software is verified against the plan, non-conformances are resolved, and records are archived. | Part 3: 7.9.2.3, 7.9.2.4, 7.9.2.5 | **Not started** | Follows RM-VER-01. | — |
| RM-VER-03 | Static analysis results are recorded. | Part 3: 7.9.2.12, A.9.3, B.8.4 | **Done** | Confirm current results (John). | See the [MISRA C Compliance Review](../safety/misra_c_compliance_review.md). |
| RM-VER-04 | Every safety-related module is inspected, with findings recorded and followed up. | Part 1: 5.2.6; Part 3: 7.4.6.1, 7.9.2.4, 7.9.2.5.a, 7.9.2.5.b, 7.9.2.5.c, 7.9.2.12, A.9.3, B.8.3, B.8.6.a, B.8.6.b | **Partial** | Hold a code inspection of the safety modules (Raj). | Every change is reviewed by a code owner in a pull request. There's no inspection checklist record yet. |
| RM-VER-05 | Module tests are specified, reviewed and run for every safety-related module using boundary-value and equivalence-class analysis, with results documented and failures corrected. | Part 2: 7.9.2.5; Part 3: 7.4.5.4, 7.4.7.1, 7.4.7.2, 7.4.7.3, 7.4.7.4, 7.9.2.6.b, 7.9.2.7.f, 7.9.2.7.h, 7.9.2.11.b, A.5.2, A.5.3, A.5.4, A.5.7, A.9.4, B.2.1, B.2.6, B.3.4, B.8.1 | **Partial** | — | Host and `pstop_c` requirement tests run in CI ([Testing](../developing/testing/testing.md)). |
| RM-VER-06 | Structural coverage (entry points, statements, branches) is measured. | Part 3: B.2.7.a, B.2.7.b, B.2.7.c | **Partial** | Confirm coverage targets (John). | Coverage is measured in CI and reported in the [Coverage Report](../safety/coverage_report.md). |
| RM-VER-07 | Test management and automation tools are used. Model-based testing and numerical analysis are applied where relevant. | Part 3: A.5.6, A.5.8, A.9.7, B.2.4, B.3.2 | **Partial** | — | Tests are automated in GitHub Actions and the [HIL](../developing/testing/hil.md) harness. |

## Integration Testing

Integration test planning, cases and results (Part 2 §7.5, Part 3 §7.4.8, §7.5).

| ID | Obligation | IEC 61508 clauses | Status | Action item | Resolution |
|---|---|---|---|---|---|
| RM-INT-01 | An integration test plan covers hardware and software, integration sets, environment, pass/fail criteria, corrective action and any testing at a user's site. | Part 2: 7.4.2.11, 7.4.6.5.b, 7.4.6.5.c, 7.4.6.6, 7.5.2.1, 7.5.2.3, 7.5.2.6.b, 7.5.2.7, B.3.2, B.3.3, B.3.5; Part 3: 7.4.3.2.f, 7.4.5.5, 7.4.8.1, 7.4.8.2.a, 7.4.8.2.d, 7.4.8.2.e, 7.4.8.2.f, 7.5.2.1, 7.5.2.2.a, 7.5.2.2.c, 7.5.2.2.d, 7.5.2.2.e, 7.5.2.3, 7.5.2.4.a, 7.5.2.5 | **Not started** | Create the integration test plan (Raj). | — |
| RM-INT-02 | Integration test cases test the safety functions using functional, black-box, performance, stress, boundary-value, equivalence-class and interface testing. | Part 2: 7.4.6.5.a, 7.5.2.7, 7.9.2.10, B.3.1, B.3.4, B.5.1; Part 3: 7.4.8.2.b, 7.4.8.2.c, 7.5.2.2.b, 7.5.2.2.c, 7.5.2.7, 7.9.2.7.g, 7.9.2.14, A.5.2, A.5.4, A.5.5, A.5.6, A.5.7, A.6.1, A.6.2, B.2.1, B.2.4, B.2.6, B.3.2, B.3.4, B.6.1, B.6.2, B.6.3 | **Partial** | Fold the existing ladders into the integration test plan (Raj). | Chaos, netem, soak and multi-remote ladders exist ([Testing](../developing/testing/testing.md)). |
| RM-INT-03 | Automated test tools are used for integration testing. | Part 2: 7.4.6.4; Part 3: A.5.8 | **Partial** | — | See the [HIL](../developing/testing/hil.md) harness and the test ladders. |
| RM-INT-04 | The integration test plan is reviewed in a recorded review. | Part 1: 5.2.6; Part 2: 7.9.2.7.b, 7.9.2.8.a, 7.9.2.8.c, 7.9.2.9; Part 3: 7.4.5.5, 7.4.8.2.c, 7.9.2.6.b, 7.9.2.9.b, 7.9.2.10.b | **Not started** | Review the plan once it exists, under the [Meeting Procedure](meeting_procedure.md) (Raj). | — |
| RM-INT-05 | Integration results record the test case, versions, tools, pass/fail outcome and the analysis of any failure or modification. | Part 2: 7.5.2.1, 7.5.2.2, 7.5.2.3, 7.5.2.4, 7.5.2.5, 7.5.2.6.a, 7.5.2.6.b, 7.5.2.6.c, 7.5.2.6.d, 7.5.2.6.e, 7.5.2.6.f, 7.5.2.6.g, 7.5.2.7, 7.9.2.5, 7.9.2.9, 7.9.2.10, B.3.3; Part 3: 7.4.8.3, 7.4.8.4, 7.4.8.5, 7.5.2.5, 7.5.2.6, 7.5.2.8, 7.9.2.7.i, 7.9.2.7.j, A.5.3 | **Partial** | Record results against the plan with date and owner. | Soak, failover and validation reports are under Developing, then Testing. |

## Validation

Validation planning, techniques, type testing and results (Part 2 §7.3, §7.7, Part 3 §7.3, §7.7).

| ID | Obligation | IEC 61508 clauses | Status | Action item | Resolution |
|---|---|---|---|---|---|
| RM-VAL-01 | A validation plan says how each safety requirement is validated, with pass/fail criteria, when and by whom, the technical strategy, and the test environment and tools. | Part 2: 7.3.2.1, 7.3.2.2.a, 7.3.2.2.b, 7.3.2.2.c, 7.3.2.2.d, 7.3.2.2.e, 7.3.2.2.f, 7.3.2.2.g, 7.4.6.4, 7.7.2.2, 7.7.2.3, 7.7.2.4.c, 7.9.2.4.a, 7.9.2.4.b, B.5.1, B.5.5, B.5.6, B.5.7, B.5.9, B.5.10, B.5.14, B.5.16; Part 3: 7.3.2.1, 7.3.2.2.a, 7.3.2.2.b, 7.3.2.2.e, 7.3.2.2.f, 7.3.2.2.g, 7.3.2.2.h, 7.3.2.2.i, 7.3.2.3.a, 7.3.2.3.b, 7.3.2.3.c, 7.3.2.3.d, 7.3.2.5.a, 7.3.2.5.b, 7.3.2.5.c, 7.7.2.7.a, 7.9.2.7.k | **Partial** | Finish the validation test plan, including hardware validation, and move it into this repository (Raj). | A validation test plan is drafted internally. |
| RM-VAL-02 | Validation uses expanded functional and worst-case testing, black-box testing, fault insertion, process simulation and modelling where appropriate. | Part 2: B.5.1, B.5.4, B.5.7, B.5.8, B.5.9, B.5.11, B.5.12, B.5.13, B.5.15; Part 3: 7.7.2.7.b1, 7.7.2.7.b2, A.7.2, A.7.3, A.7.4, B.2.6, B.5.2.a, B.5.2.c, B.5.3 | **Partial** | — | HIL tests cover button, discordance and power cycle ([HIL](../developing/testing/hil.md)). Fault-injection gaps are tracked in [Open Items](../safety/open_items.md). |
| RM-VAL-03 | The hardware is type-tested for temperature, humidity, shock, vibration, ingress, surge and EMC immunity. | Part 2: 7.3.2.2.f, A.16.3, A.16.4, B.5.2, B.5.3 | **Not started** | Plan the type tests once RM-SRQ-12 sets the limits. | — |
| RM-VAL-04 | Validation results record the test case, plan version, safety function, equipment and calibration, configuration under test and any failures with their change requests, and name any outside labs. | Part 1: 7.14.2.3; Part 2: 7.4.9.3.e, 7.7.2.1, 7.7.2.2, 7.7.2.3, 7.7.2.4.c, 7.7.2.5.a, 7.7.2.5.b, 7.9.2.5, 7.9.2.10, B.5.6; Part 3: 7.7.2.2, 7.7.2.4, 7.7.2.5.a, 7.7.2.5.b, 7.7.2.5.c, 7.7.2.5.d, 7.7.2.5.e, 7.7.2.5.f, 7.7.2.6, 7.7.2.9.b, 7.7.2.9.c | **Partial** | Record results against the validation plan. | Existing test reports give dates and versions but aren't tied to a plan. |

## Modification

Change requests, impact analysis, re-verification and user notification (Part 1 §7.16, Part 2 §7.8, Part 3 §7.8).

| ID | Obligation | IEC 61508 clauses | Status | Action item | Resolution |
|---|---|---|---|---|---|
| RM-MOD-01 | A modification procedure defines how change requests are raised and authorized, using a change request system. | Part 1: 7.16.2.1, 7.16.2.5; Part 2: 7.8.2.2; Part 3: 6.2.3.d, 7.8.2.1, A.8.6 | **In review** | Merge the procedure. | The procedure, with GitHub Issues as the change request system, is in review in [#136](https://github.com/polymathrobotics/protective-stop/pull/136). |
| RM-MOD-02 | Every modification has a documented impact analysis that names the affected modules and the earlier lifecycle phase to return to. | Part 1: 5.2.2, 7.16.2.3, 7.16.2.6; Part 2: 7.8.2.1.b; Part 3: 6.2.2, 6.2.3.d, 7.1.2.9, 7.8.2.3.a, 7.8.2.3.b, 7.8.2.4, 7.8.2.5, A.8.1, A.8.6 | **In review** | — | An impact analysis template and return-to-phase rule are in review in [#136](https://github.com/polymathrobotics/protective-stop/pull/136). |
| RM-MOD-03 | Each change record holds the reason, description, status, dates, staff and any deviations, and links to its impact analysis. | Part 1: 7.16.2.2; Part 2: 7.8.2.1.a, 7.8.2.1.d, 7.8.2.1.g; Part 3: 7.8.2.2.b, 7.8.2.2.c, 7.8.2.6.a, 7.8.2.6.b, 7.8.2.8.a, 7.8.2.8.b, 7.8.2.8.d, A.8.6 | **In review** | Process the first change end to end through the procedure (Raj). | A change request issue form with mandatory fields is in review in [#136](https://github.com/polymathrobotics/protective-stop/pull/136). No change has been processed through it yet. |
| RM-MOD-04 | Changes are reviewed and approved before release, affected documentation is updated in the same change, and revision history is kept. | Part 1: 7.16.2.7; Part 2: 7.8.2.1.c, 7.8.2.1.f, 7.8.2.1.i; Part 3: 7.8.2.8.c, 7.8.2.8.e, A.8.5, A.8.6 | **Partial** | — | Code-owner review is required on `main` ([`CODEOWNERS`](/.github/CODEOWNERS)). The same-change documentation rule is in review in [#136](https://github.com/polymathrobotics/protective-stop/pull/136). |
| RM-MOD-05 | Re-verification covers the changed module and every affected module, with the tests to run named per change. Regression and full revalidation are each used where they're permitted. | Part 1: 7.16.2.7, 7.18.2.1, 7.18.2.2, 7.18.2.3; Part 2: 7.8.2.1.e, 7.8.2.4; Part 3: 7.8.2.6.c, 7.8.2.6.d, A.8.2, A.8.3, A.8.4.a, A.8.4.b, A.8.6 | **In review** | List the build acceptance and release qualification suites as named tests with pass criteria (John). | Change classes and verification gates are defined in [#136](https://github.com/polymathrobotics/protective-stop/pull/136). |
| RM-MOD-06 | Re-verification and re-validation results for each modification are documented and analysed. | Part 1: 7.18.2.4; Part 3: 7.8.2.9 | **In review** | — | Result content and location are defined in [#136](https://github.com/polymathrobotics/protective-stop/pull/136). |
| RM-MOD-07 | Modifications are carried out with the same rigour as the original development. | Part 1: 7.16.2.4; Part 2: 7.8.2.3; Part 3: 7.8.2.5, 7.8.2.6, 7.8.2.7 | **In review** | — | Defined in [#136](https://github.com/polymathrobotics/protective-stop/pull/136). |
| RM-MOD-08 | Users are notified when a defect affecting safety is found in a released baseline. | Part 2: 7.8.2.2 | **Partial** | Create a deployment register and write the user notification procedure (Raj). | Advisories are published through the [Security](../developing/security.md) channel. The notification steps are in review in [#136](https://github.com/polymathrobotics/protective-stop/pull/136). |

## Safety Manual and User Documentation

Information an integrator needs to use the element safely (Part 2 Annex D, Part 3 Annex D).

| ID | Obligation | IEC 61508 clauses | Status | Action item | Resolution |
|---|---|---|---|---|---|
| RM-MAN-01 | A safety manual describes the functions, input and output interfaces, effect of internal faults, safe state and the exact hardware and software configuration, with recommended settings. | Part 2: 7.4.9.3.a, 7.4.9.6, D.2.1, D.2.1.a, D.2.1.b, D.2.2.g, D.2.2.i, D.2.4; Part 3: 7.4.2.12.b, D.2.3.a, D.2.3.b, D.2.4.k, D.2.4.n | **Not started** | Write the safety manual. | — |
| RM-MAN-02 | The safety manual gives failure rates, safe failure fraction, element type, hardware fault tolerance, systematic capability, diagnostic test interval, useful lifetime, environmental limits and any assumptions. | Part 2: 7.2.3.2.d, 7.4.5.3, 7.4.5.4, 7.4.9.3.c, 7.4.9.4.e, 7.4.9.4.f, 7.4.9.4.i, 7.4.9.4.k, 7.4.9.4.l, 7.4.9.4.m, 7.4.9.6, 7.4.9.7, 7.6.2.3, D.2.1.c, D.2.2.a, D.2.2.b, D.2.2.c, D.2.2.d, D.2.2.e, D.2.2.f, D.2.2.j, D.2.2.k, D.2.3.a; Part 3: D.2.3.c | **Not started** | Depends on the quantified FMEDA (RM-SRQ-03). | — |
| RM-MAN-03 | Proof testing and maintenance are described, including what to do after a failure and how to report it. | Part 2: 7.4.9.4.g, 7.4.9.6, 7.6.2.1.a, 7.6.2.1.c, 7.6.2.1.e, 7.6.2.1.g, D.2.1.c, D.2.2.h | **Partial** | — | Recovery and troubleshooting are documented in the [Recovery Playbook](../guides/recovery_playbook.md). |
| RM-MAN-04 | Security measures against threats and vulnerabilities are stated. | Part 1: 7.4.2.3; Part 2: 7.4.9.6; Part 3: D.2.4.m | **Partial** | Summarise these in the safety manual. | See the [Secure Boot Guide](../guides/secure_boot_guide.md) and [Security](../developing/security.md) policy. |
| RM-MAN-05 | Integrators get installation instructions, the competence expected of them, instructions and constraints on use, interface constraints and installation hazards. | Part 2: 7.4.9.3.b, 7.6.2.1.b, D.2.1.c; Part 3: D.2.2, D.2.4.a, D.2.4.c, D.2.4.l | **Partial** | — | Installation is covered by the [Quickstart](../guides/quickstart.md) and [Host](../guides/host.md) guides. |
| RM-MAN-06 | Release notes give the reason for each release, outstanding anomalies and compatibility, and integrators know how to raise a change request. | Part 3: 7.7.2.7.c, D.2.4.d, D.2.4.e, D.2.4.f, D.2.4.g, D.2.4.i, D.2.4.j | **Partial** | — | Releases are published on [GitHub Releases](https://github.com/polymathrobotics/protective-stop/releases). A change request form for integrators is in review in [#136](https://github.com/polymathrobotics/protective-stop/pull/136). |
| RM-MAN-07 | Operation and maintenance instructions consider usability, limited operating modes and protection against operator error, use graphics and text, and are reviewed. | Part 2: B.4.1, B.4.2, B.4.3, B.4.4, B.4.5, B.4.6, B.4.7, B.4.8 | **Partial** | — | Guides on this site cover operation. |
| RM-MAN-08 | The degree of reliance an integrator may place on the element is stated, and the evidence behind safety manual claims is available. | Part 3: D.2.4.b, D.3.3 | **Partial** | — | The safety case is published under the Safety tab. |

## Traceability

Forward and backward traceability between requirements, design, code, tests and changes (Part 3 Annex A).

| ID | Obligation | IEC 61508 clauses | Status | Action item | Resolution |
|---|---|---|---|---|---|
| RM-TRC-01 | Every safety requirement traces to at least one validation test, and every validation test traces back to a requirement. | Part 1: 5.2.5; Part 2: 7.3.2.2.a, 7.7.2.4.b; Part 3: A.7.5, A.7.6 | **Partial** | Close the open test gaps listed in the traceability matrix. | Bidirectional traceability is maintained in [Requirements Traceability](../safety/requirements_traceability.md) and checked in CI. CI checks that citations resolve, not that tests pass. |
| RM-TRC-02 | Safety requirements trace forward to software requirements, architecture and design, and back again. | Part 1: 5.2.5; Part 3: 7.9.2.9.a, A.1.2, A.2.9, A.2.10, A.4.8 | **Partial** | — | Requirements trace to functions and code locations in [Requirements Traceability](../safety/requirements_traceability.md). |
| RM-TRC-03 | Design traces to module tests, integration tests and code reviews, and back again. | Part 1: 5.2.5; Part 3: A.5.9, A.6.3, A.9.5, A.9.6 | **Not started** | Follows RM-INT-01 and RM-VER-04. | — |
| RM-TRC-04 | Each modification traces to the safety requirements it affects, and back again. | Part 1: 5.2.5; Part 3: A.8.7, A.8.8 | **In review** | — | The impact analysis cites requirement IDs, and a check that they resolve is in review in [#136](https://github.com/polymathrobotics/protective-stop/pull/136). |
| RM-TRC-05 | Safety requirements trace back to the hazards and needs they come from. | Part 1: 5.2.5, 7.10.2.1; Part 2: 7.2.2.1, 7.2.2.2.c, 7.4.2.1; Part 3: A.1.3 | **Partial** | — | Requirements derive from the safety goals in the [Hazard Analysis and Risk Assessment](../safety/hazard_analysis_and_risk_assessment.md). |
