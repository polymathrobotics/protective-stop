# MOD — Modification Procedure

> **Warning:** This procedure is **written but not yet exercised**. No change has been processed through it as of the date below. Treat every claim here as a statement of intent until the first completed change record exists.

# 1. Purpose and scope

This procedure defines how a change to the Protective Stop (PSTOP) is requested, assessed, authorized, implemented, re-verified and released, so that functional safety is preserved across the life of the product.

**Integrity target: SIL 3 (IEC 61508) with an equivalent PL e (ISO 13849) track.** The obligations in section 7 are set to the SIL 3 level.

It applies to every change, after first release, to:

- The `pstop_c` protocol and machine-safety library
- Remote firmware, host machine wrapper, components, tools and scripts
- Hardware: schematics, PCB layout, BOM, mechanical parts, enclosure CAD
- The safety requirements, HARA, architecture, design, FMEDA, test plans, traceability matrix and safety manual
- Build toolchain, compiler flags, MISRA configuration and CI definition
- Third-party or open source components integrated into PSTOP

It does not apply to changes made before the baseline of the first released version, which are governed by the development process, nor to documentation with no bearing on the safety argument.

Changes to protocol or safety behaviour belong upstream in `pstop_c` and are never made from the shell repository. A `pstop_c` bump that changes the CRC is a wire break and is always Class C.

# 2. Definitions

| Term | Meaning |
|---|---|
| Change Request (CR) | A GitHub Issue carrying the `change-request` label. The single entry point for all changes in scope. |
| Impact Analysis (IA) | The recorded assessment of what a proposed change affects, and what must be re-done as a result. |
| Change Record | The completed CR issue plus its linked IA, pull requests, test evidence and approvals. Covers one change. |
| Release Record | A GitHub Release plus its linked Gate 2 evidence and approval. Covers every change since the previous release. |
| BAT | Build Acceptance Test. A fast smoke suite, Gate 0. Fails the build early so that no expensive testing is wasted on a broken candidate. It is a filter, not a qualification. |
| Safety module | Any module that implements or can influence a safety requirement. Includes all of `pstop_c`, the lockstep comparator, the arming policy, and the heartbeat and timeout paths. |
| Baseline | A released, tagged configuration of hardware and software, recorded as a GitHub Release and in Approved Versions For Deployment. |

# 3. Roles

| Role | Held by | Responsibility |
|---|---|---|
| Requester | Anyone, including external contributors | Raises the CR with reason, description and affected hazards. |
| Impact Analyst | Assigned per CR, not the sole implementer | Completes the Impact Analysis and proposes the change class and re-verification scope. |
| Authorizer | `@iliabaranov`, `@rajasimman-madhivanan`, or `@davidt315` | Approves or rejects on the basis of the IA. Two distinct Authorizers are required for Class C. Records the decision on the CR. |
| Implementer | Assigned per CR | Makes the change by pull request. |
| Reviewer | A competent person other than the Implementer | Reviews against the coding standard and the IA before merge. |
| Release Approver | `@iliabaranov`, `@rajasimman-madhivanan`, or `@davidt315` | Confirms Gate 2 evidence is complete and passing before a baseline is tagged and published. |

The Authorizer for a given CR is not also its Implementer. Where team size makes this impossible, the deviation is recorded on the CR with a justification. This team-size deviation is retained but is currently not applicable because three Authorizers are available. The competency of the staff assigned to a modification is identified on the CR.

# 4. When this procedure is triggered

A CR is raised whenever any of the following occurs:

1. A defect is found in a released baseline, whether by the team, a user, or in the field.
2. A new or amended safety requirement is proposed.
3. A change to hardware, software, toolchain or third-party component is proposed for a released baseline.
4. A corrective action from a review, audit or incident requires a product change.
5. A dependency of PSTOP publishes a version change that the team intends to adopt.
6. A configuration default that affects safety behaviour is changed, including heartbeat interval, missed-heartbeat count, minimum stop hold, or the operator allowlist policy.

Work that bypasses this procedure is not merged to `main`.

# 5. Change classification

The Impact Analyst proposes a class; the Authorizer confirms it. The class determines the Gate 1 scope in section 7.

| Class | Definition | Approval |
|---|---|---|
| A — No safety impact | Comments, formatting, non-shipped tooling, documentation with no bearing on the safety argument. No change to executable behaviour or hardware. | One Authorizer |
| B — Indirect safety impact | Change to a non-safety module, or to a safety module with no change to its interface, timing, state machine, or the requirements it implements. | One Authorizer |
| C — Direct safety impact | Change to a safety requirement, a safety module interface, the wire protocol or CRC, a timing budget, the bond or arming state machine, the lockstep comparator, diagnostic coverage, or any hardware in the stop path. | Two distinct Authorizers |

When the class is uncertain, the higher class applies.

# 6. The procedure

## Step 1 — Raise the Change Request

The Requester opens a GitHub Issue using the [Change Request issue form](../../.github/ISSUE_TEMPLATE/change-request.yml) and applies the `change-request` label. The CR states the reason for the change, a detailed description of what is proposed covering both hardware and software, and the identified hazards that may be affected. It is assigned a status of `Proposed`.

CR status values, tracked by GitHub label: `Proposed`, `Under Analysis`, `Authorized`, `Rejected`, `In Implementation`, `In Verification`, `Merged`, `Released`, `Closed`.

## Step 2 — Impact Analysis

The Impact Analyst completes the [Impact Analysis Template](templates/IMPACT_ANALYSIS.md) as a comment on the CR, or as a linked document where the analysis is long. The IA must identify:

- Which modules are changed, and which modules depend on them
- Which hardware items are changed
- Which safety requirements are affected
- Which HARA entries, architecture, design, FMEDA, test plan, traceability matrix and safety manual sections require update
- The proposed change class
- The earliest lifecycle phase the change must return to (Step 3)
- The specific tests that must be run to validate the change, and the specific tests that must be re-run to confirm nothing else regressed
- Whether any deviation from normal operating conditions is involved
- The effect on human interaction with the machine, and on the operating environment
- Any other modification currently in flight that could interact with this one
- Whether functional safety is preserved *during* the modification, as well as after it
- Whether the change affects a fielded unit and therefore triggers section 10

An IA that does not name specific tests is incomplete. The IA is documented on the CR before authorization.

## Step 3 — Determine the return-to-phase point

The IA states the earliest lifecycle phase the change re-enters. All subsequent phases are then executed under the normal development process.

| Nature of change | Return to |
|---|---|
| Hazard newly identified or reassessed | HARA |
| Safety requirement added, removed or altered | Safety Requirements Specification |
| Module interface, wire protocol, timing budget or state machine altered | Architecture / High Level Design |
| Internal logic of a module altered, interface unchanged | Detailed Design |
| Defect fix with no design consequence | Implementation |
| Hardware component, layout, or enclosure altered | System Architecture Design, plus FMEDA and diagnostic coverage review |

## Step 4 — Authorize

The Authorizer reviews the IA and records `Authorized` or `Rejected` on the CR with a dated comment stating the basis for the decision. Authorization rests on the assessment of the impact analysis and on the systematic capability claimed for the affected element, not on the description of the change alone. Class C requires two distinct Authorizers to comment.

Implementation does not begin before this step completes.

## Step 5 — Implement

The Implementer makes the change by pull request against the [protective-stop repository](https://github.com/polymathrobotics/protective-stop), with the CR issue number in the branch name and the PR title, and the CR linked from the PR body. Software, hardware and test changes all follow this same path; commit and PR history is the revision record for all three.

Changes to protocol or machine-safety logic are made upstream in `pstop_c` and consumed here, never edited in place.

All documentation identified in the IA is updated in the same change, not deferred. This includes any change to system procedures.

## Step 6 — Gate 0 and Gate 1

Executed per section 7. Results are attached to the CR before review.

## Step 7 — Review and approve

A Reviewer who is not the Implementer reviews the change against the coding standard and confirms that everything the IA required has been done. The GitHub pull request review is the review record, for hardware as well as software.

Merge is blocked until review passes and Gate 1 is green. On merge the CR moves to `Merged`.

## Step 8 — Close the change

The CR moves to `Closed` when it has been merged and its Gate 1 evidence is attached. A CR is never closed with outstanding IA actions. Release happens separately, in section 8.

# 7. Verification gates

Three gates. Each has a different scope and answers a different question.

## 7.1 Gate 0 — Build Acceptance Test

A fast smoke suite that runs on every push and on every release candidate, for every change class including Class A. Purpose is to fail early so that no expensive testing is spent on a broken candidate.

Contents: clean build of firmware, host and library; `pre-commit` clean; MISRA C:2012 pass with no new findings; the full unit test suite; one end-to-end protocol round trip including a bond, an arm, a stop, and a stop-on-silence.

A red BAT stops all downstream testing. Nothing merges or releases on a red BAT.

BAT is a filter. It does not qualify a change and it does not qualify a release.

## 7.2 Gate 1 — Merge qualification

Scope is tailored to the change, per the class and the IA. This answers "is this change correct and did it break its neighbours."

| Class | Required at Gate 1, after a green Gate 0 |
|---|---|
| A | Nothing further. |
| B | Unit tests for the changed module and for every module that directly depends on it. Integration tests covering the affected interfaces. |
| C | The above, plus every validation test that exercises an affected safety requirement, plus the traceability re-check in both directions: forward from the affected safety requirements to the re-verification and re-validation performed, and backward from that work to the requirements it covers. |
| Hardware change | The above for any coupled software, plus re-execution of environmental, EMC and fault injection testing where the IA finds the change could affect those results, plus FMEDA and diagnostic coverage review. |

Any change that alters a timing budget, the wire protocol or CRC, the bond or arming state machine, the lockstep comparator, or diagnostic coverage is Class C regardless of how small the code delta is.

## 7.3 Gate 2 — Release qualification

Scope is the whole system, untailored, run against the release candidate. This answers "does the assembled baseline still meet every safety requirement, including where two independently-merged changes interact."

Gate 2 covers every change since the previous release, not one CR. It runs in full regardless of how small the changes were.

Contents: every test that verifies a safety requirement, as listed in the traceability matrix. The Gate 2 suite is derived from the requirements, not from any change, which is what makes it an invariant. Today that means the arming-policy suite over the real wire protocol, the chaos ladder, the netem ladder, per-transport soaks across Ethernet, USB-NCM and WiFi, multi-remote and multi-machine validation, the two-site failover scenario, lockstep and fail-safe-silence checks, stop-on-silence timing, and full traceability closure across every CR in the release.

The suite changes only when the safety requirements change, and it changes through this procedure like anything else: a CR that adds or alters a requirement also adds or alters the Gate 2 entry that verifies it. It never varies because of what a particular change touched.

Type tests are the exception. EMC, environmental, fault injection and relay endurance are too costly to run every release, so they sit outside Gate 2 and are triggered by the Impact Analysis instead. When the IA says a change could affect them, they run before that release ships.

Gate 2 is honest about its own coverage. Where a safety requirement has no verifying test, the release record states that rather than passing silently.

At SIL 3, revalidation of the complete system is a highly recommended technique, and Gate 2 is how this procedure satisfies it. Regression validation at Gate 1 is the tailored alternative used per change; it does not replace Gate 2.

## 7.4 Scope of module re-verification

At SIL 3, re-verification covers the changed module and all modules affected by the change, as determined by the dependency analysis in the IA. Narrowing to the changed module alone is not permitted at this integrity target.

## 7.5 Recording results

Results at every gate are attached to the CR, or to the [Release Record](templates/RELEASE_RECORD.md) for Gate 2, with the date, the person or CI job that ran them, the software and hardware versions under test, and the pass or fail outcome per test. A summary line without underlying results is not evidence.

Gate results are analysed as a body, not only read individually. A rising failure rate in one area is a signal about the design, not just about the change that tripped it.

# 8. Release

A release aggregates every change merged since the previous baseline. It has its own record.

1. The Release Approver opens a [Release Record](templates/RELEASE_RECORD.md): a draft GitHub Release listing every CR included.
2. Gate 0 runs against the release candidate.
3. Gate 2 runs in full against the release candidate. Evidence is attached to the Release Record.
4. Traceability is closed across the whole release: every affected safety requirement traces forward to the verification performed and backward from that verification to the requirement.
5. The Release Approver confirms the evidence is complete and passing.
6. The baseline is tagged and published. Approved Versions For Deployment is updated with the new firmware, host and hardware versions.
7. Release notes are written using the Release Notes Template and reference every CR in the release.
8. The safety manual is reissued if any IA in the release required it.
9. Every CR in the release moves to `Released`.

A `pstop_c` version bump that changes the CRC is a wire break. Remote and machine are released and deployed together, and the release notes say so explicitly.

# 9. Emergency changes

An emergency change is one where a defect creates an immediate safety risk to a fielded unit, or blocks safe operation, and the normal timeline cannot be met.

The emergency path compresses the schedule. It does not remove steps.

1. The Requester raises the CR with the `emergency` label and notifies two Authorizers directly.
2. A short-form IA is completed covering, at minimum: what changed, what it could affect, and which tests will be run. It is recorded on the CR before implementation.
3. Two distinct Authorizers approve, in writing on the CR, regardless of class.
4. The change is implemented, reviewed by a second person, and Gate 0 is run. Gate 0 is never waived.
5. Gate 2 may be reduced to the subset the IA justifies, and the reduction is recorded on the Release Record with the reasoning and the Release Approver's name.
6. Release proceeds with the release notes marked `emergency`.
7. Within five working days of release, the full Impact Analysis is completed retrospectively, the omitted Gate 2 coverage is executed, and both records are updated. The CR remains open until this is done.

Emergency changes are reviewed as a group at the periodic safety review. A rising count is treated as a signal that the normal path is too slow.

# 10. Notifying users of a safety-affecting defect

Where a defect in a released baseline affects safety, the following applies in addition to the change process:

1. The defect is recorded on the CR with the `safety-defect` label at the point it is identified, before a fix exists.
2. The affected baselines and, where known, the affected deployments are identified.
3. A notice is published stating the defect, the affected versions, the interim mitigation, and the expected remedy. This is published as a GitHub security advisory on the repository and repeated in the release notes.
4. The notice is issued on identification of a safety-affecting defect, not deferred until a fix ships.

There is currently no register recording where PSTOP builds are deployed, so step 2 cannot be completed for external users. See gap G-2.

# 11. Records and where they live

| Record | Location |
|---|---|
| Change Request and status | [GitHub Issues](https://github.com/polymathrobotics/protective-stop/issues), `change-request` label |
| Impact Analysis | Comment or linked document on the CR issue |
| Authorization decision | Dated comment on the CR issue |
| Revision history, all of software, hardware and tests | Git commit and [pull request](https://github.com/polymathrobotics/protective-stop/pulls) history |
| Review record, all of software, hardware and tests | GitHub pull request review |
| Gate 0 and automated Gate 1 results | [GitHub Actions](https://github.com/polymathrobotics/protective-stop/actions) run linked from the CR |
| Manual Gate 1 and Gate 2 results | Attached to the CR or the Release Record; reports under [`docs/`](../) |
| Release Record | GitHub Release, tagged |
| Released baseline | Approved Versions For Deployment |
| Release notes | GitHub Release body, per the Release Notes Template |
| Safety-defect notice | GitHub security advisory, per [`SECURITY.md`](../../SECURITY.md) |

# 12. Equivalent rigor

Modification activities are planned, performed and documented with at least the same level of expertise, automated tooling, planning and management as the original development. The same coding standard applies, the same MISRA configuration and thresholds apply, the same review requirements apply, the same competency expectations apply to the people involved, and the same toolchain version is used unless a toolchain change is itself the subject of the CR.

A change is not a licence to work to a lower standard because it is small.

# 13. Referenced documents

| Document | Location | Status |
|---|---|---|
| System definition | [`docs/safety/SYSTEM_DEFINITION.md`](../safety/SYSTEM_DEFINITION.md) | Exists |
| HARA | [`docs/safety/HARA.md`](../safety/HARA.md) | Exists |
| **Safety requirements — authoritative baseline** | [`docs/safety/SAFETY_REQUIREMENTS.md`](../safety/SAFETY_REQUIREMENTS.md) | Exists. This is the baseline the Impact Analysis traces against. |
| FMEA | [`docs/safety/FMEA.md`](../safety/FMEA.md) | Exists |
| FMEDA and firm-up playbook | [`docs/safety/FMEDA.md`](../safety/FMEDA.md), [`docs/safety/FMEDA_FIRMUP_GUIDE.md`](../safety/FMEDA_FIRMUP_GUIDE.md) | Exists, unquantified |
| Traceability matrix | [`docs/safety/TRACEABILITY.md`](../safety/TRACEABILITY.md) | Exists. Defines the Gate 2 suite — see 7.3. |
| Structural coverage baselines and tool policy | [`docs/safety/COVERAGE.md`](../safety/COVERAGE.md) | Exists |
| Object-code diversity argument | [`docs/safety/DU3_OBJECT_CODE_DIVERSITY.md`](../safety/DU3_OBJECT_CODE_DIVERSITY.md) | Exists |
| Clock guard verification | [`docs/safety/CLOCK_GUARD_AND_GPIO_REVERIFY.md`](../safety/CLOCK_GUARD_AND_GPIO_REVERIFY.md), [`docs/safety/MACHN_CLOCK_GUARD_HIL.md`](../safety/MACHN_CLOCK_GUARD_HIL.md) | Exists |
| Document reconciliation record | [`docs/safety/RECONCILIATION.md`](../safety/RECONCILIATION.md) | Exists |
| Open items against a full quantified claim | [`docs/safety/OPEN_ITEMS.md`](../safety/OPEN_ITEMS.md) | Exists. Authoritative register for safety-case gaps; section 15 here covers process gaps only. |
| Test harness and validation approach | [`docs/TESTING.md`](../TESTING.md) | Exists |
| Connectivity soak procedure | [`docs/CONNECTIVITY_SOAK.md`](../CONNECTIVITY_SOAK.md) | Exists |
| Safety chain and recovery | [`docs/SAFETY_CHAIN.md`](../SAFETY_CHAIN.md), [`docs/RECOVERY_PLAYBOOK.md`](../RECOVERY_PLAYBOOK.md) | Exists |
| MISRA compliance and deviation register | [`docs/MISRA_COMPLIANCE_2026-07-21.md`](../MISRA_COMPLIANCE_2026-07-21.md) | Exists; excludes `pstop_c`, which is on its own track |
| Failover and arming design | [`docs/FAILOVER_AND_ARMING_DESIGN_2026-07-21.md`](../FAILOVER_AND_ARMING_DESIGN_2026-07-21.md) | Exists |
| Multi-remote validation and operation | [`docs/MULTI_REMOTE_VALIDATION_2026-07-22.md`](../MULTI_REMOTE_VALIDATION_2026-07-22.md), [`docs/MULTI_REMOTE_MULTI_MACHINE.md`](../MULTI_REMOTE_MULTI_MACHINE.md) | Exists |
| Two-site failover report | [`docs/TWO_SITE_FAILOVER_2026-07-21.md`](../TWO_SITE_FAILOVER_2026-07-21.md) | Exists |
| Contribution rules and CI gate | [`CONTRIBUTING.md`](../../CONTRIBUTING.md) | Exists |
| Security and defect reporting | [`SECURITY.md`](../../SECURITY.md) | Exists |
| Hardware design and per-file licence manifest | [`hardware/README.md`](../../hardware/README.md) | Exists, work in progress |
| Coding Standard | Notion: CS_D060 | Exists |
| High Level Software Design Specification | Notion: SWA_D049 | Exists |
| Validation Test Plan | Notion: VTP_D069 | Exists, in progress |
| Safety Requirements Specification (Notion SRS_D040) | Notion | **Superseded.** Retained as working notes only. `docs/safety/SAFETY_REQUIREMENTS.md` is authoritative. |
| Approved Versions For Deployment | Notion: CM-2 | Exists |
| Release Notes Template | Notion: CM-5 | Exists |
| Impact Analysis Template | [`templates/IMPACT_ANALYSIS.md`](templates/IMPACT_ANALYSIS.md) | Exists |
| Change Request Template | [`.github/ISSUE_TEMPLATE/change-request.yml`](../../.github/ISSUE_TEMPLATE/change-request.yml) | Exists |
| Integration Test Plan | — | Not yet written |
| Verification Plan | — | Not yet written |
| Non-Conformance Reporting Procedure | — | Not yet written |
| Corrective Action Procedure | — | Not yet written |
| Safety Manual | — | Not yet written |

# 14. Clause index

Clause numbers refer to IEC 61508:2010. The standard's text is not reproduced here; the numbers are given so that a reader holding the standard can check the mapping.

| Section here | IEC 61508:2010 clauses |
|---|---|
| 1 Scope — modification procedures exist before any change | Part 1 §7.16.2.1; Part 3 §7.8.2.1 |
| 4, 6 Step 1 — Raise the CR | Part 1 §7.16.2.2; Part 3 §6.2.3 d, §7.8.2.2 |
| 6 Step 2 — Impact Analysis | Part 1 §7.16.2.3; Part 2 §7.8.2.1 b; Part 3 §7.1.2.9, §7.8.2.3, Annex A.8.1 |
| 6 Step 2 — Impact Analysis is documented | Part 1 §7.16.2.4; Part 3 §7.8.2.4 |
| 6 Step 3 — Return to earlier phase | Part 1 §7.16.2.6; Part 3 §7.1.2.9, §7.8.2.3 b, §7.8.2.5 |
| 6 Step 4 — Authorize | Part 1 §7.16.2.5; Part 2 §7.8.2.1 c; Part 3 §6.2.3 d, §7.8.2.10 |
| 6 Step 5 — Implement as planned | Part 3 §7.8.2.7 |
| 6 Step 5 — Revision history and configuration management | Part 2 §7.8.2.1 f; Part 3 §6.2.3 c, §7.8.2.8 c, Annex A.8.5 |
| 7.1, 7.2, 7.3 Re-verification and re-validation after modification | Part 2 §7.8.2.4; Part 3 §7.8.2.6 d |
| 7.2 Regression validation, tailored per change | Part 3 Annex A.8.4 b |
| 7.3 Gate 2 — revalidation of the complete system | Part 3 Annex A.8.4 a |
| 7.2, 8 Traceability re-check, both directions | Part 3 Annex A.8.7, A.8.8 |
| 7.4 Scope of module re-verification | Part 3 Annex A.8.2 (changed module), Annex A.8.3 (affected modules) |
| 7.5 Recording and analysing results | Part 1 §7.16.2.7, §7.18.2.1 to §7.18.2.4; Part 2 §7.8.2.1 e; Part 3 §7.8.2.9, Annex A.8.6 |
| 7, 9 Verification planning for the modification | Part 3 §7.8.2.6 c |
| 8 Documentation and procedure updates released with the change | Part 2 §7.8.2.1 h, §7.8.2.1 i; Part 3 §7.8.2.8 e |
| 10 User notification of a safety-affecting defect | Part 2 §7.8.2.2 |
| 11 Records | Part 1 §7.16.2.7; Part 2 §7.8.2.1 a, c, d, e, f, g, h, i; Part 3 §7.8.2.8 a to e |
| 3, 12 Competency, tooling, planning and management equal to original development | Part 2 §7.8.2.3; Part 3 §7.8.2.5, §7.8.2.6 a, §7.8.2.7 |

# 15. Declared gaps

These are open. They are listed so that the limits of this procedure are stated rather than implied.

| ID | Gap | Consequence | Owner |
|---|---|---|---|
| G-1 | Resolved. `docs/safety/SAFETY_REQUIREMENTS.md` is the authoritative safety requirements baseline; Notion SRS_D040 is superseded working notes. | None. Retained for the record. | — |
| G-2 | No deployment register exists recording where PSTOP builds are running. | Section 10 step 2 cannot be executed for external users. Safety-defect notification is best-effort only. | Raj |
| G-3 | The Gate 0 BAT suite is defined in principle but not yet enumerated as a fixed, named set of tests with pass criteria. | Section 7.1 is not executable as written. | Ilia / John |
| G-4 | The Gate 2 suite is derivable from the traceability matrix, but that matrix currently shows a minority of safety requirements strictly verified. | A passing Gate 2 today certifies less than it appears to. The release record must state the unverified requirements explicitly. | Ilia / John |
| G-5 | No Verification Plan, Integration Test Plan, Non-Conformance Reporting procedure or Corrective Action procedure exists. | Steps 4 and 6 reference processes that are not yet documented. | Raj |
| G-6 | The stop-on-silence timing in the requirement baseline and in the shipped configuration do not agree. | Traceability cannot be closed on the affected safety requirement. Deferred by decision; recorded here so it is not lost. | Ilia |
| G-7 | No change has yet been processed through this procedure. | There are no change records to demonstrate the procedure is followed in practice. | Raj |
