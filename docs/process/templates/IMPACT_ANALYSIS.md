> Copy this into a comment on the Change Request issue, or into a linked document where the analysis is long. Every field is answered. "None" is a valid answer; blank is not.

# Identification

| Field | Value |
|---|---|
| Change Request | GitHub issue number and link |
| Analyst | Name |
| Date | YYYY-MM-DD |
| Baseline affected | Software version and hardware revision this change applies to |

# 1. What is changing

One paragraph in plain language. What is being changed and why.

# 2. Affected software

- Modules changed:
- Modules that directly depend on the changed modules:
- Is any changed module a safety module (implements or influences Safety Requirements Specification section 5)? Yes / No, and which.
- Does the change alter any of the following? Answer each Yes / No.
  - Public interface or API signature
  - Message protocol or wire format
  - Timing budget, including the bond-loss stop latency
  - Bond / stop / OK / unbond state machine
  - Diagnostic or fault-detection behaviour
  - Memory allocation behaviour

# 3. Affected hardware

- Items changed (schematic, PCB, BOM line, mechanical part, enclosure CAD):
- Is any changed item in the stop signal path? Yes / No.
- Could the change plausibly affect environmental performance, EMC behaviour, or fault-injection results? Yes / No, with reasoning.
- Does the change affect the FMEDA, the diagnostic coverage argument, or the common-cause argument? Yes / No, with reasoning.

# 3b. Wider effects

- Effect on human interaction with the machine:
- Effect on the operating environment, or assumptions about it:
- Other modifications currently in flight that could interact with this one:
- Is functional safety preserved *during* the modification, as well as after it? State how.

# 4. Affected requirements and documents

| Artifact | Affected? | What must change |
|---|---|---|
| Safety requirements baseline (state which — see gap G-1) |  |  |
| HARA |  |  |
| FMEDA / diagnostic coverage / common cause |  |  |
| System Architecture Design |  |  |
| High Level Software Design |  |  |
| Detailed Software Design |  |  |
| Coding Standard |  |  |
| Validation Test Plan |  |  |
| Integration Test Plan |  |  |
| Traceability matrix |  |  |
| Safety Manual |  |  |
| Release notes |  |  |

# 5. Proposed change class

A, B or C, with one sentence of justification. Where the class is uncertain, the higher class is proposed.

# 6. Return-to-phase point

The earliest lifecycle phase this change re-enters, and why. All later phases are then executed under the normal development process.

# 7. Verification plan for this change

Name the tests. A plan that says "run the relevant tests" is not complete. This defines Gate 1 scope only; Gate 2 release qualification runs in full regardless of what is entered here.

| Purpose | Specific tests |
|---|---|
| Tests that validate the change itself |  |
| Tests re-run to confirm nothing else regressed |  |
| New tests that must be written |  |
| Hardware tests to re-run, if any |  |

A complete Gate 0 Build Acceptance Test runs regardless of what is entered above, and a red Gate 0 stops all of it.

# 8. Deviations

Any deviation from normal operating conditions, normal process, or normal roles involved in this change, and the justification for it. Includes the case where the Authorizer and Implementer are the same person.

# 9. Fielded units

- Does this change relate to a defect present in a released baseline? Yes / No.
- Does that defect affect safety? Yes / No.
- If yes to both, section 10 of the Modification Procedure applies. Record the affected baselines here.

# 10. Analyst conclusion

One paragraph. What this change touches, what could go wrong if the analysis is incomplete, and the recommendation to the Authorizer.
