> One Release Record per release, covering every change merged since the previous baseline. This is the body of the GitHub Release. Gate 2 evidence attaches here, not to any individual Change Request.

# Identification

| Field | Value |
|---|---|
| Tag |  |
| Previous baseline |  |
| Firmware version |  |
| Host / machine wrapper version |  |
| `pstop_c` version |  |
| Hardware revision |  |
| Release Approver |  |
| Date |  |

# 1. Changes included

Every Change Request merged since the previous baseline. A release with an unlisted merged change is incomplete.

| CR | Class | Summary | Gate 1 passed |
|---|---|---|---|
|  |  |  |  |

# 2. Interaction assessment

Where two or more changes in this release touch related areas, state what could interact and what was done about it. This is the question Gate 1 cannot answer, because each change was tested alone.

# 3. Wire compatibility

- Did `pstop_c` change in a way that alters the CRC or wire format? Yes / No.
- If yes, this is a wire break. Remote and machine must be updated together. State that plainly in the release notes.

# 4. Gate 0 — Build Acceptance Test on the release candidate

| Version under test | CI run | Date | Result |
|---|---|---|---|
|  |  |  |  |

# 5. Gate 2 — Release qualification

Run in full against the release candidate, regardless of how small the changes were. Any omission is a deviation and is recorded in section 8.

| Suite | Version under test | Run by | Date | Result |
|---|---|---|---|---|
| Arming policy suite over the real wire protocol |  |  |  |  |
| Chaos ladder — loss, delay, duplication, corruption |  |  |  |  |
| Netem ladder — underlay impairment |  |  |  |  |
| Soak — Ethernet |  |  |  |  |
| Soak — USB-NCM |  |  |  |  |
| Soak — WiFi |  |  |  |  |
| Multi-remote and multi-machine validation |  |  |  |  |
| Two-site failover |  |  |  |  |
| Lockstep mismatch and fail-safe silence |  |  |  |  |
| Stop-on-silence timing |  |  |  |  |

This list is provisional until the Validation Test Plan fixes it as a named set with pass criteria. See gap G-4.

# 6. Traceability closure

Across every change in this release, not per change.

- [ ] Forward: every affected safety requirement traces to the verification performed
- [ ] Backward: every piece of verification performed traces to the requirement it covers
- [ ] Traceability matrix updated and committed

# 7. Documentation

- [ ] Release notes written, referencing every CR
- [ ] Approved Versions For Deployment updated
- [ ] Safety manual reissued, or recorded as not required
- [ ] Any system procedure changes published
- [ ] Open items register updated

# 8. Deviations

Anything omitted from Gate 2, the reasoning, and the Release Approver who accepted it. An emergency release records its reduced scope and the date by which the omitted coverage will be run.

# 9. Approval

| Release Approver | Date | Statement |
|---|---|---|
|  |  | Gate 2 evidence is complete and passing; this baseline is approved for release. |
