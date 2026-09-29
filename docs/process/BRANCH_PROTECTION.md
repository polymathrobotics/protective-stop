# Intended Branch Protection

These are intended settings for a repository administrator to apply to `main`. This document does not assert that the settings are currently active; protection details were not readable by the implementing account.

| Setting | Value | Rationale |
|---|---|---|
| Require a pull request before merging | On | Modification Procedure merge gate |
| Required approvals | 1 | Class C's second approval is checked by `change-control`, so the rule follows class rather than burdening every change |
| Dismiss stale approvals on new commits | On | An approval applies to a diff, not a branch name |
| Require review from Code Owners | On | Requires one of the three authorizers |
| Required status checks | `pre-commit`, `host-check`, `firmware-build`, `ros2_build`, `pstop_c_build`, `wire-break` | Gate 0 subset; add `change-control` only after enforce mode is authorized |
| Require branches up to date | On | Independently green changes can fail together |
| Allow force pushes | Off | Published history stands |
| Allow deletions | Off | Preserve the protected baseline |

The repository reports `main` as protected, but the classic protection-details endpoint returned 404 and no applicable rules were visible. These intended settings must be compared with live settings by an administrator before application.
