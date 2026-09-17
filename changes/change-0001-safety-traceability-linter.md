# change-0001 — Safety traceability linter: machine-checked requirements coverage

**Implements:** No ADR. Authority is the safety chain itself —
`docs/safety/SAFETY_REQUIREMENTS.md`, `docs/safety/TRACEABILITY.md`,
`docs/safety/SYSTEM_DEFINITION.md`.
**Branch:** `change-0001-safety-traceability-linter`
**Base:** `main`
**Status:** implementation complete; PR CI pending
**Safety class:** B — assurance tooling and CI only; no runtime safety-path,
interface, timing, state-machine, protocol, or requirement change.
**Authorization:** one authorizer approved implementation with the exact instruction
`proceed` on 2026-09-11.

Read `docs/safety/TRACEABILITY.md` §1 (Method) and §3 (coverage summary) in full
before starting, then `docs/safety/SAFETY_REQUIREMENTS.md` §1 (numbering and
conventions). Those documents are the authority; this document is the work
breakdown. Where the two disagree, the safety documents win — stop and flag the
conflict rather than choosing.

The repository is the source of truth for the safety case. Where any Notion page
disagrees with a file under `docs/safety/`, the file wins
(`docs/safety/RECONCILIATION.md` establishes this).

The design decisions in §1 and §1a are settled and are not open for
reinterpretation during implementation.

---

## 1. What this change delivers

A checked-in linter that parses the existing safety markdown, asserts
bidirectional traceability consistency across `SAFETY_REQUIREMENTS.md`,
`TRACEABILITY.md` and `SYSTEM_DEFINITION.md`, computes the requirements-coverage
numbers, regenerates the §3 summary block in `TRACEABILITY.md`, and fails CI when
a change breaks traceability or silently drops a requirement's verification.

**Settled decision 1 — markdown is canonical; the linter parses it.** The SR
tables carry hand-authored safety argument in the Status column (reconciliation
dates, commit hashes, named residuals). Extracting them into YAML and generating
the markdown back would flatten that prose into quoted strings and make review
worse, not better. The linter reads the tables and never rewrites them, with one
exception: the generated block in §4d.

**Settled decision 2 — stdlib only.** No `pyyaml`, no `pytest`, no new package
manager. The repo has no Python packaging (`pyproject.toml` absent) and its
existing Python tools are standalone `python3` scripts. Baseline and output files
are JSON, which the `polymath-json` pre-commit hook already covers.

**Settled decision 3 — ratchet, not gate.** The tree does not pass these checks
today: five functions carry no requirement, and `OPEN_ITEMS.md` and
`TRACEABILITY.md` disagree on the requirement count. A baseline file records every
pre-existing violation with a reason. CI fails on new violations only, and fails
when a baselined violation is fixed but not removed from the baseline.

### 1a. Ratified implementation clarifications (2026-09-11)

1. Define the SRS status vocabulary in `checks.py` as `Satisfied`,
   `Partially satisfied`, `Gap`, and `Residual-accepted`, longest-match-first.
   Do not derive it from the conventions prose and do not edit that prose. Emit an
   informational finding when a status in use is absent from the conventions list.
2. The CI workflow has no path filter. It runs on every pull request and every push
   to `main` so moved or deleted evidence cannot escape the check.
3. Baseline the broken `test_timing_floors` citation with owner `raj`. Git history
   establishes that `test_timing_floors.cpp` was added by `1f226be` and deleted by
   `9a28d4a` during the ROS 2 convention restoration. Record those facts without
   deciding whether SR-M-01 remains Verified.
4. Do not rearrange `TRACEABILITY.md`. Use multiple named generated marker pairs
   around purely numeric regions. A bullet mixing numbers with hand-authored prose
   remains outside markers and is checked rather than rewritten. If markers cannot
   meet that rule, stop and quote the resisting text.
5. Evidence resolves only through the documented shorthand, an explicit repository
   path, a repository-wide unique filename or test-source stem, or an existing HIL
   or evidence report that names the cited SR. An ambiguous stem is a finding. Never
   choose the nearest match and never infer evidence from prose.
6. The linter measures citations, not passing execution. Generated output is labelled
   "SRs with at least one cited verifying test" and includes a footnote that citation
   resolution, not test execution, is checked. Existing "passing" wording is not
   edited. If the generated label conflicts with surrounding prose, stop and report.
   If computed citation coverage differs from committed 32/40, report both values and
   the per-SR delta and change nothing.

---

## 2. Scope

### IN scope

1. `tools/safety_lint/` — parsers for the three safety documents.
2. Nine consistency checks (§4c), each with an error/warning severity.
3. Coverage computation reproducing the §3.1 and §3.2 numbers from the parsed
   data.
4. Generated-block rendering into `TRACEABILITY.md` §3 between HTML markers, with
   a `--check` mode that fails when the committed block is stale.
5. `docs/safety/lint-baseline.json` — recorded pre-existing violations with
   reasons and owners.
6. `.github/workflows/safety-lint.yml` — runs the linter on every PR and on every
   push to `main`, without path filters.
7. `tools/safety_lint/self_test.py` — the linter's own test suite, stdlib
   `unittest`, run by the same workflow.

### OUT of scope — do NOT build

- **Any edit to the safety argument itself.** Do not add, remove, reword or
  re-status a requirement. Do not change a Status cell to make a check pass. If a
  check fails on real content, baseline it and report it.
- Any file under `docs/safety/` other than approved generated/check-only markers in
  `TRACEABILITY.md` §3 and the new `lint-baseline.json`. In particular, do not edit
  the SRS conventions section.
- Anything under `pstop_c/` — certified library, separate upstream track,
  excluded from pre-commit for that reason.
- Anything under `ros2/` — governed by ament linters.
- Line-number verification of `file:line` code citations. Files are checked to
  exist; line numbers drift constantly and are explicitly documented as "±a few
  lines as the tree evolves" (`SAFETY_REQUIREMENTS.md` §1). Do not assert on them.
- Parsing `FMEA.md`, `HARA.md` or `FMEDA.md` beyond extracting bare ID sets for
  check C7.
- Structural-coverage integration. `COVERAGE.md` and `docs/safety/coverage/host-summary.json`
  stay out; this change measures requirements coverage only.
- The modification procedure migration from Notion. Separate change.
- Any CI gate on `main` branch protection. The workflow is added; enabling it as a
  required check is a repo-admin action, not this change.

---

## 3. Integration facts (do not rediscover these)

| Fact | Value |
|---|---|
| Requirements spec | `docs/safety/SAFETY_REQUIREMENTS.md` |
| Traceability matrix | `docs/safety/TRACEABILITY.md` |
| Function decomposition | `docs/safety/SYSTEM_DEFINITION.md` §4 (`## 4. Function / item decomposition`, line 95) |
| FMEA (DU register) | `docs/safety/FMEA.md` §3 |
| HARA (safety goals SG-1..6, hazards H-nn) | `docs/safety/HARA.md` §5 |
| Reconciliation record ("code wins") | `docs/safety/RECONCILIATION.md` |
| Open items register | `docs/safety/OPEN_ITEMS.md` |
| SR ID grammar | `SR-<area>-<nn>`; areas `SYS`, `R`, `H`, `M`, `I` (`SAFETY_REQUIREMENTS.md` §1) |
| SR table columns | `ID · Requirement (shall) · Derived from · Allocated to · Integrity · Verify · Status` |
| SR section headings | `## 2.` SR-SYS, `## 3.` SR-R (subsections `### 3.1`, `### 3.2`), `## 4.` SR-H, `## 5.` SR-M, `## 6.` SR-I |
| SRS status vocabulary | `Satisfied`, `Partially satisfied`, `Gap`, `Residual-accepted` — bold-wrapped, often followed by parenthetical prose |
| Traceability table columns | `SR · Alloc F-xx · Code (file:line) · Verifying test(s) · Method · Status` |
| Traceability section headings | `## 2. Traceability matrix` with `### 2.1`–`### 2.5`; `## 3. Requirements coverage summary`; `## 4. Test-gap register`; `## 5. Function → SR reverse map` with `### 5.1`–`### 5.x` |
| Traceability status vocabulary | `Verified`, `Partially-verified`, `Unverified-gap`, `Residual-accepted` (defined in a table under `## 1. Method`) |
| No-test marker | The literal string `NO TEST`, bold-wrapped, in the Verifying test(s) cell |
| Test-file shorthand legend | `TRACEABILITY.md` §2 preamble: `EV`, `MR`, `HIL10/20/30`, `JL`, `REQ n_nn` — each maps to a real path |
| `EV` | `firmware/test/test_estop_verdict.c` |
| `MR` | `tools/pstop_multi_remote_test.py` |
| `HIL10/20/30` | `tools/hil/test_10_button.py`, `test_20_discordance.py`, `test_30_power_cycle.py` |
| `JL` | `ros2/protective_stop_machine/test/test_json_lite.cpp` |
| `REQ n_nn` | `pstop_c/pstop/test/src/pstop/requirements/req_n_nn_test.c` |
| Existing CI guard script pattern | `scripts/check_estop_diversity.sh` — SPDX header, comment block naming the SR and DU it guards, `set -uo pipefail`, exit 2 for "cannot run", exit 1 for "check failed" |
| How a guard is wired to CI | `.github/workflows/firmware-build.yml:46-52` |
| Existing standalone Python test pattern | `tools/test_config_floor.py` — `#!/usr/bin/env python3`, SPDX header, module docstring naming the SR and DU |
| Existing workflows | `pre-commit.yml`, `firmware-build.yml`, `host-check.yml`, `ros2_build.yml`, `pstop_c_build.yml`, `pstop_c_coverage.yml`, `coverage.yml` |
| Checkout action version in use | `actions/checkout@v7` |
| Pre-commit config | `.pre-commit-config.yaml`, `polymath_code_standard` v2.2.0; hooks include `polymath-python`, `polymath-json`, `polymath-markdown`, `polymath-copyright` |
| Pre-commit exclusions | `pstop_c/`, `ros2/`, `archive/`, vendored wireguard/x25519, `hardware/` binaries. `tools/` and `docs/` are NOT excluded — new files must satisfy the hooks |
| Required file header | SPDX two-line header, Apache-2.0, matching `tools/test_config_floor.py` and `scripts/check_estop_diversity.sh` |
| Markdown hook caveat | `OPEN_ITEMS.md` §8 records that the markdown hook rewrites `-` list markers to `+` and that repo-wide normalization is deliberately deferred. Do not let the hook reformat `TRACEABILITY.md` wholesale — only the generated block changes |

### The requirement count disagreement

`TRACEABILITY.md` §3.1 states 40 safety requirements. `OPEN_ITEMS.md` §5 states
39 with a different coverage fraction, dated earlier. **Do not resolve this by
editing either document.** Piece 4 computes the count from the parsed SR tables.
Report the computed number. If it is neither 39 nor 40, stop and raise before
proceeding — the parser is wrong, not the documents.

---

## 4. Pieces, in dependency order

Each piece is committable on its own. Pieces 1–3 add no CI surface and cannot
break the build. Piece 5 is the first piece that can fail a PR.

---

### Piece 1 — SR parser

**Files:** `tools/safety_lint/__init__.py`, `tools/safety_lint/parse_srs.py`,
`tools/safety_lint/model.py`.

**1a. Record types** in `model.py`, frozen dataclasses, stdlib only:

```python
@dataclass(frozen=True)
class SafetyRequirement:
    sr_id: str                  # "SR-R-03"
    area: str                   # "SYS" | "R" | "H" | "M" | "I"
    number: int                 # 3
    shall_text: str             # raw cell, markdown intact
    derived_from: tuple[str, ...]   # ("SG-1", "H-09", "DU-3")
    allocated_to: tuple[str, ...]   # ("F-R-02",)
    integrity: str              # raw cell
    verify_methods: tuple[str, ...] # ("Fault-injection", "Test")
    status: str                 # normalized token from the SRS vocabulary
    status_prose: str           # everything after the token
    source_line: int            # 1-based line in SAFETY_REQUIREMENTS.md
```

**1b. Table discovery.** Walk the file line by line. A requirements table is a
markdown pipe table whose header row's first cell is `ID` and which contains a
cell `Derived from`. Do not hardcode section numbers — `SAFETY_REQUIREMENTS.md`
has subsections (`### 3.1`, `### 3.2`) and more may be added. Ignore every other
pipe table in the file, including the ID-grammar table in §1.

**1c. Cell splitting.** Cells are separated by unescaped `|`. Cells contain inline
code spans with pipes inside them in at least one row — split on `|` only when not
inside a backtick span. Trim, then strip surrounding `**`.

**1d. ID parse.** `SR-(SYS|R|H|M|I)-(\d{2})`, taken from the bolded first cell.
An ID that does not match is an error, not a skip.

**1e. Reference extraction.** From `derived_from`, extract every token matching
`SG-\d`, `H-\d{2}`, `DU-\d`, and FMEA cross-refs of the form `[A-Z]\d{2}-\d`.
From `allocated_to`, extract every `F-[A-Z]-\d{2}`. Expand the range form
`F-R-01..05` into its members. Expand the slash form `F-M-03/04` into
`F-M-03`, `F-M-04`. Both forms occur.

**1f. Status normalization.** Match the leading bolded token against the SRS
vocabulary, case-insensitively, longest match first so `Partially satisfied` is
not read as `Satisfied`. Note that `Partially satisfied — feedback limb
DESCOPED 2026-08` occurs; everything after the token is `status_prose`. A cell
whose leading token is outside the vocabulary is an error naming the file, line
and cell.

**Tests** (`self_test.py`, stdlib `unittest`):
- `test_parses_all_sr_areas` — parse the real `SAFETY_REQUIREMENTS.md`; assert at
  least one SR in each of the five areas.
- `test_sr_ids_unique` — no duplicate `sr_id`.
- `test_allocated_to_range_expansion` — `"F-R-01..05, F-H-03"` yields six
  functions including `F-R-03`.
- `test_allocated_to_slash_expansion` — `"F-M-03/04"` yields two.
- `test_status_longest_match_wins` — a cell beginning `**Partially satisfied**`
  normalizes to `Partially satisfied`, never `Satisfied`. Drift-verify: reorder
  the vocabulary so the short token matches first, confirm the test fails, revert.
- `test_pipe_inside_code_span_does_not_split_cell` — synthetic row containing
  `` `a|b` `` parses as one cell.

---

### Piece 2 — Traceability and system-definition parsers

**Files:** `tools/safety_lint/parse_traceability.py`,
`tools/safety_lint/parse_system_definition.py`, `tools/safety_lint/model.py`.

**2a. `TraceRow`** — `sr_id`, `allocated_to`, `code_refs`, `test_refs`,
`methods`, `status`, `status_prose`, `has_no_test_marker`, `source_line`.

**2b. Matrix discovery.** A traceability table's header row's first cell is `SR`
and it contains a cell beginning `Verifying test`. Same subsection caveat as 1b.

**2c. Code-reference extraction.** From the Code cell, extract tokens shaped
`path:line` or `path:line-line` or a bare backticked filename. Resolve a bare
filename against the paths named in `SYSTEM_DEFINITION.md` §4 where possible;
where not, record it as unresolved rather than failing.

**2d. Test-reference extraction.** Expand the shorthand legend from §3 above into
real paths. Handle the group-letter form `MR[B/D/E/F]` — the path is the same, the
bracket is a group selector, so record the path once. Handle `REQ 2_02/2_03` as
two `req_*_test.c` paths. Also resolve explicit repository paths and a bare filename
or test-source stem only when it is unique across the whole repository. Ambiguity is
a finding, never a nearest-match choice. An HIL/evidence report counts only when the
file exists and names the row's SR. Never infer evidence from prose. Record the
literal `NO TEST` marker separately from test paths; a cell may contain both a
`NO TEST` phrase and real tests, and it does in several rows.

**2e. Reverse-map parser.** Parse §5's tables into `{F-xx: (function_name,
tuple_of_sr_ids)}`. A cell whose SR list is an em-dash-led "none" phrase yields an
empty tuple plus a `declared_non_safety` flag when the phrase contains
`non-safety`.

**2f. System-definition parser.** Parse `SYSTEM_DEFINITION.md` §4's tables into
the authoritative `F-xx` set with names. This is the spine both other documents
are checked against.

**Tests:**
- `test_every_trace_row_has_sr_id`
- `test_test_shorthand_expands_to_existing_paths` — every expanded shorthand path
  exists on disk.
- `test_mr_group_selector_yields_single_path` — `MR[B/D/E/F]` yields one path.
- `test_req_slash_form_yields_two_paths`
- `test_no_test_marker_detected_alongside_real_tests` — a cell with both records
  `has_no_test_marker=True` and a non-empty `test_refs`.
- `test_reverse_map_flags_declared_non_safety` — `F-R-08` parses with an empty SR
  tuple and `declared_non_safety=True`.
- `test_system_definition_function_set_nonempty`

---

### Piece 3 — Consistency checks

**Files:** `tools/safety_lint/checks.py`.

Each check returns zero or more `Finding(check_id, severity, subject, message,
file, line)`. `severity` is `error` or `warning`.

**C1 — SR set parity.** Every SR in `SAFETY_REQUIREMENTS.md` appears exactly once
in the traceability matrix, and vice versa. Severity: error. Subject: the SR ID.

**C2 — Status vocabulary closed.** Every SRS status normalizes to the SRS
vocabulary; every traceability status normalizes to the traceability vocabulary.
Severity: error.

**C3 — Status and evidence agree.** A traceability row with status `Verified` or
`Partially-verified` names at least one resolvable test path. A row with status
`Unverified-gap` carries the `NO TEST` marker and names no test path outside a
`NO TEST` clause. Severity: error. This is the check that catches a requirement
losing its test silently.

**C4 — Cited test files exist.** Every resolved test path exists in the working
tree. Severity: error. Paths under `pstop_c/` are checked for existence but never
opened.

**C5 — Cited code files exist.** Every resolved code path exists. Severity:
error for a missing file. Line numbers are never checked — see §2 OUT of scope.
An unresolved bare filename is severity `warning`.

**C6 — Function allocation is real.** Every `F-xx` in any `Allocated to` cell
exists in the `SYSTEM_DEFINITION.md` §4 set. Severity: error.

**C7 — Upstream references resolve.** Every `SG-n` appears in `HARA.md`, every
`H-nn` appears in `HARA.md`, every `DU-n` appears in `FMEA.md`. Extract bare ID
sets by regex from those files; do not parse their structure. Severity: warning —
those documents restructure and a false positive here must not block a PR.

**C8 — Reverse map completeness.** Every `F-xx` in the system-definition set
appears in the reverse map, and the reverse map's SR list for each function equals
the set of SRs whose `Allocated to` names it. Severity: error for a disagreement,
warning for a function with no SRs at all — the latter is a real known state
(`F-H-04`, `F-M-01`, `F-M-06`) and belongs in the baseline.

**C9 — SRS and traceability agree on allocation.** For each SR, the `Allocated
to` set in the SRS equals the `Alloc F-xx` set in the matrix. Severity: error.

**Baseline handling.** `docs/safety/lint-baseline.json`:

```json
{
  "generated": "2026-09-10",
  "note": "Pre-existing findings accepted at linter introduction. Each entry needs an exact finding message, owner, and reason. Removing a fixed entry is required; CI fails on a stale entry.",
  "findings": [
    {
      "check_id": "C8",
      "subject": "F-H-04",
      "finding": "function has no allocated safety requirement",
      "reason": "Robot status output + logging carries no SR. Known requirements-coverage hole, TRACEABILITY.md §5.2.",
      "owner": "raj"
    }
  ]
}
```

A baseline entry matches exactly one finding by `(check_id, subject, finding)`,
where `finding` is the complete stable finding message. That exact finding is
downgraded to informational and reported in a separate section. A new finding on
the same check and subject remains active. Duplicate exact entries are invalid,
and a baselined entry with no exact matching finding is itself an error — this is
the ratchet.

**Tests:**
- One `test_<check_id>_flags_<condition>` per check, driven by small synthetic
  markdown fixtures under `tools/safety_lint/fixtures/`, not by the real
  documents. The real documents are exercised in Piece 4's integration test.
- `test_baseline_suppresses_exact_matching_finding`
- `test_same_subject_new_finding_is_not_suppressed`
- `test_stale_exact_baseline_entry_is_an_error` — a baselined exact finding with
  no corresponding finding produces an error.
- `test_duplicate_exact_baseline_entries_are_rejected`
- Drift-verify C3 and C8: break the guarded condition in a fixture, confirm the
  test fails, revert. Report that you did it.

---

### Piece 4 — Coverage computation and generated summary

**Files:** `tools/safety_lint/coverage.py`, `tools/safety_lint/render.py`,
`docs/safety/TRACEABILITY.md` (generated block only).

**4a. Counts.** From the parsed traceability rows, compute per area and in total:
count, and counts of `Verified`, `Partially-verified`, `Unverified-gap`,
`Residual-accepted`.

**4b. The two headline fractions**, matching the definitions in §3.1 verbatim:
- **(a) SRs with at least one passing verifying test** = rows whose `test_refs` is
  non-empty, irrespective of status. Note that this currently includes one
  `Residual-accepted` row that has a test (`SR-M-06`) and excludes the other
  (`SR-M-04`, inspection only). Do not special-case by status; count by evidence.
- **(b) Functions traced to at least one SR** = functions in the system-definition
  set with a non-empty SR list. Report both the raw fraction and the fraction
  excluding functions flagged `declared_non_safety`.
- **Strict, fully-verified** = rows with status exactly `Verified`.

**4c. Reproduce before you replace.** Run the computation against the current
tree and compare with the committed §3.1 and §3.2 numbers. They should match. If
any figure differs, **stop and report the discrepancy with both numbers** before
touching the document. A mismatch means either the parser is wrong or the
committed summary is stale; both need a human decision, and the second is a real
finding worth surfacing.

**4d. Generated regions.** Use multiple named marker pairs around purely numeric
regions only:

```
<!-- BEGIN GENERATED: safety-lint <name> -->
...
<!-- END GENERATED: safety-lint <name> -->
```

The surrounding prose in §3 — mixed numeric/prose bullets, the "Reading:"
paragraph, the `†` footnote, and bracketed reconciliation notes — is hand-authored
and stays outside the markers without being moved. Mixed bullets are checked, not
rewritten. Generated coverage is explicitly labelled "SRs with at least one cited
verifying test" and states that the linter checks citation resolution, not test
execution. Rendering replaces only marker contents. `--check` mode renders and
diffs without writing, exiting non-zero on a difference.

**Tests:**
- `test_coverage_matches_committed_summary` — integration against the real
  documents. If Piece 4c found a legitimate discrepancy, this test asserts the
  computed value and carries a comment naming the discrepancy and its resolution.
- `test_generated_block_is_idempotent` — render twice, byte-identical.
- `test_check_mode_detects_stale_block` — mutate a number inside the markers,
  assert `--check` exits non-zero.
- `test_render_does_not_touch_prose_outside_markers` — assert the "Reading:"
  paragraph is byte-identical before and after a render.

---

### Piece 5 — CLI and CI wiring

**Files:** `tools/safety_lint/__main__.py`, `.github/workflows/safety-lint.yml`.

**5a. CLI.** `python3 -m tools.safety_lint [--check] [--write] [--json]`, run from
the repository root.
- Default: run all checks, print a human-readable report, exit 1 on any
  non-baselined error, 0 otherwise.
- `--check`: also verify the generated block is current; exit 1 if stale.
- `--write`: regenerate the block in place.
- `--json`: emit findings as JSON to stdout for future tooling.

Exit codes follow `scripts/check_estop_diversity.sh`: `0` pass, `1` check failed,
`2` cannot run (a required document missing or unparseable).

**5b. Report format.** Errors first, then warnings, then a baselined section, then
the coverage summary. Every finding prints `file:line: [check_id] subject —
message`. The coverage summary prints the same numbers that go into the generated
block, so a reviewer reading CI output does not need to open the diff.

**5c. Workflow.** `.github/workflows/safety-lint.yml`, modelled on the existing
workflows: `actions/checkout@v7`, `ubuntu-latest`, triggers on every push to `main`
and every `pull_request`, with no path filters. Two steps: run `self_test.py`, then
run the linter with `--check`.

**5d. Do not add this to `.pre-commit-config.yaml`.** The linter reads several
files and computes cross-document state; pre-commit's per-file model fits it
badly and the markdown hook's list-marker rewrite is a known landmine
(`OPEN_ITEMS.md` §8). CI only.

**Tests:**
- `test_cli_exit_code_zero_on_clean_tree` — with the baseline in place, the real
  tree exits 0.
- `test_cli_exit_code_one_on_injected_error` — copy the safety docs to a temp
  directory, delete a traceability row, assert exit 1 and a C1 finding.
- `test_cli_exit_code_two_on_missing_document`
- Workflow: confirm it runs and passes on the PR that introduces it. Paste the
  run URL.

---

## 5. Definition of Done

Non-negotiable. A piece is not done until all of these are true and reported.

1. **Tests written first** (red → green) and exercising the real parsing path — no
   monkeypatching the parser under test, no asserting on hand-built record objects
   where the point is that the parser produced them.
2. **Drift-verify each guard test** named above: break the guarded thing, confirm
   the test fails, revert. Report that you did it. C3, C8 and the status
   longest-match test are the three that matter most.
3. **Run the real gate yourself and paste actual output:**
   `python3 tools/safety_lint/self_test.py` and
   `python3 -m tools.safety_lint --check`.
4. **Run `pre-commit run --all-files`** and paste the result. New files under
   `tools/` and `docs/` are in scope for the hooks; the SPDX header and
   `polymath-python` formatting are enforced.
5. **Confirm `git diff docs/safety/` touches only the generated block and
   `lint-baseline.json`.** Paste the diff. Any other change to a safety document
   is a scope violation and must be reverted.
6. **The baseline is justified line by line.** Every entry carries a reason
   naming the document section it comes from, and an owner. A baseline entry with
   a reason of "pre-existing" is not acceptable.
7. **Report two separate sections,** and do not merge them:
   - **"Out of scope, confirmed not built"**
   - **"In scope, required, not done"** — must be empty. Anything in it is
     blocking.
8. **Report the computed requirement count** and state whether it is 39, 40, or
   something else, with the per-area breakdown.

---

## 6. Acceptance criteria

- **AC-1 — parity.** C1 reports zero non-baselined findings against the current
  tree. Every SR in the spec appears exactly once in the matrix and vice versa.
- **AC-2 — count resolved.** The linter reports a single authoritative requirement
  count with a per-area breakdown, and the report states whether it agrees with
  `TRACEABILITY.md` §3.1, `OPEN_ITEMS.md` §5, both, or neither.
- **AC-3 — coverage reproduces.** The computed headline fractions match the
  committed §3.1 numbers, or every difference is reported with both values and an
  explanation before any document is modified.
- **AC-4 — evidence check bites.** Delete `tools/hil/test_10_button.py` in a
  scratch copy; the linter reports a C4 error naming `SR-SYS-03` among the
  affected requirements. Restore. Record the output.
- **AC-5 — silent-drop check bites.** In a scratch copy, change one
  `**Verified**` row's Verifying test cell to `**NO TEST**` while leaving the
  status as `Verified`; the linter reports a C3 error. Restore. Record the output.
- **AC-6 — ratchet works.** Remove one entry from `lint-baseline.json` without
  fixing the underlying issue; the linter reports the finding as a new error.
  Re-add it, then fix the underlying issue in a scratch copy without removing the
  baseline entry; the linter reports the stale entry as an error.
- **AC-7 — generated block is safe.** Running `--write` twice produces no diff on
  the second run, and the "Reading:" paragraph and the `†` footnote in §3 are
  byte-identical before and after.
- **AC-8 — pre-commit clean.** `pre-commit run --all-files` passes, and the run
  did not reformat any file outside this change's scope.
- **AC-9 — CI green.** `safety-lint.yml` passes on the introducing PR. Paste the
  run URL.

---

## 7. Workspace hygiene

- `git status` before you start. If the tree has changes you did not make, stop
  and report — do not stage, revert, or build over them.
- Work on `change-0001-safety-traceability-linter`, branched from `main`. Never
  commit to `main` and never `git push origin main`.
- Touch only the files this plan names. If another file must change, say so and
  why before changing it.
- Every change lands via this branch and a PR targeting `main`.
- Note that `origin/pstop` → `main` may still be open for review
  (`OPEN_ITEMS.md` §8). Confirm `docs/safety/` is present on `main` before
  branching; if it is not, stop and raise.

---

## 8. Known traps

1. **Do not fix the safety documents to make the linter pass.** The linter exists
   to surface disagreements, and the disagreements it surfaces are safety
   findings owned by a human. Editing a Status cell, adding a test reference, or
   silently correcting a count converts a finding into a lie. Baseline it and
   report it.
2. **`Partially satisfied` starts with the substring `Satisfied` is false, but
   `Residual-accepted` and `Residual-with-test` both start with `Residual`.**
   Longest-match-first on status normalization, always. A short-match bug scores
   partials as fully verified and inflates the headline number — the single most
   damaging failure mode this tool can have.
3. **Cells contain pipes inside backtick spans.** A naive `line.split("|")`
   shreds rows and the damage is silent: you get a parsed row with shifted columns
   rather than an exception.
4. **`Allocated to` uses three notations in the same document** — plain
   (`F-R-02`), range (`F-R-01..05`), and slash (`F-M-03/04`). All three appear in
   §2's SR-SYS table alone.
5. **A `NO TEST` marker can coexist with real test references in one cell.**
   `SR-SYS-01`, `SR-SYS-02`, `SR-SYS-05`, `SR-SYS-08`, `SR-SYS-09` and `SR-R-13`
   all name real tests and then state that a specific leg has none. Treating the
   marker as "this row has no tests" mis-scores six rows.
6. **`OPEN_ITEMS.md` and `TRACEABILITY.md` disagree on the requirement count and
   the coverage fractions.** They were written at different dates. Neither is
   authoritative for this change; the parsed tables are.
7. **The markdown pre-commit hook rewrites `-` list markers to `+`.**
   `OPEN_ITEMS.md` §8 records that repo-wide normalization is deliberately
   deferred and that per-file application creates inconsistency. If the hook
   rewrites list markers across `TRACEABILITY.md`, revert and report — the
   generated block must not drag a repo-wide reformat in with it.
8. **`pstop_c/` is excluded from pre-commit and is on a separate upstream track.**
   Test paths under it are checked for existence and never opened, never
   formatted, never modified.
9. **The five functions with no SR are real, not parser bugs.** `F-R-08` and
   `F-R-10` are declared non-safety; `F-H-04`, `F-M-01` and `F-M-06` are genuine
   holes recorded in `TRACEABILITY.md` §5. All five belong in the baseline with
   that distinction preserved.
10. **Line numbers in `file:line` citations drift by design.**
    `SAFETY_REQUIREMENTS.md` §1 says so explicitly. A linter that asserts on them
    fails on every unrelated commit and will be disabled within a week.
