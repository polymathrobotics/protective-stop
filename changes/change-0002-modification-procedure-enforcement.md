# change-0002 — Modification procedure in-repo, and automated change-record enforcement

**Implements:** The Protective-Stop Modification Procedure, currently held in Notion
and migrated to the repository by this change. Authority for the safety argument
remains `docs/safety/`.
**Original PR branch:** `change-0002-modification-procedure-enforcement`, preserved
as PR #120 recovery and review history.
**Current replacement branch:** `change-0002-process-foundation`
**Base:** current `main` at `6ec5c03`
**Depends on:** change-0001 (safety traceability linter). Piece 6 consumes
`tools/safety_lint`. Pieces 1–5 and 7 do not.
**Status:** sequential replacement delivery in progress; this branch delivers only
the process foundation and repository intake artifacts.
**Safety class:** B — process governance, assurance tooling, and CI guards only;
no runtime safety-path or wire-format change.
**Authorization:** one authorizer approved implementation with the exact instruction
`proceed` on 2026-09-11.
**Sequential delivery map (not stacked):**

1. `change-0002-process-foundation` — this PR, process docs and intake.
2. `change-0002-wire-break` — created from updated `main` only after #1 merges;
   always-enforcing wire guard.
3. `change-0002-change-control-warn` — created after #2 merges; warn-mode record
   checker and mode file.
4. `change-0002-coverage-delta` — created after #3 merges; deterministic advisory
   coverage comments.

Original PR #120 remains preserved as recovery and review history and will close only
after all replacement PRs exist.
**Security contact authorization:** on 2026-09-11 the authorizer confirmed
`security@polymathrobotics.com` as the private reporting route, authorizing removal
of its placeholder warning in `SECURITY.md` and use by the issue chooser.

Read `changes/change-0001-safety-traceability-linter.md` §3 and §8 before starting —
the integration facts and traps there apply here unchanged. Read the four Notion
pages named in Piece 1 in full before migrating them; they are the source text and
this document is the work breakdown. Where the two disagree, the Notion pages win for
*content* and this document wins for *destination and format*.

**Everything this change adds is inert until switched on.** A single file sets warn
or enforce mode. Land it in warn, read what it would have blocked over two or three
weeks of real changes, then flip. One exception, stated in Piece 5 and non-negotiable:
the wire-break check enforces from day one.

---

## 1. What this change delivers

The modification procedure and its three record templates move from Notion into the
repository. The Change Request becomes a real GitHub issue form. Labels, CODEOWNERS
and branch protection give the procedure's approval rules a mechanism. A CI job checks
that each PR has an authorized change request with a completed impact analysis behind
it, that the class label matches what the diff actually touches, and that the tests the
impact analysis named were the tests that ran. A separate check refuses an unannounced
wire-format change. The Claude review bot posts an advisory comment on requirement
coverage; it gates nothing and never writes to a safety document.

**Settled decision 1 — the mode switch is a file, not a repository variable.** Flipping
enforcement is itself a change to how the safety process operates and belongs in the
git history with an author and a reviewer.

**Settled decision 2 — CI enforces that an artifact exists, is complete, and arrived in
the right order. It never judges whether the artifact is correct.** Classification
accuracy, impact-analysis quality, and verification sufficiency are human judgements.
The bot may advise on all three; it decides none of them.

**Settled decision 3 — the bot's output never reaches a certification number.** The
coverage figure in `docs/safety/TRACEABILITY.md` comes only from the deterministic
linter. A model's judgement is not reproducible, and wiring one into a safety gate
would oblige a tool-qualification argument under IEC 61508-3 §7.4.4 that nothing else
in this change requires.

---

## 2. Scope

### IN scope

1. `docs/process/MODIFICATION_PROCEDURE.md` plus three templates, migrated from Notion.
2. `.github/ISSUE_TEMPLATE/change-request.yml` — the Change Request as a GitHub issue
   form.
3. Label set, `CODEOWNERS`, and a written branch-protection configuration.
4. `docs/process/enforcement-mode` — the one-line file that sets warn or enforce.
5. `.github/workflows/change-control.yml` and `tools/change_control/` — the CR, impact
   analysis, classification-floor and IA-versus-CI checks.
6. `.github/workflows/wire-break.yml` and `scripts/check_wire_format.sh` — enforcing
   from day one.
7. `.github/workflows/coverage-delta.yml` — runs the change-0001 linter on base and
   head and posts the delta.
8. Claude review bot configuration for the advisory sufficiency comment.
9. `docs/process/EXTERNAL_CONTRIBUTIONS.md` — the path for a PR that arrives without a
   change request.
10. `SECURITY.md` — remove the now-resolved placeholder warning for the confirmed
    private reporting address.

### OUT of scope — do NOT build

- **Any edit to `docs/safety/`.** Same prohibition as change-0001 and for the same
  reason. If a check fails on safety-document content, report it; do not fix it.
- **Deleting or archiving the Notion pages.** Migration is copy-then-verify. Raj
  retires them once exida has the URL map from Piece 1e.
- Enabling enforce mode. This change lands in warn.
- Enabling branch protection. Piece 4 writes the configuration down and the repo admin
  applies it out of band; an agent does not change repository settings.
- The Gate 2 release-qualification workflow. That is change-0003.
- Any change to `tools/safety_lint/` beyond calling it. If it needs a new output mode,
  stop and raise rather than editing it here.
- `pstop_c/` contents. Piece 5 reads its headers and never modifies them.
- Any automated status transition in `TRACEABILITY.md`.

---

## 3. Integration facts (do not rediscover these)

| Fact | Value |
|---|---|
| Repository | `polymathrobotics/protective-stop`, public |
| Existing workflows | `pre-commit.yml`, `firmware-build.yml`, `host-check.yml`, `ros2_build.yml`, `pstop_c_build.yml`, `pstop_c_coverage.yml`, `coverage.yml` |
| Existing `.github` contents | the seven workflows above plus `dependabot.yml`. No `CODEOWNERS`, no `ISSUE_TEMPLATE`, no `PULL_REQUEST_TEMPLATE` |
| Checkout action version in use | `actions/checkout@v7` |
| Authorizers | Ilia Baranov, Raj, David Tarazi — all three are code owners of everything and all three may authorize |
| Protocol version constant | `pstop_c/pstop/include/pstop/config.h` — `PSTOP_VERSION 0x02U`, `PSTOP_MESSAGE_SIZE 48U` |
| Wire-behavior files | Eight public headers under `pstop_c/pstop/include/pstop/` — `config.h`, `constants.h`, `protocol.h`, `protocol_data.h`, `pstop_msg.h`, `checksum.h`, `device_id.h`, `endian.h` — plus `pstop_c/pstop/src/pstop/pstop_msg.c`, `pstop_c/pstop/src/pstop/checksum.c`, and `pstop_c/pstop/src/pstop/endian.c` |
| `pstop_c` is vendored in-tree | A directory, NOT an ESP-IDF managed component. It does not appear in `firmware/dependencies.lock`. A "version bump" is a directory update, so the check compares header content, not a lockfile line |
| Linter entry point (change-0001) | `python3 -m tools.safety_lint --json` |
| Pre-commit exclusions | `pstop_c/`, `ros2/`, `archive/`, vendored wireguard and x25519, `hardware/` binaries. `tools/`, `docs/` and `.github/` are in scope |
| Required file header | SPDX two-line, Apache-2.0. Copy `tools/test_config_floor.py` or `scripts/check_estop_diversity.sh` |
| Guard-script convention | `set -uo pipefail`, repo-root `cd`, header comment naming the SR and DU it guards, exit `0` pass / `1` check failed / `2` cannot run |
| Notion — Modification Procedure | `3d6c0b1ac5fa8105b456e862e939f051` |
| Notion — Impact Analysis Template | `3d6c0b1ac5fa81838a09f22388095c33` |
| Notion — Change Request Template | `3d6c0b1ac5fa8167927af2e753be67c5` |
| Notion — Release Record Template | `3d6c0b1ac5fa81708a8cff41150d2b86` |

### Two things the plan author could not determine

This plan was written without authenticated GitHub access. **Piece 0 discovers both
before anything else is built.** Do not assume either answer.

1. **Which branch-protection and CODEOWNERS features this repository actually has.**
   Public repositories get most of them free, but required-reviewer behaviour and
   rulesets differ by plan.
2. **How the Claude review bot is installed and configured.** There is no
   `.github/claude.yml` and no reference to `claude` or `anthropic` anywhere under
    `.github/`, so it is an org-level GitHub App or configured outside the repo.

### Piece 0 discovery result — 2026-09-11

1. The repository is public on GitHub Team. `main` reports `protected: true`, but
   this non-admin account receives 404 from the classic protection-details endpoint;
   repository and branch rules endpoints expose no applicable rules. Existing
   protection details therefore remain unknown and Piece 4 documents the intended
   settings without applying them.
2. Required reviews from CODEOWNERS are available for this public Team repository.
   Verified write-capable handles are `@iliabaranov`,
   `@rajasimman-madhivanan`, and `@davidt315`. The originally suggested `@dtarazi`
   resolves but has read-only repository access and must not be used as a code owner.
3. Check-run metadata identifies GitHub App `claude`, owned by `anthropics`, as the
   review bot. Neither this repository nor the organization `.github` repository
   contains visible Claude configuration, and installation metadata requires
   unavailable `admin:org` access. Per Piece 8e, build deterministic coverage delta
   only; do not build or guess bot configuration.
4. Existing labels are only GitHub's defaults: `bug`, `dependencies`,
   `documentation`, `duplicate`, `enhancement`, `github_actions`, `good first issue`,
   `help wanted`, `invalid`, `question`, and `wontfix`. None collides with Piece 3.

---

## 4. Pieces, in dependency order

Piece 0 is discovery and must complete first. Pieces 1–4 add no CI surface. Piece 5 is
the only thing in this change that can block a merge on the day it lands.

---

### Piece 0 — Discovery

**Files:** none. Output is a report.

**0a.** Run, and record the output verbatim:

```
gh api repos/polymathrobotics/protective-stop --jq '{visibility,default_branch,allow_auto_merge,delete_branch_on_merge}'
gh api repos/polymathrobotics/protective-stop/branches/main/protection 2>&1 | head -40
gh api repos/polymathrobotics/protective-stop/rulesets 2>&1 | head -40
gh api orgs/polymathrobotics/installations --jq '.installations[].app_slug' 2>&1
gh api repos/polymathrobotics/protective-stop/labels --jq '.[].name'
```

**0b. Report four answers before writing any code:**
- Is branch protection already configured on `main`, and with what?
- Are required reviewers via CODEOWNERS available on this plan?
- Which app provides the Claude review bot, and where does it read configuration from?
  If you cannot determine this from the API, say so and check for an org-level
  `.github` repository holding shared workflow or app configuration.
- Which labels already exist, so Piece 3 adds rather than collides.

**0c.** If required reviewers are unavailable on this plan, **stop and raise.** The
two-authorizer rule for Class C then has no mechanism and Piece 4 needs redesigning as
a CI check against PR review state rather than a CODEOWNERS rule. Do not silently
substitute one for the other.

---

### Piece 1 — Migrate the procedure and templates out of Notion

**Files:** `docs/process/MODIFICATION_PROCEDURE.md`,
`docs/process/templates/IMPACT_ANALYSIS.md`,
`docs/process/templates/RELEASE_RECORD.md`,
`docs/process/NOTION_MIGRATION_MAP.md`.

**1a.** Read all four Notion pages in full using the Notion MCP tools. Bare page UUIDs
work; full URLs are less reliable.

**1b. Migrate content unchanged.** Preserve section numbering, the gate model, the
clause index, the declared-gaps table and the callout warning that the procedure has
not yet been exercised. **Do not improve the text.** Three corrections are authorized
and no others:
- Rewrite Notion-relative references as repo-relative links.
- Update the Roles section for three authorizers. The deviation clause covering
  "where team size makes this impossible" stays in the document and is annotated as
  not currently applicable.
- Where the procedure names a template, link the migrated file rather than the Notion
  page.

**1c. The Change Request template does NOT become a markdown file.** It becomes the
issue form in Piece 2. `docs/process/` holds the procedure, the impact analysis
template and the release record template only.

**1d. Convert Notion callouts and tables to plain markdown** that the `polymath-markdown`
pre-commit hook accepts. Watch the list-marker rewrite recorded in
`docs/safety/OPEN_ITEMS.md` §8 — if the hook reformats files outside `docs/process/`,
revert and report.

**1e. `NOTION_MIGRATION_MAP.md`** — a table mapping each Notion page URL to its new
repository path. Required for the certification evidence chain: the assessment
workbook cites Notion URLs as evidence and those citations need a forwarding address.
Include the four pages in this change and note that other Notion citations
(`FSM-1`, `FSM-11`, `CM-2`, `CM-5`, `VTP-8`) are not yet migrated.

**Tests:** none — this piece is content migration. Verification is Piece 1f.

**1f. Verify the migration by diff, not by reading.** For each of the three migrated
pages, produce a normalized text comparison between the Notion source and the
committed markdown, and report any sentence present in one and absent from the other.
Attach the comparison to the PR. A migration that silently drops a paragraph is the
failure mode here, and re-reading your own output will not catch it.

---

### Piece 2 — Change Request as a GitHub issue form

**Files:** `.github/ISSUE_TEMPLATE/change-request.yml`,
`.github/ISSUE_TEMPLATE/config.yml`, `.github/PULL_REQUEST_TEMPLATE.md`.

**2a.** Build `change-request.yml` as a GitHub issue form carrying every field from the
Notion Change Request Template. Fields marked required in the form:
- Reason for the change
- **Hazards that may be affected** — required, with the placeholder making clear that
  "none identified, because…" is a valid answer and blank is not
- Description, covering both hardware and software
- Baseline affected — firmware, host and hardware versions
- Proposed class — dropdown A / B / C, **defaulting to C**

Non-required at creation because they are completed later: impact analysis link,
authorization, implementation, gate evidence, review, deviations, release record.

**2b. Default to C.** A requester who does not know the class produces a Class C
request, which gets reviewed. The reverse default produces silent under-classification.

**2c. `config.yml`** — `blank_issues_enabled: false` is **not** set. A public repository
needs a path for bug reports and questions that are not change requests. Add a contact
link to `SECURITY.md` for safety-defect reports.

**Implementation blocker (2026-09-11):** `SECURITY.md` explicitly marks
`security@polymathrobotics.com` as a placeholder requiring confirmation, and GitHub
issue-form contact links require a URL rather than a repository-relative file. No
truthful private reporting destination is available in the repository, so
`.github/ISSUE_TEMPLATE/config.yml` is intentionally not created. An authorizer must
confirm a private destination before Piece 2c can be completed; a public issue route
must not be presented as suitable for safety-defect reports.

**2d. PR template** — a short form with: linked change request (`Closes #NN` or
`Refs #NN`), class, impact analysis link, and a checklist mirroring the definition of
done. Kept short; a long PR template gets ticked without reading.

**2e. The form is the single source of truth for its own field list.** Piece 6's
checker reads the required-field names from this YAML rather than holding a private
copy. A checker with its own idea of the format is a second source of truth.

**Tests:**
- `test_issue_form_is_valid_yaml_and_parses`
- `test_required_fields_present` — the five fields in 2a are marked required.
- `test_class_dropdown_defaults_to_c` — drift-verify: change the default to A, confirm
  the test fails, revert.

---

### Piece 3 — Labels

**Files:** `tools/change_control/labels.json`, `scripts/sync_labels.sh`.

**3a.** Define the label set as data, and a script that creates or updates them via
`gh label`. Do not create them by hand — the set has to be reproducible.

| Label | Purpose |
|---|---|
| `change-request` | Marks an issue as a CR. Applied automatically by the issue form |
| `class-a`, `class-b`, `class-c` | Safety classification |
| `emergency` | Compressed-timeline path |
| `safety-defect` | Defect in a released baseline affecting safety |
| `wire-break` | Applied automatically by Piece 5 |
| `needs-change-request` | External PR awaiting a maintainer-opened CR |
| `status:proposed`, `status:under-analysis`, `status:authorized`, `status:rejected`, `status:in-implementation`, `status:in-verification`, `status:merged`, `status:released` | CR lifecycle |

**3b.** Use whatever labels Piece 0b found already present rather than creating a
near-duplicate. Report any collision instead of resolving it yourself.

**Tests:** `test_labels_json_has_no_duplicate_names`;
`test_every_label_referenced_in_the_procedure_exists_in_labels_json` — parse the
migrated procedure for backticked label names and assert each is defined. This catches
the procedure and the tooling drifting apart.

---

### Piece 4 — CODEOWNERS and branch protection

**Files:** `.github/CODEOWNERS`, `docs/process/BRANCH_PROTECTION.md`.

**4a. CODEOWNERS — everyone owns everything**, per the Director's instruction:

```
*  @iliabaranov @rajasimman-madhivanan @davidt315
```

Resolve the real GitHub handles with `gh api users/...` or from the repo's contributor
list; do not guess them. A CODEOWNERS file with an unresolvable handle silently matches
nothing, which is the worst failure mode available here — it looks configured and
enforces nothing. **Verify each handle resolves and report the verification.**

**4b. `BRANCH_PROTECTION.md`** — the intended configuration, written down for a repo
admin to apply. It is documentation in this change, not an applied setting.

| Setting | Value | Rationale |
|---|---|---|
| Require a pull request before merging | on | The procedure's merge gate |
| Required approvals | 1 | Class C's second approval comes from the Piece 6 check, so that the rule tracks the class rather than applying to every change |
| Dismiss stale approvals on new commits | on | An approval is of a diff, not of a branch |
| Require review from Code Owners | on | Makes one of the three authorizers a required reviewer |
| Require status checks | `pre-commit`, `host-check`, `firmware-build`, `ros2_build`, `pstop_c_build`, `wire-break` | Gate 0. `change-control` joins this list when enforce mode is switched on, not before |
| Require branches up to date | on | Two changes each green alone can be red together |
| Allow force pushes | off | Public repo; published history stands |
| Allow deletions | off | |

**4c. Self-approval.** GitHub already refuses a review from the PR author, so
"authorizer ≠ implementer" holds for the first approval without extra machinery. The
second Class C approval is checked in Piece 6, which must assert two distinct
approving reviewers neither of whom is the author.

**Tests:** `test_codeowners_parses_and_covers_root`;
`test_codeowners_handles_resolve` — network-gated, skipped without `GH_TOKEN`, and its
skip must be visible in the output rather than silent.

---

### Piece 5 — Wire-break check (enforces immediately)

**Files:** `scripts/check_wire_format.sh`, `.github/workflows/wire-break.yml`,
`tools/change_control/wire_format.sha256`.

**5a. What this guards.** `pstop_c` is vendored in-tree. A change to the message
layout alters the checksum computed over it, and a remote and a machine on different
layouts reject each other's messages entirely. The machine's heartbeat times out and
it fail-safes to STOP — safe, and permanently stopped until both ends are updated
together. The build stays green throughout. This check makes that impossible to ship
unannounced.

**5b. The signature.** A SHA-256 over the normalized content of the wire-behavior files
listed in §3, plus the literal values of `PSTOP_VERSION` and `PSTOP_MESSAGE_SIZE`.
Normalize by stripping comments and collapsing whitespace so a comment edit does not
trip it. Store the expected value in `wire_format.sha256` with a comment recording the
`PSTOP_VERSION` it corresponds to.

**5c. Behaviour on mismatch.** Fail the PR with a message naming which files changed
and stating that remote and machine must be released and deployed together. Apply the
`wire-break` label and require the `class-c` label. The fix path is to update
`wire_format.sha256` in the same PR, which makes the change explicit in the diff and
reviewable — the point is not to prevent wire changes, it is to prevent *silent* ones.

**5d. This check ignores `docs/process/enforcement-mode` and always enforces.** It has
no judgement in it and no false positives — the watched files either changed or they
did not — and the failure mode is a field outage rather than a process complaint. Hardcode
this; do not make it configurable.

**Tests:**
- `test_signature_stable_across_comment_only_change` — add comments to watched headers
  and implementation files in a scratch copy, assert the signature is unchanged.
- `test_signature_changes_on_field_addition` — add a field to a struct in a scratch
  copy, assert it changes.
- Mutate each watched implementation file in a scratch copy: reorder adjacent field
  writes in `pstop_msg.c`, change the CRC polynomial in `checksum.c`, and change a
  byte-order operation in `endian.c`; each file hash and the aggregate must change.
- `test_signature_changes_on_message_size_change`
- `test_check_exits_one_on_mismatch_and_names_the_headers`
- Drift-verify: corrupt `wire_format.sha256`, confirm CI fails, restore. Paste the run.

---

### Piece 6 — Change-control checks

**Files:** `tools/change_control/__main__.py`, `tools/change_control/checks.py`,
`.github/workflows/change-control.yml`, `docs/process/enforcement-mode`.

Stdlib only, same constraint as change-0001. GitHub state comes from `gh api` called
via `subprocess`, not from a Python GitHub library.

**6a. The mode file.** `docs/process/enforcement-mode` contains exactly one word,
`warn` or `enforce`. Land it as `warn`. Every check prints the active mode in its
output so no result is ever ambiguous about which mode produced it. In `warn`, findings
print and the job exits 0. In `enforce`, findings print and the job exits 1.

**6b — E1. A change request exists and is authorized.** The PR body links an issue
carrying `change-request`. That issue has the `status:authorized` label and an
authorization comment from one of the three authorizers. A PR labelled
`needs-change-request` is exempt and reported as pending — see Piece 7.

**6c — E2. An impact analysis exists and is complete.** The linked CR has a comment
containing the impact analysis. Every section heading from
`docs/process/templates/IMPACT_ANALYSIS.md` is present **and has non-empty content
beneath it**. Presence alone is not completeness — an IA with every heading and "N/A"
under each passes a heading check perfectly, which is why this check reads content.

**This check cannot tell whether the content is true.** It catches wholesale omission
only. Say so in the output so nobody reads a green E2 as an endorsement.

**6d — E3. Cited requirement IDs are real.** Every `SR-<area>-<nn>` in the impact
analysis parses and exists in `docs/safety/SAFETY_REQUIREMENTS.md`. Reuse
`tools.safety_lint.parse_srs` rather than writing a second parser.

**6e — E4. Classification floor.** Path and content rules that force a minimum class
regardless of the applied label:

| Trigger | Minimum |
|---|---|
| Any file under `pstop_c/` | C |
| `firmware/main/main.c` or `machn/main/main.c` | C |
| Wire signature changed (Piece 5) | C |
| Any file under `docs/safety/` | C |
| Any file under `components/` or `common/` | B |
| `sdkconfig.defaults` in `firmware/` or `machn/` | B |
| Any `.github/workflows/**` or `scripts/**` guard | B |

A label below the floor is a finding. **A label above the floor is never a finding** —
over-classification is always allowed. The floor cannot catch a Class C change in an
unlisted path; that is what the bot's advisory comment in Piece 8 is for.

**6f — E5. Two distinct approvals for Class C.** Two approving reviews from two
different accounts, neither of whom is the PR author, and both among the three
authorizers. Read review state with `gh api`.

**6g — E6. The impact analysis named tests; those tests ran.** Parse test names and
paths from the IA's verification-plan section. Check each against the workflow runs on
the head commit. A test named in the IA with no corresponding run is a finding.

**This is the strongest check in the change.** It is the only one that catches the
plan and the execution diverging, which is the failure nobody notices today. Expect it
to be noisy at first — that noise is information about how IAs are actually written,
and it is the main thing warn mode exists to surface.

**6h — E7. Emergency path.** A PR labelled `emergency` requires both authorizer
approvals regardless of class, and the CR must carry a short-form IA. Report the
five-working-day retrospective deadline in the output; do not attempt to enforce a
deadline in CI.

**6i. Output.** A single PR comment, updated in place rather than appended, listing
each check with pass / fail / not-applicable, the active mode, and a one-line
explanation per failure. Never more than one comment per PR.

**Tests:** one per check against synthetic fixtures under
`tools/change_control/fixtures/`, not against live GitHub. Plus:
- `test_warn_mode_exits_zero_with_findings`
- `test_enforce_mode_exits_one_with_findings`
- `test_mode_file_rejects_unknown_value` — exit 2, not a silent default to warn.
- `test_e4_over_classification_is_not_a_finding`
- `test_e2_rejects_heading_present_but_empty` — drift-verify this one; it is the
  difference between a real check and a decorative one.

---

### Piece 7 — External contributions

**Files:** `docs/process/EXTERNAL_CONTRIBUTIONS.md`, edit to `CONTRIBUTING.md`.

**7a.** The path, per the Director's ruling: a PR arriving without a change request is
labelled `needs-change-request` and review does not begin. A maintainer opens the CR on
the contributor's behalf, classifies it, and completes the impact analysis. The
contributor is not asked to write one — they cannot, since they do not have the safety
context.

**7b.** Once the CR exists and is authorized, the label is removed and the normal
checks apply.

**7c.** Add a short section to `CONTRIBUTING.md` pointing at this document, framed so a
contributor understands the CR is maintainer work rather than a barrier to them.

**Tests:** `test_needs_change_request_label_exempts_e1` — a PR with that label produces
a pending result on E1, not a failure.

---

### Piece 8 — Advisory review

**Files:** `.github/workflows/coverage-delta.yml`, Claude bot configuration at
whatever path Piece 0 established.

**8a. Coverage delta is deterministic and comes from the linter.** Run
`python3 -m tools.safety_lint --json` at the merge base and at the head, diff the
results, post: coverage before and after, any requirement that gained or lost a
citation, any newly unresolvable citation. Mechanical, reproducible, no model
involved.

**8b. Sufficiency is advisory and comes from the bot.** Its input: the diff, the
coverage delta from 8a, and the full text of every requirement the change touched. Its
question: *are the tests in this change adequate for what changed?*

**8c. Three instructions the bot needs, because these are its predictable failure
modes:**
- **Flat coverage is not a pass.** A change can add a whole code path under an
  already-cited requirement and move no number at all. Reason about the change, not
  the metric.
- **Report what you could not assess.** A comment with an empty "could not confirm"
  section means the review was shallow, not that everything is fine.
- **Propose a classification, and flag under-classification the path rules would
  miss.** The Piece 6e floor is mechanical and cannot catch a Class C change in an
  unlisted path.

**8d. Hard boundary.** The bot's output is a PR comment. It never writes to any file
under `docs/safety/`, never proposes a status transition, never feeds a published
number, and blocks nothing. State this in its configuration, not only here.

**8e.** If Piece 0 could not determine how the bot is configured, **build 8a and stop.**
Report what you found and leave 8b–8d unbuilt. The deterministic half is the half that
matters; do not guess at an app's configuration format.

**Tests:** `test_coverage_delta_detects_lost_citation` — integration: in a scratch
clone, delete a cited test, assert the delta names the affected requirement.

---

## 5. Definition of Done

1. **Piece 0's four answers reported before any other piece was built.**
2. **Tests written first** (red → green), driving the real path.
3. **Drift-verify** the three named guards — the class default in 2c, the wire
   signature in 5e, and the empty-section check in 6e. Break it, confirm failure,
   revert, report that you did it.
4. **Run the gates yourself and paste actual output:** each new workflow's run, the
   check suites, and `pre-commit run --all-files`.
5. **`git diff docs/safety/` must be empty.** Paste it. Any change there is a scope
   violation.
6. **The migration comparison from 1f is attached to the PR.**
7. **`docs/process/enforcement-mode` says `warn`.** Confirm explicitly.
8. **The wire-break check is live and enforcing**, with the drift-verify run pasted.
9. **CODEOWNERS handles verified to resolve**, with the verification output.
10. **Two separate sections, not merged:** *"out of scope, confirmed not built"* and
    *"in scope, required, not done"* — the second must be empty.

---

## 6. Acceptance criteria

- **AC-1 — nothing new blocks a merge except the wire check.** Open a trivial
  docs-only PR with no change request. The `change-control` job reports findings and
  passes. The `wire-break` job passes. Paste both.
- **AC-2 — the wire check bites.** In a scratch branch, add a field to a struct in a
  wire-format header. `wire-break` fails, names the header, and applies the label.
  Restore. Paste the run.
- **AC-3 — the wire check does not false-positive.** Add a comment to `protocol.h`.
  The check passes. Paste the run.
- **AC-4 — enforce mode works.** Flip the mode file to `enforce` in a scratch branch,
  re-run against the AC-1 PR, confirm the job now fails with the same findings. Revert
  to `warn`. Paste both runs.
- **AC-5 — the empty-IA check is real.** A fixture IA with every heading present and no
  content under them produces an E2 finding.
- **AC-6 — over-classification is permitted.** A docs-only PR labelled `class-c`
  produces no E4 finding.
- **AC-7 — the IA-versus-CI check works.** A fixture IA naming a test that did not run
  produces an E6 finding naming that test.
- **AC-8 — migration is complete.** The 1f comparison shows no sentence present in
  Notion and absent from the repository.
- **AC-9 — the label set is reproducible.** Run `scripts/sync_labels.sh` twice; the
  second run makes no changes.
- **AC-10 — coverage delta works.** Paste the bot comment from a PR that touches a
  cited test.

---

## 7. Workspace hygiene

- `git status` before you start. Foreign changes: stop and report.
- Branch `change-0002-modification-procedure-enforcement` from `main`. Never commit to
  `main`.
- Touch only the files this plan names.
- **Do not change repository settings.** Branch protection, label creation on the live
  repo beyond `sync_labels.sh`, and app installation are human actions. Write the
  configuration down; do not apply it.
- **Do not modify or delete anything in Notion.**

---

## 8. Known traps

1. **A CODEOWNERS file with an unresolvable handle matches nothing and reports
   nothing.** It looks configured and enforces zero. Verify every handle resolves and
   paste the verification. This is the highest-consequence silent failure in the
   change.
2. **`pstop_c` is vendored in-tree, not a managed component.** It is absent from
   `firmware/dependencies.lock`. A check written against the lockfile will never fire.
   Compare header content.
3. **An impact analysis with every heading and "N/A" under each passes a formatting
   check perfectly.** That is why E2 reads content. A decorative check next to real
   ones is worse than no check, because it lends them its own emptiness.
4. **Warn mode that nobody reads is the same as no mode.** The output of warn mode is
   the deliverable of this change, not a side effect. E6 in particular will be noisy,
   and that noise is the finding.
5. **The markdown pre-commit hook rewrites `-` list markers to `+`**
   (`docs/safety/OPEN_ITEMS.md` §8). If it reformats files outside `docs/process/`,
   revert and report rather than dragging a repo-wide reformat in.
6. **Issue forms are YAML with a strict schema.** An invalid form does not error — it
   silently falls back to a blank issue, and nobody notices the hazards field stopped
   being required. Validate it and assert on the parse.
7. **`docs/safety/` is out of scope, and this change touches the process that governs
   it.** The temptation to fix one small inconsistency while in there is exactly what
   the prohibition exists to stop.
8. **Do not enable enforce mode, and do not enable branch protection.** Both are
   Director decisions after the warn period. An agent that "finishes the job" by
   switching them on has shipped an unreviewed process change.
