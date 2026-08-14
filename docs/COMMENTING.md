<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Commenting rules

Distilled from the Google C++ style guide comment section, adapted to C.
Applies to all new and modified code. Design docs own rationale and history;
git owns changelog; comments own behavior and invariants.

## Rules

1. Comments say **why** or state a **non-obvious invariant** — never restate
   what the code visibly does.
2. **No history or narrative** ("previously X", "fixed after review",
   "from tailscaled") — link a design doc if rationale needs more than
   two lines.
3. Function comments only when name + signature is insufficient; one line
   preferred. Document units, ownership, and error returns — not internals.
4. File headers: SPDX + one or two lines on what the file owns.
5. A comment that has drifted from the code is deleted or fixed on sight.

## Examples

Bad (restates code, narrates rationale):

```c
/* Set SO_RCVTIMEO to the requested timeout. If timeout is 0 (mbedTLS
 * default = "no timeout"), use 10s as a sane default to avoid indefinite
 * blocking on AT sockets. */
```

Good:

```c
/* timeout 0 means "forever" to mbedTLS; cap at 10 s so a dead socket
 * cannot block the I/O task. */
```

Bad (function comment repeats the signature):

```c
/** Custom send for mbedtls BIO. */
static int derp_bio_send(void *ctx, const unsigned char *buf, size_t len)
```

Good: no comment — the name and BIO signature say it all.

Bad (invariant buried in narrative):

```c
/* No mutex needed — single task owns the SSL context exclusively.
 * Matches v1's single-threaded DERP model. */
```

Good:

```c
/* Invariant: only the DERP I/O task touches this SSL context. */
```
