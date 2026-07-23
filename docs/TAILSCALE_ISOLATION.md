# Isolating the pstop fleet on Tailscale — advice (2026-07-21)

## The problem this solves

Every embedded remote today pulls the tailnet's **full** peer map (was
capped at 16, now 128 of ~343). Each peer's magicsock DISCO-probes ours;
that probe/crypto surface is the embedded devices' main scaling risk (see
`PEER_SCALING_DESIGN.md`). We want remotes + machines to mostly see **only
each other** (plus their control-plane/OTA needs), both to cut that load
and for safety isolation.

## The key finding: ACLs prune what a node SEES, not just what it reaches

Tailscale implements **netmap trimming** — the policy determines which
devices appear in a node's peer list at all, not merely which it can
connect to. Tightening the policy so a remote may only reach its
machine(s) genuinely **shrinks the peer list that remote is told about**,
which shrinks its magicsock state and DISCO handshake/crypto surface. So
the tailnet policy is the strongest lever on embedded load — the on-device
128-peer budgets (already shipped) are the safety net for untrimmed or
transitional tailnets, not the primary fix.

Two rules that shape the design:
1. **Visibility is bidirectional off a single grant.** A `remote → machine`
   grant makes *both* ends see each other; no reverse grant needed for
   visibility.
2. **A node's netmap also includes everything allowed to reach IT.** So to
   keep a remote's peer list minimal, keep the set of nodes granted access
   *toward* remotes tiny (ideally just one admin/OTA box).

Load caveat (validate on-device): DISCO heartbeats are traffic-aware and
relax on idle/stable paths, so trimming removes idle-peer state and the
universe of *potential* handshakes rather than a fixed per-peer cost — the
reduction is real but not strictly linear. Direct low-latency paths are
unaffected: a grant permits a flow but path selection still tries P2P
first, DERP only as fallback. Hiding peers shrinks discovery, not path
quality.

## Recommended design (zero cost, works on every plan)

**Grant-based netmap pruning on the shared tailnet**, using tags. This
tailnet already has a `tag:pstop` convention (7 nodes today) — formalize
it into role tags.

- Tag every fleet node: `tag:pstop-remote`, `tag:pstop-machine`,
  `tag:pstop-admin` (the OTA/admin box). Tagging removes user-ownership
  **and disables node-key expiry** — which eliminates the periodic
  re-auth failure mode on headless remotes. Register each with a
  **persistent, tagged auth key** (unattended, non-expiring).
- Grant only the flows needed; grant *nothing else* toward the remotes.
  Deny-by-default + netmap trimming then removes all general user devices
  from every remote's peer list automatically.

### Example policy (tailnet policy file, modern `grants` syntax)

`grants` is the current GA syntax (the legacy `acls` stanza still works
but is frozen — no new features). Replace `PSTOP_PORT` with the heartbeat
UDP port; the admin grant uses TCP 80 for the HTTP admin/OTA surface.

```jsonc
{
  "groups": { "group:pstop-admins": ["ilia@polymathrobotics.com"] },

  "tagOwners": {
    "tag:pstop-remote":  ["group:pstop-admins"],
    "tag:pstop-machine": ["group:pstop-admins"],
    "tag:pstop-admin":   ["group:pstop-admins"]
  },

  "grants": [
    // pstop link. Remotes initiate to machines; stateful replies are auto-
    // allowed. This single grant ALSO makes remote<->machine mutually
    // visible (and hides them from everything not granted).
    { "src": ["tag:pstop-remote"],  "dst": ["tag:pstop-machine"], "ip": ["udp:PSTOP_PORT"] },

    // Add ONLY if a machine must independently initiate to a remote
    // (omit for a purely remote-initiated protocol):
    { "src": ["tag:pstop-machine"], "dst": ["tag:pstop-remote"],  "ip": ["udp:PSTOP_PORT"] },

    // OTA / HTTP admin. This box appears in every remote's netmap
    // (inbound-permitted), so keep tag:pstop-admin to as few nodes as
    // possible to keep remote peer lists minimal.
    { "src": ["tag:pstop-admin"],   "dst": ["tag:pstop-remote","tag:pstop-machine"], "ip": ["tcp:80"] }
  ]
}
```

## When to go further: a separate tailnet

ACL isolation is software-enforced least-privilege distributed by the
control plane — strong, but not an air-gap, and Tailscale does not market
it as an absolute boundary. **If the safety case needs a boundary that is
provably independent of main-tailnet policy correctness, use a separate
tailnet** for the fleet (either the built-in multiple-tailnets feature —
currently alpha/sales-gated, same IdP — or a fully separate org account).
It also solves the DISCO load cleanly (the fleet only ever sees the
fleet). Cost is heavier management/keys. For day-to-day least-privilege +
the load fix, the shared-tailnet grants above are the right default;
escalate to a separate tailnet only for a defensible safety claim.

## What is NOT the tool

Subnet routers *extend* connectivity into non-Tailscale networks — they
are not a peer-isolation mechanism; don't conflate them with segmentation.

## Plan-gating / verify in the admin console

- Free on every plan: grants, tags/`tagOwners`, autogroups, auth keys,
  key-expiry-disabled-for-tags. There is **no separate "isolation" feature
  to buy** — it is how the policy file is written.
- Gated: network **flow logs** (Premium/Enterprise) — worth having to
  *audit* that no unexpected peer relationships exist. Multiple tailnets:
  alpha/sales.
- After deploying: `tailscale status` on a remote (or `/state.json` peer
  count on the chip) should show its peer list shrank to just its
  machine(s) + admin box — empirically confirm the trim.

## How this interacts with the firmware

- With a trimmed netmap, a remote pulls a handful of peers, not 128 — the
  peer-scaling work becomes headroom rather than a daily constraint.
- Keep the on-device budgets and the 128 cap: they protect during
  onboarding (before tags/policy apply), on a misconfigured policy, and on
  any tailnet without trimming.
- The `priority_peer_ip` (the machine) stays the latency-protected peer
  regardless of netmap size.

Sources: Tailscale device-visibility / netmap trimming, grants-vs-acls,
tags & key-expiry, connection-types docs (URLs in the research record).
