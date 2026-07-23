# Security Policy

The Polymath Protective Stop is **safety-relevant hardware and firmware**.
A vulnerability here can prevent a robot from stopping when commanded, or
cause it to stop when it should not. We take reports seriously and ask
that you disclose responsibly.

## Reporting a vulnerability

Please report suspected vulnerabilities **privately**. Do not open a
public issue, pull request, or discussion for a security problem.

- Email: **security@polymathrobotics.com**
  _(placeholder — please confirm the correct security contact before this
  policy is published)_

Include, where possible:

- affected component (`firmware/`, `host/`, `components/`, `pstop_c/`) and
  version / commit;
- a description of the issue and its safety impact (e.g. missed STOP,
  spurious arming, heartbeat spoofing, denial of service on the link);
- reproduction steps or a proof of concept;
- your assessment of severity and any suggested mitigation.

If the issue is in the upstream certified `pstop_c/` protocol library, we
will coordinate with the upstream maintainers, since fixes there follow a
separate review and certification path.

## Scope

In scope: the firmware, the host runner, the shared ESP-IDF components,
the test tooling, and the wire protocol / transport as used by this
project.

Out of scope: third-party ESP-IDF and managed components, general Wi-Fi /
Tailscale / network-stack issues not specific to this project, physical
attacks requiring disassembly of a deployed unit, and the WIP hardware
design files in `hardware/`.

## Response expectations

We aim to acknowledge a report within **5 business days** and to provide
an initial assessment within **10 business days**. Timelines for a fix
depend on severity and on whether the certified library is involved. We
will keep you informed of progress and coordinate a disclosure timeline
with you.

## No bug bounty

We do not currently run a bug-bounty program and cannot offer payment for
reports. We are grateful for responsible disclosure and, with your
permission, will credit you when a fix is published.
