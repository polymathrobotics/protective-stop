---
title: API Reference
---

<!-- SPDX-FileCopyrightText: 2026 Polymath Robotics -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# API Reference

The generated C and C++ API reference is published under the **API** tab in the navbar.
It covers the header files (`*.h`, `*.hpp`) in the `INPUT` directories of `docs/developing/Doxyfile`.

## Building locally

Install `doxygen` and `graphviz`, then from `website/`:

```bash
npm run docs:api
npm run start
```

`docs:api` writes to `website/static/api`, which is gitignored.
The configuration is `docs/developing/Doxyfile`.

## Writing documentation

Write `///` Doxygen comments following [Commenting Rules](commenting_rules.md).
The build treats Doxygen warnings as errors, so an undocumented `@param` on a documented function fails CI.
