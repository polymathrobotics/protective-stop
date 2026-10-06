# AGENTS.md - Protective Stop

## Docs Page Naming

Every page under `docs/` is a tab on the published website, so titles and filenames follow one standard.

**Title**: Title Case with spaces, set as `title:` in the front matter.
The first H1 matches `title:` exactly.
Acronyms stay uppercase (`FMEA`, `HIL`, `USB-NCM`).

**Filename**: the title in lowercase with underscores, ending in `.md` (`Secure Boot Guide` is `secure_boot_guide.md`).
`index.md` and `README.md` keep their names.

**Dates**: never in a title or filename.
State the date in an italic line under the H1: `*Date: 2026-07-21.*`.
Write ranges and times of day out (`*Date: 2026-07-20 to 2026-07-21.*`).

**Versions**: not in titles either; state them in the page body.

**Links**: reference pages by their filename (`[Quickstart](quickstart.md)`).
When renaming a page, update every link to it, including mentions in `firmware/` and `tools/`.

**Verify**: `npm run build` in `website/` reports broken links.
