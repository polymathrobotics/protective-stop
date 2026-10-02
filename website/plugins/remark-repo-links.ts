// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

import fs from 'fs';
import path from 'path';

// Rewrites markdown links that name a real file or directory outside the docs directory to a GitHub link.
// Root-absolute links resolve from the repository root (e.g. [hardware](/hardware/README.md)),
// relative links resolve from the linking file (e.g. [hardware](../hardware/README.md)).
// Links that don't match a repo path, or that stay inside docs/, are left to Docusaurus.

const REPO_ROOT = path.resolve(__dirname, '../..');
const DOCS_ROOT = path.join(REPO_ROOT, 'docs');
const GITHUB_BASE = 'https://github.com/polymathrobotics/protective-stop';

interface LinkNode {
  type: string;
  url?: string;
  children?: LinkNode[];
}

function visitLinks(node: LinkNode, fn: (link: LinkNode) => void): void {
  if (node.type === 'link') {
    fn(node);
  }
  for (const child of node.children ?? []) {
    visitLinks(child, fn);
  }
}

function isInside(parent: string, child: string): boolean {
  const rel = path.relative(parent, child);
  return !rel.startsWith('..') && !path.isAbsolute(rel);
}

export default function remarkRepoLinks() {
  return (tree: LinkNode, file: {path?: string}) => {
    visitLinks(tree, (link) => {
      if (!link.url || /^[a-z][a-z0-9+.-]*:|^[#?]/i.test(link.url)) {
        return;
      }
      // Preserve any trailing anchor/query on the link.
      const [pathPart, ...rest] = link.url.split(/(?=[#?])/);
      const suffix = rest.join('');

      let fsPath: string;
      if (pathPart.startsWith('/')) {
        fsPath = path.join(REPO_ROOT, pathPart);
      } else if (file.path) {
        fsPath = path.resolve(path.dirname(file.path), pathPart);
      } else {
        return;
      }
      if (!isInside(REPO_ROOT, fsPath) || !fs.existsSync(fsPath)) {
        return;
      }
      if (isInside(DOCS_ROOT, fsPath)) {
        return;
      }
      const repoPath = path.relative(REPO_ROOT, fsPath);
      const viewType = fs.statSync(fsPath).isDirectory() ? 'tree' : 'blob';
      link.url = `${GITHUB_BASE}/${viewType}/main/${repoPath}${suffix}`;
    });
  };
}
