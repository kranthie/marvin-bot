// Copy the repository's Markdown docs into the Starlight content collection, so the repository
// stays the single source of truth and the site never holds a second, drifting copy.
//
// For each source file in PAGES it:
// - takes the first "# " heading as the page title and the first paragraph as its description;
// - rewrites relative links: to another synced page -> that page's URL; to an image or video in
//   the repository -> a copy under public/repo-media/; to any other repository file -> GitHub;
// - turns GitHub alert blocks (> [!WARNING]) into Starlight asides;
// - sets the page's "Edit page" link to the source file on GitHub.
// It also copies the pinned three.js build into the guide (public/guide/vendor/three/).
// Generated files are listed in .gitignore files next to them and are never committed.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const SITE = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const REPO = path.resolve(SITE, '..');
const BASE = '/marvin-bot';
const GITHUB = 'https://github.com/kranthie/marvin-bot';
const CONTENT = path.join(SITE, 'src/content/docs');
const MEDIA = path.join(SITE, 'public/repo-media');

// repository file -> site slug (and an optional title when the file's own heading does not suit)
const PAGES = {
  'README.md': { slug: 'overview', title: 'Overview' },
  'docs/README.md': { slug: 'glossary', title: 'Documentation index and glossary' },
  'docs/plan.md': { slug: 'plan' },
  'docs/roadmap.md': { slug: 'roadmap' },
  'docs/requirements.md': { slug: 'requirements' },
  'docs/decisions.md': { slug: 'decisions' },
  'docs/design/concept.md': { slug: 'design/concept' },
  'docs/design/stairs.md': { slug: 'design/stairs' },
  'docs/design/assembly_wiring.md': { slug: 'design/assembly-wiring' },
  'sim/isaaclab/README.md': { slug: 'simulation/isaac-lab', title: 'Walking and stairs in Isaac Lab' },
  'sim/motion/README.md': { slug: 'simulation/motion-tracking', title: 'Motion tracking pipeline' },
  'docs/skills.md': { slug: 'simulation/skills' },
  'bom/README.md': { slug: 'build/bom', title: 'Bill of materials' },
  'bench/README.md': { slug: 'build/bench', title: 'Actuator bench (phase 2)' },
  'docs/safety.md': { slug: 'build/safety' },
  'CONTRIBUTING.md': { slug: 'project/contributing' },
  'ACKNOWLEDGMENTS.md': { slug: 'project/acknowledgments' },
  'SECURITY.md': { slug: 'project/security' },
  'CODE_OF_CONDUCT.md': { slug: 'project/code-of-conduct' },
  'LICENSES/README.md': { slug: 'project/licences', title: 'Licences' },
};
const MEDIA_EXT = /\.(png|jpe?g|gif|webp|svg|mp4|webm)$/i;
const ALERTS = { NOTE: 'note', TIP: 'tip', IMPORTANT: 'note', WARNING: 'caution', CAUTION: 'danger' };

const pageUrl = (slug, hash = '') => `${BASE}/${slug}/${hash}`;

function rewriteLink(target, fromFile) {
  if (/^([a-z]+:|#|\/\/)/i.test(target)) return target; // absolute URL, anchor or protocol-relative
  const [p, hash = ''] = target.split('#');
  const anchor = hash ? `#${hash}` : '';
  const rel = path.posix.normalize(path.posix.join(path.posix.dirname(fromFile), p));
  if (PAGES[rel]) return pageUrl(PAGES[rel].slug, anchor);
  if (MEDIA_EXT.test(rel) && fs.existsSync(path.join(REPO, rel))) {
    const dest = path.join(MEDIA, rel);
    fs.mkdirSync(path.dirname(dest), { recursive: true });
    fs.copyFileSync(path.join(REPO, rel), dest);
    return `${BASE}/repo-media/${rel}`;
  }
  const isDir = fs.existsSync(path.join(REPO, rel)) && fs.statSync(path.join(REPO, rel)).isDirectory();
  return `${GITHUB}/${isDir ? 'tree' : 'blob'}/main/${rel}${anchor}`;
}

function convertAlerts(md) {
  // > [!WARNING]\n> text\n> more  ->  :::caution\ntext\nmore\n:::
  return md.replace(/^> \[!(\w+)\][ \t]*\n((?:>.*(?:\n|$))*)/gm, (_, kind, body) => {
    const text = body.replace(/^> ?/gm, '').trimEnd();
    return `:::${ALERTS[kind.toUpperCase()] || 'note'}\n${text}\n:::\n`;
  });
}

function plain(s) {
  return s
    .replace(/!\[[^\]]*\]\([^)]*\)/g, '')
    .replace(/\[([^\]]*)\]\([^)]*\)/g, '$1')
    .replace(/[*_`>#]/g, '')
    .replace(/\s+/g, ' ')
    .trim();
}

const yaml = (s) => JSON.stringify(s); // a JSON string is a valid YAML scalar

function sync() {
  fs.rmSync(MEDIA, { recursive: true, force: true });
  const written = [];
  for (const [src, { slug, title }] of Object.entries(PAGES)) {
    const file = path.join(REPO, src);
    if (!fs.existsSync(file)) {
      console.warn(`skipped (missing): ${src}`);
      continue;
    }
    let md = fs.readFileSync(file, 'utf8');
    const h1 = md.match(/^# (.+)$/m);
    const pageTitle = title || (h1 ? h1[1].trim() : slug);
    if (h1) md = md.replace(h1[0], '');
    // status badges belong on GitHub; on the site they would stack up as separate images
    md = md.replace(/^\[?!\[[^\n]*(?:badge\.svg|shields\.io)[^\n]*\n/gm, '');
    md = convertAlerts(md);
    // Markdown links and images, outside code blocks
    md = md
      .split(/(^```[\s\S]*?^```)/m)
      .map((chunk, i) =>
        i % 2
          ? chunk
          : chunk.replace(/(!?)\[([^\]]*)\]\(([^)\s]+)(\s+"[^"]*")?\)/g, (m, bang, text, url, t = '') => `${bang}[${text}](${rewriteLink(url, src)}${t})`),
      )
      .join('');
    const firstPara = md.split(/\n\s*\n/).map(plain).find((p) => p.length > 40 && !p.startsWith('|')) || pageTitle;
    const description = firstPara.length > 200 ? `${firstPara.slice(0, 197).replace(/\s+\S*$/, '')}…` : firstPara;
    const front = [
      '---',
      `title: ${yaml(pageTitle)}`,
      `description: ${yaml(description)}`,
      `editUrl: ${yaml(`${GITHUB}/edit/main/${src}`)}`,
      '---',
      '',
      `<!-- Generated from ${src} by site/scripts/sync-docs.mjs. Edit the source file, not this copy. -->`,
      '',
    ].join('\n');
    const footer = `\n\n---\n\n<small>This page is generated from [\`${src}\`](${GITHUB}/blob/main/${src}) in the repository.</small>\n`;
    const out = path.join(CONTENT, `${slug}.md`);
    fs.mkdirSync(path.dirname(out), { recursive: true });
    fs.writeFileSync(out, front + md.trim() + footer);
    written.push(`${slug}.md`);
  }
  fs.writeFileSync(
    path.join(CONTENT, '.gitignore'),
    `# Generated by site/scripts/sync-docs.mjs from the repository's Markdown\n${written.map((w) => `/${w}`).join('\n')}\n`,
  );
  console.log(`synced ${written.length} pages`);
}

function vendorThree() {
  // The guide's 3-D pages load three.js from here, so the site works offline and the version is
  // the one pinned in package-lock.json. three.js is MIT-licensed; its licence goes with it.
  const from = path.join(SITE, 'node_modules/three');
  const to = path.join(SITE, 'public/guide/vendor/three');
  fs.rmSync(to, { recursive: true, force: true });
  fs.mkdirSync(path.join(to, 'addons/controls'), { recursive: true });
  for (const f of ['three.module.js', 'three.core.js']) fs.copyFileSync(path.join(from, 'build', f), path.join(to, f));
  fs.copyFileSync(path.join(from, 'examples/jsm/controls/OrbitControls.js'), path.join(to, 'addons/controls/OrbitControls.js'));
  fs.copyFileSync(path.join(from, 'LICENSE'), path.join(to, 'LICENSE'));
}

sync();
vendorThree();
