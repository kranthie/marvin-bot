// Check every internal link, image and script in the built site (dist/) points at a file that exists,
// including #anchors within the site. Run after `npm run build`; exits 1 if anything is broken.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const DIST = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../dist');
const BASE = '/marvin-bot/';
const files = [];
(function walk(d) {
  for (const e of fs.readdirSync(d, { withFileTypes: true })) {
    const p = path.join(d, e.name);
    if (e.isDirectory()) walk(p);
    else if (p.endsWith('.html')) files.push(p);
  }
})(DIST);

const ids = new Map();
const idsOf = (file) => {
  if (!ids.has(file)) ids.set(file, new Set([...fs.readFileSync(file, 'utf8').matchAll(/\bid="([^"]+)"/g)].map((m) => m[1])));
  return ids.get(file);
};
function resolve(from, url) {
  const [p, hash] = url.split('#');
  let target;
  if (!p) target = from;
  else if (p.startsWith(BASE)) target = path.join(DIST, decodeURI(p.slice(BASE.length)));
  else if (p.startsWith('/')) return { bad: 'outside the site base' };
  else target = path.resolve(path.dirname(from), decodeURI(p));
  if (fs.existsSync(target) && fs.statSync(target).isDirectory()) target = path.join(target, 'index.html');
  if (!fs.existsSync(target)) return { bad: 'missing file' };
  if (hash && target.endsWith('.html') && !idsOf(target).has(decodeURIComponent(hash))) return { bad: `missing #${hash}` };
  return {};
}
let bad = 0;
for (const f of files) {
  const html = fs.readFileSync(f, 'utf8');
  for (const m of html.matchAll(/\b(?:href|src)="([^"]+)"/g)) {
    const url = m[1];
    if (/^(https?:|mailto:|data:|javascript:)/.test(url) || url.includes('${')) continue;
    const r = resolve(f, url);
    if (r.bad) {
      bad++;
      console.log(`${path.relative(DIST, f)}: ${url} (${r.bad})`);
    }
  }
}
console.log(`${files.length} pages checked, ${bad} broken link${bad === 1 ? '' : 's'}`);
process.exit(bad ? 1 : 0);
