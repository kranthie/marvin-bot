# Marvin Bot documentation site

The project website, published to GitHub Pages at <https://kranthie.github.io/marvin-bot/>. It has three
parts:

| Part | Where it comes from |
|---|---|
| Home page: the robot in 3-D | `src/pages/index.astro` and `src/scripts/marvin-viewer.js` |
| Documentation | the repository's own Markdown (`README.md`, `docs/`, `sim/*/README.md`, `bom/README.md`, `bench/README.md`, `CONTRIBUTING.md`, …), copied in at build time by `scripts/sync-docs.mjs`; plus two pages written for the site (`src/content/docs/getting-started.md`, `tutorials.md`) |
| The interactive guide (tutorials) | Markdown pages in `src/content/docs/guide/` (a map, 23 playgrounds, a glossary and a sources page), with each playground's controls, plots and script in `src/guide/<page>/` |

It is built with [Astro Starlight](https://starlight.astro.build) (MIT). Dependencies are pinned in
`package.json` and `package-lock.json`.

## Editing

- **Docs:** edit the Markdown in the repository, never the copies in `src/content/docs/` (they are
  generated and git-ignored; the list is in `src/content/docs/.gitignore`). Relative links between
  synced files become site links; links to other repository files go to GitHub; images are copied.
  To add a document to the site, add it to `PAGES` in `scripts/sync-docs.mjs` and to the sidebar in
  `astro.config.mjs`.
- **The guide:** each page is an MDX file in `src/content/docs/guide/`. The prose is Markdown, with
  Starlight asides for the "You will learn", "In the repo" and "Where sources and the repo disagree"
  boxes. A playground's interactive parts are plain HTML in `src/guide/<page>/widget-N.html`
  (rendered by `<RawHtml>`), its script in `script.js` (run at the end of the page by `<PageScript>`)
  and any styles it needs in `style.css`; `src/styles/guide.css` maps the playgrounds' class names
  onto Starlight's colours, so they follow the light and dark themes. The 3-D model page bundles its
  viewer (`src/guide/00-3d-model/viewer.js`) with the site's three.js. Follow the guide's
  conventions: cite every non-obvious claim with a numbered source, say where a source and the
  repository disagree, use SI units and British spelling, refer only to open-source projects (bought
  parts are named as parts), and date the repository numbers. Update the glossary and sources pages
  when a playground adds a term or a source. The old `public/guide/*.html` addresses redirect to the
  new pages.
- **The 3-D model:** `scripts/build_model.py` packs the generated URDF, `description/cad_model.yaml` and
  the code-CAD meshes into `public/guide/model/marvin_model.js` (git-ignored), used by the home page and
  playground 00. Both viewers share `src/scripts/marvin-render.js`: smooth shading that keeps sharp
  edges (crease-angle normals), image-based lighting and soft shadows. The Pages workflow rebuilds it on every deploy, so it always matches the design.

## Building and previewing locally

You need Node.js 22.22 or later and the repository's Python environment (for the 3-D model).

```bash
pip install -e ".[cad,hand]"          # from the repository root
python -m cad.build > /dev/null       # part meshes (git-ignored)
python site/scripts/build_model.py    # the 3-D model data for the site
cd site
npm ci
npm run build                         # syncs the docs, then builds into dist/
npm run check-links                   # every internal link, image and anchor resolves
npx astro preview                     # http://localhost:4321/marvin-bot/
```

`npm run dev` serves a live-reloading copy while you edit.

## Deploying

`.github/workflows/pages.yml` builds and deploys the site on every push to `main` that touches the docs,
the description or the site. In the repository's settings, **Pages → Build and deployment → Source**
must be set to **GitHub Actions** once.

## Licences and credits

The site's code and text, including the guide, are Apache-2.0 like the rest of the documentation. The
3-D model data is derived from the hardware design files (CERN-OHL-S-2.0); the hand meshes from the
AmazingHand by Pollen Robotics (CC-BY-4.0). three.js (MIT) is bundled from `node_modules` at build
time, keeping its licence header. Every source the guide cites is listed on
its sources page.

## If the 3-D model does not appear

The home page and playground 00 need WebGL (3-D graphics in the browser). If a page says WebGL is off,
open `chrome://gpu` in Chrome or Chromium: the "WebGL" line should say "Hardware accelerated". If not,
switch on "Use graphics acceleration when available" in `chrome://settings/system`; if it is still off,
the graphics card is probably on the browser's block list, which `chrome://flags/#ignore-gpu-blocklist`
overrides. Or use another browser, such as Firefox.
