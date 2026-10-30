// Marvin Bot documentation site: Astro Starlight, published to GitHub Pages by
// .github/workflows/pages.yml. The docs pages are copied from the repository's Markdown
// (scripts/sync-docs.mjs), the home page is src/pages/index.astro, and the interactive guide is
// in src/content/docs/guide/, with its playgrounds' HTML and scripts in src/guide/.
import starlight from '@astrojs/starlight';
import { defineConfig } from 'astro/config';

const repo = 'https://github.com/kranthie/marvin-bot';
// one collapsible sidebar group per stage of the guide's learning path
const stage = (label, pages) => ({ label, collapsed: true, items: pages.map((p) => `guide/${p}`) });

export default defineConfig({
  site: 'https://kranthie.github.io',
  base: '/marvin-bot',
  trailingSlash: 'always',
  // three.js is one large module, loaded only by the 3-D viewers; keep its licence header
  vite: { build: { chunkSizeWarningLimit: 1500, rolldownOptions: { output: { comments: { legal: true } } } } },
  integrations: [
    starlight({
      title: 'Marvin Bot',
      description:
        'An open-source 1.44 m humanoid robot on RobStride actuators and a Jetson AGX Orin, designed in code and trained in simulation.',
      logo: { src: './src/assets/logo.svg', replacesTitle: false },
      favicon: '/favicon.svg',
      social: [{ icon: 'github', label: 'GitHub', href: repo }],
      editLink: { baseUrl: `${repo}/edit/main/` },
      lastUpdated: false,
      // link previews: the social card (made by scripts/social-card.html) on every page
      head: [
        { tag: 'meta', attrs: { property: 'og:image', content: 'https://kranthie.github.io/marvin-bot/social-card.png' } },
        { tag: 'meta', attrs: { property: 'og:image:width', content: '1280' } },
        { tag: 'meta', attrs: { property: 'og:image:height', content: '640' } },
        { tag: 'meta', attrs: { property: 'og:image:alt', content: 'Marvin Bot, an open humanoid robot you can build, shown as its 3-D model' } },
      ],
      customCss: ['./src/styles/theme.css', './src/styles/guide.css'],
      sidebar: [
        {
          label: 'Start here',
          items: [
            { label: 'Home', link: '/' },
            'overview',
            'getting-started',
            'plan',
            'roadmap',
            'requirements',
            'glossary',
          ],
        },
        {
          label: 'Design',
          items: ['decisions', 'design/concept', 'design/stairs', 'design/assembly-wiring'],
        },
        {
          label: 'Simulation',
          items: ['simulation/isaac-lab', 'simulation/motion-tracking', 'simulation/skills'],
        },
        { label: 'Build', items: ['build/bom', 'build/bench', 'build/safety'] },
        {
          label: 'Tutorials',
          items: [
            'tutorials',
            'guide',
            'guide/00-3d-model',
            stage('Stage 1 · Foundations', ['01-dof-kinematics', '02-balance', '03-architecture']),
            stage('Stage 2 · Mechanics', ['04-actuators', '05-leg-loads-stairs', '06-ankle-linkage', '07-mass-proportions', '08-structure-cad']),
            stage('Stage 3 · Electrical, sensing and safety', ['09-power-battery', '10-can-bus', '11-sensors-compute', '12-safety']),
            stage('Stage 4 · Model and simulation', ['13-description', '14-simulation', '15-pd-control']),
            stage('Stage 5 · Learning', ['16-rl-walking', '17-motion-tracking', '18-wbc-vla', '19-hands']),
            stage('Stage 6 · Toward hardware', ['20-sim-to-real', '21-bom-costs', '22-roadmap']),
            'guide/glossary',
            'guide/sources',
          ],
        },
        {
          label: 'Project',
          items: [
            'project/contributing',
            'project/acknowledgments',
            'project/security',
            'project/code-of-conduct',
            'project/licences',
          ],
        },
      ],
    }),
  ],
});
