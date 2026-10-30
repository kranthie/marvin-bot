// Marvin Bot documentation site: Astro Starlight, published to GitHub Pages by
// .github/workflows/pages.yml. The docs pages are copied from the repository's Markdown
// (scripts/sync-docs.mjs), the home page is src/pages/index.astro, and the interactive guide is
// static HTML under public/guide/.
import starlight from '@astrojs/starlight';
import { defineConfig } from 'astro/config';

const repo = 'https://github.com/kranthie/marvin-bot';

export default defineConfig({
  site: 'https://kranthie.github.io',
  base: '/marvin-bot',
  trailingSlash: 'always',
  // three.js is one large module, loaded only by the home page
  vite: { build: { chunkSizeWarningLimit: 1500 } },
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
      customCss: ['./src/styles/theme.css'],
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
            { label: 'Marvin Bot in 3-D', link: '/guide/3d/', attrs: { target: '_self' } },
            { label: 'The interactive guide (map)', link: '/guide/', attrs: { target: '_self' } },
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
