import { defineConfig } from 'vitepress'

const RATAVYR = 'https://ratavyr.bzy-logics.com'
const SINDRAVA = 'https://sindrava.bzy-logics.com'

export default defineConfig({
  title: 'MotionCore',
  description: 'Plugin-first, hardware-independent robotics motion control in Rust',
  cleanUrls: true,
  head: [['link', { rel: 'icon', href: '/logo.svg', type: 'image/svg+xml' }]],
  srcExclude: ['README.md'],
  sitemap: { hostname: 'https://motioncore.bzy-logics.com' },

  themeConfig: {
    logo: '/logo.svg',

    nav: [
      { text: 'Guide', link: '/guide/introduction', activeMatch: '/guide/' },
      { text: 'Hardware', link: '/hardware/', activeMatch: '/hardware/' },
      { text: 'Reference', link: '/reference/cli', activeMatch: '/reference/' },
      { text: 'Internals', link: '/internals/architecture', activeMatch: '/internals/' },
      {
        text: 'Docs',
        items: [
          { text: 'motioncore — engine & firmware', link: '/' },
          { text: 'ratavyr — REST/WebSocket gateway', link: RATAVYR },
          { text: 'sindrava — operator UI', link: SINDRAVA },
        ],
      },
    ],

    socialLinks: [{ icon: 'github', link: 'https://github.com/bzy-logics/motioncore' }],

    editLink: {
      pattern: 'https://github.com/bzy-logics/motioncore/edit/main/website/:path',
      text: 'Edit this page on GitHub',
    },

    sidebar: {
      '/guide/': [
        {
          text: 'Getting started',
          items: [
            { text: 'Introduction', link: '/guide/introduction' },
            { text: 'Quick start (simulation)', link: '/guide/quick-start' },
            { text: 'Running the full stack', link: '/guide/full-stack' },
          ],
        },
        {
          text: 'Concepts',
          items: [
            { text: 'How it fits together', link: '/guide/concepts' },
            { text: 'Robot states', link: '/guide/robot-states' },
            { text: 'Safety model', link: '/guide/safety' },
          ],
        },
        {
          text: 'Using MotionCore',
          items: [
            { text: 'Robot configuration', link: '/guide/configuration' },
            { text: 'Moving axes', link: '/guide/motion' },
            { text: 'Plugins', link: '/guide/plugins' },
          ],
        },
        {
          text: 'Other components',
          items: [
            { text: 'Gateway (ratavyr) ↗', link: RATAVYR },
            { text: 'Operator UI (sindrava) ↗', link: SINDRAVA },
          ],
        },
      ],
      '/hardware/': [
        {
          text: 'Hardware',
          items: [
            { text: 'Overview & supported boards', link: '/hardware/' },
            { text: 'Firmware: build, flash, verify', link: '/hardware/firmware' },
            { text: 'STM32F411 Black Pill', link: '/hardware/stm32f411' },
            { text: 'RP2040 (Raspberry Pi Pico)', link: '/hardware/rp2040' },
          ],
        },
      ],
      '/reference/': [
        {
          text: 'Reference',
          items: [
            { text: 'CLI', link: '/reference/cli' },
            { text: 'Configuration schema', link: '/reference/config-schema' },
            { text: 'Engine gRPC API', link: '/reference/engine-api' },
            { text: 'Faults & error codes', link: '/reference/errors' },
            { text: 'Environment variables', link: '/reference/environment' },
            { text: 'REST & WebSocket API ↗', link: `${RATAVYR}/reference/rest-api` },
          ],
        },
      ],
      '/internals/': [
        {
          text: 'Internals',
          items: [
            { text: 'Architecture', link: '/internals/architecture' },
            { text: 'Motion pipeline', link: '/internals/motion-pipeline' },
            { text: 'MCU wire protocol', link: '/internals/mcu-protocol' },
            { text: 'Contracts & versioning', link: '/internals/versioning' },
            { text: 'Testing', link: '/internals/testing' },
            { text: 'Roadmap & status', link: '/internals/roadmap' },
          ],
        },
      ],
    },

    search: { provider: 'local' },
    outline: { level: [2, 3] },

    footer: {
      message: 'Licensed under GPL-3.0. Not a certified safety system — always use an independent hardware E-stop.',
    },
  },
})
