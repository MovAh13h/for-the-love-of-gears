# for-the-love-of-gears

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="exports/banner-dark.svg">
  <img alt="for-the-love-of-gears" src="exports/banner-light.svg" width="1280">
</picture>

A Rust library that simplifies gear-related computation and abstracts away
the complications.

## Asset map

| File | Use |
| --- | --- |
| `exports/mark-{light,dark}.svg` | The gear mark on its own — favicon source, small inline marks |
| `exports/square-{light,dark}.svg` | Square avatar — GitHub profile, crates.io avatar |
| `exports/lockup-{light,dark}.svg` | Horizontal mark + wordmark — sidebars, docs headers |
| `exports/badge-{light,dark}.svg` | Slim badge-style mark — inline next to crates.io / docs.rs badges |
| `exports/banner-{light,dark}.svg` | Wide README hero — the `<picture>` block above |

All SVGs render with the JetBrains Mono / fallback monospace stack. They are
pure vector — scale to any size, edit by hand if needed.
