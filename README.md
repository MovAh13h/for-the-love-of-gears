# For the love of Gears!

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/MovAh13h/for-the-love-of-gears/master/assets/banner-dark.svg">
  <img alt="for-the-love-of-gears" src="https://raw.githubusercontent.com/MovAh13h/for-the-love-of-gears/master/assets/banner-light.svg" width="1280">
</picture>

[![CI](https://github.com/MovAh13h/for-the-love-of-gears/actions/workflows/ci.yml/badge.svg)](https://github.com/MovAh13h/for-the-love-of-gears/actions/workflows/ci.yml)
[![Crates.io](https://img.shields.io/crates/v/for_the_love_of_gears.svg)](https://crates.io/crates/for_the_love_of_gears)
[![Docs.rs](https://docs.rs/for_the_love_of_gears/badge.svg)](https://docs.rs/for_the_love_of_gears)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://github.com/MovAh13h/for-the-love-of-gears/blob/master/LICENSE)

ISO-standard gear geometry in Rust. Calculate module, pitch, tooth profile dimensions, diameters, and centre distance — all in millimetres, all following ISO 54 and ISO 1328.

---

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
for_the_love_of_gears = "*"
```

---

## Roadmap

Ideas for future additions — contributions welcome:

- [ ] `Gear` struct combining all parameters for a complete gear definition
- [ ] Gear pair — meshing validation, gear ratio, contact ratio
- [ ] Helical gear support — helix angle, normal and transverse module
- [ ] Profile shift — non-standard addendum/dedendum for strength optimisation
- [ ] Rack geometry — the limiting case of infinite radius
- [ ] Bevel and worm gear families

---

## Contributing

Contributions are welcome. A few guidelines:

- **New formulas must cite a source.** ISO standards, KHK, Shigley's, or equivalent. Engineering formulas without a reference will not be merged.
- **Every public type needs tests.** Add them to the matching file in `tests/`.
- **All values are in millimetres.** Do not introduce other units without explicit conversion types.
- Open an issue first for large changes so the direction can be agreed before you write code.

---

## License

[MIT](https://github.com/MovAh13h/for-the-love-of-gears/blob/master/LICENSE) — Tanishq Jain
