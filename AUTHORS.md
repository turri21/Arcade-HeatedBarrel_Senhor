# Authors and Credits

## HeatedBarrel_MiSTer core

**Author**: Umberto Parisi ([rmonic79](https://github.com/rmonic79))

The original RTL source files for the Heated Barrel specific logic
(under `rtl/HeatedBarrel/`: the Seibu SEI300 / COP coprocessor
`HeatedBarrel_cop3`, the Seibu CRTC, the video timing, the BG / MG / FG / TEXT
tile and text renderers, the sprite renderer, the palette, the main-CPU map and
node, the audio Z80 wrapper, the SDRAM / DDR3 bridges, the pause overlay, and
the project wrapper `Template.sv`) are copyright Umberto Parisi and distributed
under GNU GPL v3 or later.

## Third-party components

This core builds on top of excellent open-source projects. All third-party
sources retain their original copyright and license. The core as a whole
is distributed under **GNU GPL v3 or later** to stay compatible with the
most restrictive upstream (JTFRAME / JTCORES).

| Component | Author | Project | License |
|-----------|--------|---------|---------|
| **fx68k** — cycle-accurate Motorola 68000 (main CPU) core | Jorge Cwik ([ijor](https://github.com/ijor)) | [ijor/fx68k](https://github.com/ijor/fx68k) | own license (see header) |
| **T80** — Zilog Z80 (sound CPU) core | Daniel Wallner (original, OpenCores), with MikeJ fixes and Sorgelig / MiSTer-devel maintenance | [MiSTer-devel](https://github.com/MiSTer-devel) | GPL-3 |
| **JTOPL2** — Yamaha YM3812 (OPL2) FM synthesizer | Jose Tejada ([@topapate](https://twitter.com/topapate)) | [jotego/jtopl](https://github.com/jotego/jtopl) | GPL-3 |
| **JT6295** — OKI MSM6295 ADPCM decoder | Jose Tejada | [jotego/jt6295](https://github.com/jotego/jt6295) | GPL-3 |
| **JTFRAME** — framework, clock enables, filters, mixer, shift registers | Jose Tejada | [jotego/jtframe](https://github.com/jotego/jtframe) | GPL-3 |
| **SDRAM / DDR3 controllers** — SDRAM controller and `ddram` DDR3 reader (derived from `ddram_4port.sv`) | Sorgelig / MiSTer-devel | [MiSTer-devel](https://github.com/MiSTer-devel) | GPL-3 |
| **MAME** — reference for the Seibu SEI300 / COP coprocessor (`raiden2cop`), Seibu CRTC, SEI0211 sprites, graphics banking, memory maps and timing | MAMEDev team | [mamedev/mame](https://github.com/mamedev/mame) | GPL-2+ |
| **sys/ framework** — MiSTer HPS/IO, OSD, video scaler, audio | Sorgelig / MiSTer-devel | [MiSTer-devel/Main_MiSTer](https://github.com/MiSTer-devel/Main_MiSTer) | GPL-3 |

## Reference

- **Heated Barrel arcade hardware** — TAD Corporation / Seibu, 1992. This FPGA
  core is a reimplementation from hardware documentation, MAME source code, and
  observation of real hardware behavior. ROMs are **not** included and must be
  provided by the user.
- **MAME project** — invaluable reference for memory maps, timing, the Seibu
  SEI300 / COP coprocessor (`raiden2cop`), the Seibu CRTC and SEI0211 sprite
  hardware, and the graphics banking. [mamedev/mame](https://github.com/mamedev/mame)
