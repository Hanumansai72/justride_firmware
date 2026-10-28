# 1.75-inch round display port

The existing work was checkpointed and pushed before this port: `12504b9`
on `origin/version-2.0`. The round-display implementation is maintained on
`round_display`; `version-2.0` retains the original display implementation.
Build output, local caches, packaged downloads and editor metadata are
excluded by `.gitignore`. Previously tracked artifacts are removed from the
round branch's index without deleting the local copies or rewriting history.

## Hardware evidence

- `H0175Y003AMT003 V0.pdf`, module parameters, page 4: round 1.75-inch
  AMOLED, **466 RGB x 466**, **CO5300**, capacitive **CST9217** touch.
  Its physical viewing area is 43.76 mm in diameter.
- `H0175Y003AMT003 V1接线图.pdf`: use the **QSPI** column, not the MIPI
  wiring. QSPI D2 is FPC **20** and D3 is FPC **19** (easy to reverse).
- `CO5300_Datasheet_V0.00_20230328(1).pdf`, sections 5.2 and 7.5:
  command opcode `02`, 24-bit address `00 CMD 00`, four-lane pixel opcode
  `32`, RGB565 `3A/55`, brightness `51`, brightness enable `53/20`.
- CST9217 report/ACK/identity verified against
  [SensorLib TouchDrvCST92xx](https://github.com/lewisxhe/SensorLib/blob/master/src/touch/TouchDrvCST92xx.cpp).

The CO5300 IC datasheet covers many possible panels. The module specification
has an empty initialization-code section. Startup therefore uses documented
DCS reset, pixel format, sleep-out and display-on commands. Supplier-specific
power/gamma initialization and glass offsets still require validation on the
actual V1 module; a successful compile is not a panel bring-up test.

## Wiring configuration

Defaults are an explicit proposed ESP32 wiring assignment, **not pin numbers
specified by the display manufacturer**. Change them in
`idf.py menuconfig` > `JustRide display` to match the assembled board.

| Signal | V1 FPC pin | Default ESP32 GPIO |
| --- | --- | --- |
| QSPI clock | 18 | 18 |
| CS | 15 | 5 |
| D0 | 16 | 23 |
| D1 | 17 | 19 |
| D2 | 20 | 25 |
| D3 | 19 | 26 |
| Display reset | 14 | 4 |
| Touch SDA | 4 | 32 |
| Touch SCL | 5 | 33 |
| Touch reset | 3 | 27 |
| Touch interrupt | 2 | 14 |

Touch uses I2C1, address `0x5A`, separate from MPU6500 I2C0/GPIO21,22.
Swapping or mirroring touch coordinates is configurable for the assembly.
GPIO32 is no longer a PWM backlight: this OLED uses its brightness register.

Follow the V1 power diagram: IOVCC 1.65–3.3 V (recommended 1.8 V),
VCI 2.7–3.6 V (recommended 3.3 V), VBAT 3.7–4.5 V, and VCI-EN
2.7–3.6 V (recommended 3.3 V). TP supply is 3.3 V. The V0 mechanical
drawing labels touch I2C as 1.8 V: use suitable level translation unless the
supplier confirms the V1 touch logic tolerates the host's 3.3 V. External
regulated supplies/power sequencing are required; GPIOs are signal pins.
MTP programming power is unconnected in the V1 diagram. Do not apply its
programming voltage during normal use.

## UI and protocol compatibility

Reference assets are `justride-customer/src/assets/device-front-nav.png`,
`device-music.png`, `device-caller.png`, and
`public/images/aventura/clock-screen-v4.png`.

- Navigation: full circular map with a vignette, glowing green rounded route,
  shaded/outlined rider arrow, white turn arrow and distance/action text,
  upper-right compass and red/white numeric badge. Its preview uses the
  supplied reference's `70` sample only in an explicitly labeled `DEMO`.
  The BLE packet currently has vehicle speed but no speed-limit field:
  the production speed-limit sign shows `--` rather than mislabeling vehicle
  speed as a legal road limit. A supplied speed-limit model value is supported.
- Clock: green perimeter, large clock/date, route-remaining distance.
  No battery percentage is invented when battery telemetry is absent.
- Music: title/artist, previous/play/pause/next touch controls and volume.
  No elapsed-time bar is fabricated: the current BLE media packet has no
  position/duration fields.
- Call: caller name, red decline on the left and green accept on the right,
  with action bits/connection state controlling availability.
- Notifications: bounded round-safe card, tap to dismiss.
- Existing settings, brightness, gallery and OTA remain available in an
  inscribed 352x264 physical pane. Their existing 320x240 protocol/hitboxes
  are preserved through a shared inverse transform.

Rendering uses a 320-square logical canvas expanded to 466-square scanlines.
The transport needs two physical scanlines rather than a 434 KB framebuffer.
The existing BLE map/gallery sizes remain compatible with the phone app.
Received real map pixels and rider markers are centered without changing
their aspect ratio, and map attribution is retained. On opening navigation
without geometry, a deterministic, explicitly labeled reference demo appears
immediately (irregular dark roads, S-shaped route, `300 m`, `70`). These are
not telemetry. Once a real map or at least two route points arrive, live
rendering replaces the demo. After that, missing data never restores sample
guidance. An explicitly opened navigation screen without an active ride can
also show the demo; normal inactive-ride screen cycling is unchanged.
Without a live raster map, only received route geometry is drawn in live mode.
The map covers the entire dial, including the instruction area. Antialiased
glyphs and strokes blend over buffered map pixels instead of clearing black
rectangles. Raster maps already containing a route retain the phone's route
geometry/style; the firmware's vector fallback uses the glowing green stroke.

## Verification and preview

Run `python tools/preview_round.py` with Pillow and MSVC Build Tools. It compiles
the production `round_ui.c`, `hud_ui.c`, font assets and TFT primitives into a
host raster sink, asserts drawing bounds, checks display/touch transforms,
and checks CST9217 active/release/malformed reports. Output:
`build/round-preview/screens.png`, plus individual screen PNGs.
An optional `--reference PATH_TO_IMAGE` also writes
`build/round-preview/reference-comparison.png` beside the untouched reference.

Other screen values, names, date and live-map test packets are host-only
fixtures. The navigation demo is the same function used by production.
These are software-rendered previews, not photographs of an attached panel.
The production firmware uses the existing BLE/RTC values. Final panel startup,
color order, touch orientation/voltage and timing need hardware confirmation.

Build with `idf.py build`. The default is the new round profile. Disable
`CONFIG_JUSTRIDE_ROUND_AMOLED` to select the original ILI9341/XPT2046 profile.

Verification completed 27 October 2026: ESP-IDF 5.5.5 ESP32 build passed,
application fits with 57% free in the smallest OTA slot. Host round
captures and geometry/CST9217 report assertions passed. Original XPT2046
driver/gesture regression checks passed in all four orientations (82 each,
328 total). The test harness was updated to include the local theme/config
headers and honor the existing `TOUCH_INVERT_RAW_X` setting.
The reference refinement also passes transparent-footer, large-distance,
heading-rotation and phone-map crop render cases. Initial navigation captures
are byte-for-byte identical even after a splash transition. Splash progress
assertions verify that no pixels outside the progress bar are modified.

## Grid root cause and reference correction

The unwanted grid was in `tools/preview_round.py`, function
`reference_map_fixture()`: a `y += 62` line loop and an `x += 90` line loop
drew repeated crossing diagonals. It was host-preview geometry, not generated
by the CO5300, touch controller, or the production phone-map receiver.

Those loops are removed. `round_ui.c:reference_roads()` now draws a bounded
table of individually placed, irregular reference-derived junctions, only
for `round_ui_navigation_demo()`. `display.c:display_nav_render_partial()`
selects this labeled initial scene until live geometry is ready; the
existing BLE queues, map packets and route updates remain in place. The host
checks exercise both vector and received-raster rendering, not a live BLE
connection to a physical device.

`round_ui_splash()` uses existing official brand masks on the full round
canvas with a plain black footer (bottom contour waves removed).
`round_ui_splash_progress()` updates only its bar. In
`display_show_splash()`, brightness is zero while the first complete frame
is rendered, and background tasks are excluded by the existing mutex. This
is software sequencing protection, not a claim of verified glitch-free
electrical startup on the untested panel.

The shared `JR_RGB565` macro also needed parentheses around its argument:
composed RGB expressions previously produced incorrect road colors. A host
assertion covers expression-to-literal equivalence.

Captures in `build/round-preview/`:

- `splash.png`, `splash-start.png`, `splash-complete.png`: real C splash renderer.
- `navigation.png`: actual initial-demo renderer; `reference-comparison.png`:
  untouched supplied reference alongside its 466-pixel round screen capture.
- `navigation-before.png` and `before-after.png`: saved previous grid preview
  and comparison (comparison generated when the saved before image exists).
- `live-vector.png`, `phone-map.png`, `navigation-left.png`, `gps-lost.png`,
  `unknown-speed-limit.png`: separate live-data renderer cases.

Unavoidable/current differences: the metal enclosure is physical hardware,
not screen content; `DEMO` is added to avoid presenting samples as guidance;
roads are reference-derived vectors, not the unavailable original map source;
the existing 320-square logical renderer is expanded to native 466-square
scanlines, so edges/fonts are not identical to the antialiased marketing
image. The preview reproduces the transport's integer pixel replication.
Final color, scan timing, touch alignment and power-cycle behavior still need
the actual display module.
