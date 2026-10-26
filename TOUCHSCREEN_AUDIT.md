# JustRide touchscreen audit and software corrections

Audit date: 19 October 2026. Scope confirmed by the user: the current ILI9341/XPT2046 firmware. No physical device was flashed or reset. Existing unrelated local work was preserved.

## A. Hardware and documentation

### Identification from the actual build

| Item | Evidence and finding |
| --- | --- |
| MCU | `sdkconfig`, `sdkconfig.defaults` and `build/project_description.json`: **ESP32**, not ESP32-S3; 160 MHz configured; ESP-IDF 5.5.5 |
| Display module | Source describes a **2.8-inch SPI TFT module**. Exact module SKU, PCB revision and active-area dimensions in millimeters are not supplied. No schematic or BOM was found. |
| Display IC | `display_tft.c::tft_init`: **ILI9341** initialization; this is configuration evidence, not a physical ID-register measurement |
| Panel resolution | Native 240×320; current landscape logical space **320×240** |
| Display interface | SPI2, 40 MHz requested, MOSI=23, MISO=19, SCLK=18, CS=5, DC=2, reset=4, backlight=32 |
| Pixel format | RGB565, 16 bits, big-endian pixel bytes; `COLMOD=0x55` |
| Panel timing | Frame-control command `0xB1` receives `{0x00,0x18}`; actual panel refresh rate is **not measured**. SPI frequency and render-task cadence are not panel refresh measurements. |
| Host renderer | Custom C graphics, not LVGL; render task targets 50 ms/20 FPS and uses dirty updates and a 5,120-byte internal DMA strip |
| Display startup | Backlight off; hardware reset; software reset; display off; power/driver/VCOM setup; pixel/frame format; MADCTL rotation; sleep-out; display-on; black clear. Existing sequence and delays preserved. |
| DMA | `spi_bus_initialize(..., SPI_DMA_CH_AUTO)`; synchronous polling transactions; reusable `DMA_ATTR` strip for buffered regions; no new framebuffer |
| Touch IC | **XPT2046**, four-wire resistive controller, identified by driver/configuration and user-confirmed scope; no physical marking, schematic or live ID interrogation was available |
| Touch bus/pins | Shared SPI2; 2 MHz mode 0, CS=27; T_IRQ=14 configured as an input, not an ISR; no separate controller reset signal configured |
| Memory | PSRAM disabled. Touch task: 3,072-byte stack, priority 5; IMU task priority 4; render task priority 3. These settings remain unchanged. |

The README still contains historical ESP32-S3 instructions; **do not switch this checkout to ESP32-S3 based on that heading**. The audited build/configuration targets classic ESP32.

### Technical sources

- [XPTEK XPT2046 manufacturer datasheet, May 2007, hosted by Waveshare](https://files.waveshare.com/wiki/common/XPT2046_Datasheet.pdf): ADC/digital interface, control-byte tables and PENIRQ sections.
- [ILITEK ILI9341 manufacturer datasheet, hosted by Adafruit](https://cdn-shop.adafruit.com/datasheets/ILI9341.pdf): native resolution, COLMOD and frame-rate control; exact display-module assembly remains unidentified.
- [ESP-IDF 5.5.5 ESP32 SPI Master documentation](https://docs.espressif.com/projects/esp-idf/en/v5.5.5/esp32/api-reference/peripherals/spi_master.html): inline transaction storage, DMA buffer requirements, polling and shared-device locking. Checked against the installed `components/esp_driver_spi/src/gpspi/spi_master.c` implementation.
- [ESP-IDF 5.5.5 FreeRTOS documentation](https://docs.espressif.com/projects/esp-idf/en/v5.5.5/esp32/api-reference/system/freertos_idf.html): relative delay versus fixed-period scheduling. Scheduling was investigated but not retuned without measurements.

The XPT2046 documentation specifies a 12-bit ADC with up to 125 kHz sampling throughput. That is **not a 125 kHz coordinate report rate**. The existing host requests 2 MHz DCLK and polls every 20 ms plus work time. Its commands use differential conversion, 12-bit mode and PD1:PD0=00. No command byte was changed.

PENIRQ is documented and can signal contact between conversions; its behavior changes during conversions. An interrupt implementation would need to account for those transitions. With no measured idle-current or latency baseline, the existing polling task was retained rather than adding an unvalidated ISR or new task.

No configurable capacitive sensitivity, glove mode, water-rejection mode, controller gesture engine or downloadable controller firmware was established for this resistive controller. Those features were not enabled or claimed. Thresholding, filtering and gestures are implemented on the host.

## B. Baseline findings before driver edits

### End-to-end path

Physical pressure → XPT2046 conversion → `xpt2046_read_channel` over SPI under the shared TFT mutex → Z1/Z2 checks → five X and five Y samples → sorted trimmed mean → calibrated portrait coordinates → compile-time rotation and edge rejection → `main.c::touch_task` → release-confirmed gesture → `display_handle_touch_event` under display mutex → screen-specific handler → immediate/dirty rendering.

Existing parameters were recorded and retained:

| Parameter | Existing value retained |
| --- | --- |
| Polling | `vTaskDelay(20 ms)` after processing; nominal ceiling around 50 polls/s, not a measured rate |
| Contact check | Z1 > 180 and Z2 < 4000; not a computed resistance/pressure ratio |
| Coordinate filter | Five samples per axis, remove min/max, average the remaining three; not a pure median |
| Raw calibration | X: 300–3800; Y: 250–3850; extra sanity envelope ±200 ADC counts |
| Edge rejection | 4 logical pixels; unchanged |
| Press confirmation | One valid poll |
| Release confirmation | Three invalid/up polls; two transient bad samples are tolerated |
| Tap movement | At most 28 pixels maximum excursion on either axis |
| Swipe classification | At least 40 pixels; dominant-axis displacement must exceed 1.5× the other axis |
| Long hold | No separate long-press action; a stationary hold becomes one tap on release |
| Double tap | No separate double-tap feature |

### Reproduced defects

1. **Transport failures became ADC zeroes.** A failed Z2 read could satisfy the no-saturation test; a failed coordinate sample could be silently discarded by the trimmed mean while the result was accepted. Tests inject an error at every one of the 12 active-sample transactions.
2. **Stale output on rejection.** Calls made before initialization and some invalid reads did not consistently clear `is_pressed` and `pressure`.
3. **Ignored initialization errors.** IRQ GPIO configuration and warm-up SPI errors did not reliably stop initialization.
4. **Rotation/bounds defects.** Flipped portrait lacked a coordinate transform; the edge guard always used landscape bounds and rejected valid lower portrait coordinates. Current landscape mapping itself was retained.
5. **Short DMA buffers.** The original three-byte stack arrays did not meet the documented alignment/size criteria for the DMA fast path, permitting temporary DMA-buffer allocation/copying. Inline transaction storage is a supported path for this 24-bit transfer.

Before edits, the host fault-injection/gesture checks produced:

| Compile-time rotation | Checks | Failed before | Failed after |
| --- | ---: | ---: | ---: |
| Portrait | 82 | 27 | 0 |
| Landscape (active build) | 82 | 25 | 0 |
| Portrait flipped | 82 | 29 | 0 |
| Landscape flipped | 82 | 25 | 0 |
| Total | 328 | 106 | 0 |

These are assertion counts, not measured physical touch events or performance trials. Logs: `build/touch-baseline-tests.log` and `build/touch-after-tests.log`.

### Latency, rendering and environment limitations

The task sleeps after processing, so time spent waiting for SPI/display locks adds to its period. Gesture dispatch waits for release, then for three up samples. Under ideal 20 ms polling, release confirmation adds approximately 40–60 ms from physical release; real scheduling and lock waits can add more. These are timing deductions from code, not measurements. First-touch visual-feedback targets cannot be claimed while preserving the existing release-triggered UI.

At 2 MHz, 12 transactions of 24 bits account for 144 microseconds of wire time alone, excluding software/controller/locking costs. Idle pressure checks use two transactions. Transfer count and command bytes remain unchanged; transport failure now stops an invalid acquisition early.

The shared recursive SPI mutex is also held across some complete buffered drawing regions. The display mutex serializes gesture handling with rendering. Changing this ownership could corrupt rendering or behavior, so no rendering/locking redesign was performed. BLE/display queues and IMU redraw gating already exist and were preserved. OTA intentionally ignores touch actions; claiming active touch control during OTA would contradict current application behavior.

The driver allocates no history/queue/task objects per sample. The SDK can allocate temporary DMA buffers for unsuitable transaction buffers; the inline-storage change removes the original short external buffers from this path. No hardware allocator trace was captured.

### Hitbox review

No visible geometry or hitbox was changed. The notification-dismiss area is x=100–235, y=4–31 (136×28 pixels); navigation recenter is approximately x=288–319, y=142–171 (32×30 before the retained edge exclusion). These are the smallest notable hitboxes and should be checked against a verified 9–10 mm target. Brightness buttons, media controls and call actions have larger hit areas.

Exact active width/height in millimeters are missing, so accuracy/jitter/movement targets were **not** converted using an assumed panel geometry. Once measured, use `pixels = millimeters × logical_pixel_dimension / measured_active_dimension_mm`, swapping axes for portrait. Module diagonal labeling alone is not a calibration measurement.

## C. Changes made

| File/function | Previous → new behavior | Reason and expected benefit | Evidence |
| --- | --- | --- | --- |
| `main/display/touch.c::xpt2046_read_channel` | ADC value with zero on error → success flag plus output ADC value | Transport faults cannot masquerade as valid zero data | Failure injection at all 12 transaction positions |
| Same function | External three-byte tx/rx arrays → inline four-byte storage with 24-bit transaction length | Avoid short-buffer DMA bounce allocation/copy opportunity without new memory history | IDF documentation/local implementation; host transfer checks; ESP-IDF compile |
| `read_median` → `read_trimmed_mean` | Ambiguous name and ignored read errors → accurately named, aborts on transport failure | Preserve the same five-sample trimmed mean while rejecting invalid acquisition | Existing calibrated valid-point checks and injected failures |
| `touch_init` | Ignored GPIO/warm-up errors → propagate GPIO failure; remove SPI device and clear handle on warm-up failure | Prevent apparently initialized but nonworking input | Initialization failure/recovery checks; IDF compile |
| `touch_get_point` | Inconsistent rejection state → clear pressed/pressure before acquisition | No stale pressed/pressure data escapes a false return | Uninitialized/error/threshold/range/edge rejection checks |
| Same function | Landscape-only edge bounds and missing flipped portrait → configured bounds and flipped-portrait mapping | Correct unused orientation paths without changing active landscape calibration | Four separate host rotation builds |
| `touch.c`, `touch.h` comments/API contract | Misstated sampling/reference mode → describe actual filter, command bits and false-return contract | Make future maintenance/tuning match the implementation | Source and datasheet review |
| `tools/test_touch.py` | No dedicated touch host regression check → compile production driver plus extracted production FSM against deterministic SPI stubs | Reproducible fault/gesture checks without physical flashing | 328 assertions passing |

No pressure threshold, raw calibration, sample count, filter, gesture threshold, edge margin, poll delay, task priority, UI design, navigation state or BLE behavior was tuned. No controller register was added or changed.

## D. Before/after measurements and build results

| Requested hardware metric | Before | After |
| --- | --- | --- |
| Touch reporting rate | Not measured | Not measured |
| Touch-down latency (median / p95 / worst) | Not measured | Not measured |
| Touch-to-visible-feedback latency (median / p95 / worst) | Not measured | Not measured |
| Coordinate jitter in mm | Not measured | Not measured |
| Accuracy in mm and edge accuracy | Not measured | Not measured |
| Missed physical taps | Not measured | Not measured |
| False physical touches and exposure duration | Not measured | Not measured |
| Runtime heap / largest allocatable block | Not measured | Not measured |
| Touch-task stack high-water mark | Not measured | Not measured |
| CPU utilization | Not measured | Not measured |

Static link figures are separate from runtime heap: final map reports DRAM sections 38,532 bytes and IRAM 115,083 bytes. Firmware binary: **721,360 bytes**, fitting the existing 0x1B0000 app partition with 59% free. The complete build contains other pre-existing local changes, so these are whole-build figures, not an attributed before/after improvement.

Validation:

- `python tools/test_touch.py`: four host builds, **328 checks passed**. Uses MSVC with local test declarations; the physical display and MCU are not emulated. The production `touch.c` is compiled; the production gesture FSM is extracted with only its infinite task lifetime bounded for replay.
- ESP-IDF 5.5.5 `idf.py build`: **passed** for `esp32`; final log `build/touch-final-build.log`; binary `build/justride_firmware.bin`. No flash command executed.
- `git diff --check` on changed touch sources: passed.
- No new firmware dependencies, tasks, event queues, framebuffers or runtime diagnostic logging added.

### Coverage and hardware acceptance checklist

| Scenario | Result / remaining check |
| --- | --- |
| Quick tap | Host replay emits exactly one tap |
| Repeated taps | Host replay emits two separate taps when three release samples separate contacts |
| Long press | Host 520 ms hold preserves one tap on release; no unrequested 500 ms action added |
| Swipe / diagonal / out-and-back drag | Four directions preserved; diagonal and large out-and-back motion do not become taps |
| Release / noisy gaps | Existing three-up confirmation preserved; two missing samples do not split a swipe |
| Raw range / edge / rotations | Invalid reads rejected; two unequal calibrated interior coordinates checked per rotation; portrait lower range remains reachable |
| Idle | Synthetic 640 ms no-contact replay emits no gesture; **not** a real ghost-touch exposure test |
| Restart / initialization recovery | Warm-up failure rejected and subsequent initialization succeeds in stub tests; physical power-cycle pending |
| Vibration, orientation changes, accidental contacts | Hardware testing pending |
| Battery / USB / motorcycle supply / engine running | Hardware noise testing pending |
| Wet fingers / rain | Hardware testing pending; water rejection unverified |
| Riding gloves | Hardware testing pending; no glove-mode claim |
| BLE notifications, reconnection, GPS updates and navigation | Source ownership preserved; concurrent hardware testing pending |
| Screen flicker / UI feedback | No renderer edits; optical measurement/visual hardware check pending |
| OTA | Existing interaction lock preserved; responsiveness during flash operations not measured |
| Long-duration memory/task stability | Hardware soak test pending |

For acceptance, use the same device, power source, physical press method and firmware build configuration before/after. Record at least 100 physical trials per latency condition. Capture contact onset, first valid sample, release, gesture dispatch and optical pixel change on a common timebase; calculate median, nearest-rank p95 and maximum. Include short taps with varied phase relative to polling, edge locations, navigation/BLE/charging load and the original noise filter. Record exact idle exposure duration and false-event count for each environment, not merely "none". Record free heap, largest block, task stack high-water marks and CPU load at start/end of the soak. Existing renderer logs provide some heap/render figures but are not touch latency measurements.

## E. Final assessment

Software robustness and orientation correctness improved in reproducible host tests. Unsupported sensitivity, scan-rate and filtering changes were avoided. Whether physical response meets 35 ms touch-down, 70 ms visible feedback, 0.5 mm jitter or 1 mm accuracy targets remains **unverified**. Release-confirmed interaction and shared rendering locks are likely limits worth measuring, rather than hiding with aggressive sensitivity changes.

The next decision requires hardware timing, verified active-area dimensions and environmental tests. Consider interrupt-assisted idle waiting, fixed-period acquisition or shorter release confirmation only after measuring PENIRQ behavior, scheduling/lock waits and false-release rates on this exact board. Do not treat the passing host tests as wet/glove validation, road qualification or industrial certification.
