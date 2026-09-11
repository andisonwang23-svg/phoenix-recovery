# PHOENIX RECOVERY — Build toolchain notes

The PlatformIO toolchain on this machine had two corrupted/unsupported pieces
that were repaired during bring-up. This file records what was wrong and how it
was fixed so it can be diagnosed quickly if a future build breaks.

## 1. `framework-arduinoespressif32` was corrupted (3855 zero-byte files)

The installed ESP32 Arduino framework under `~/.platformio/packages/` had nearly
every file truncated to 0 bytes (only 11 of hundreds of `cores/esp32` files were
intact). This is not a normal state and is not something a clean checkout can
reproduce.

**Fix:** the package directory was deleted and PlatformIO re-downloaded it from
the registry:

```sh
rm -rf ~/.platformio/packages/framework-arduinoespressif32
pio run -e rocket    # re-installs the framework (≈400 MB) and builds
```

Verification that the corruption is gone:

```sh
find ~/.platformio/packages/framework-arduinoespressif32 -type f -size 0 | wc -l   # ~0
```

## 2. PlatformIO ≥ 6.0 ignores `src_dir` / `board_build.variant_dir` per-env

PlatformIO 6.x only honours `src_dir` in the global `[platformio]` section, and
the ESP32 platform reads the variant from `build.variants_dir` + `build.variant`
(the `board_build.*` form), not from a `board_build.variant_dir` path.

**Working configuration** (`platformio.ini`):

```ini
[platformio]
src_dir = firmware/src          ; global — per-env src_dir is ignored

[env:rocket]
framework = arduino             ; must be repeated in each env (see below)
board_build.variant = heltec_wifi_lora_32_V4
board_build.variants_dir = variants   ; parent dir of the variant folder
```

Notes:
- `framework = arduino` in the shared `[env]` section is **not** enough —
  PlatformIO only copies options that are literally present in the `[env:NAME]`
  section into the SCons build environment. Repeat it in each env.
- The ground-station firmware is a **separate PlatformIO project**
  (`ground_station/firmware/platformio.ini`) because one project cannot build
  two different `src_dir` trees.

## 3. The framework `tools/platformio-build.py` dispatcher

The upstream file (arduino-esp32 2.0.17) is 254 lines and does much more than
dispatch to the per-MCU script: it resolves `variants_dir`, appends the variant
to `CPPPATH`, builds the core + variant libraries, and wires bootloader /
partitions / `--elf-sha256-offset`. The file is shipped intact with the
framework — it should not need to be touched again. If `pins_arduino.h` ever
stops resolving, verify this file is the full 254-line upstream version.
