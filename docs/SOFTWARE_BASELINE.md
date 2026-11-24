# PHOENIX software baseline

Git tag `phoenix-baseline-2026-11-24` records a reviewed development snapshot
with configuration version 10. The Windows, macOS, or Linux host needs Git
and a working PlatformIO installation to restore and build the source.

Start new work from the snapshot without changing the tag.

```sh
git switch -c codex/next-feature phoenix-baseline-2026-11-24
```

The local `phoenix-baseline-2026-11-24.bundle` contains the tag, source, and
commit history. Keep a copy off this computer if you need a separate backup.
To restore a new workspace from the bundle, run this from the folder containing
the bundle.

```sh
git clone --branch phoenix-baseline-2026-11-24 phoenix-baseline-2026-11-24.bundle phoenix-restored
cd phoenix-restored
git switch -c codex/next-feature
```

## Review and verification

The review covered the recent return-phase changes, their control and telemetry
interfaces, GPS sample bookkeeping, flash log I/O, and the log exporter. It
fixed command limits when authority drops, neutral-state resets, reversal
protection across neutral, homing confirmation with unreliable course, and
classification of aborted runs. It also checks flash record lengths and write
counts, prevents status-only GPS traffic from refreshing a position, and
reports the normalized servo output from the PWM pulse the software issued.

PlatformIO Core 6.1.19 was used with the platform and library versions in the
two `platformio.ini` files. Reproduce the host tests and compilation checks
from the repository root.

```sh
pio test -e native
pio run -e rocket -e imu_i2c_scan -e servo_pwm_diag
pio run -d ground_station/firmware -e ground
```

The C++ suite exercises controller behavior with synthetic inputs. The
standalone Python descent model is a simplified legacy example and was excluded
from baseline verification. The reviewed snapshot has no hardware or flight
test results. The guided inert-drop feature, calibration unlocks, and flare
are disabled in the saved configuration.
