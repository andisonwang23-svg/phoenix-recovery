# PHOENIX LoRa Ground Station

This folder contains firmware for a second Heltec WiFi LoRa 32 V4/V4.3 board.
That second board stays with the computer and acts as the long-range LoRa bridge
to the rocket.

The ground-station board now creates its own Wi-Fi dashboard:

```text
Wi-Fi name: PHOENIX-GROUND
Password: phoenixground
Dashboard: http://192.168.8.1/
```

Use this when the rocket is too far away for the rocket's own Wi-Fi dashboard.
Your computer connects to `PHOENIX-GROUND`, then the second Heltec sends commands
to the rocket using LoRa.

The rocket still controls safety. Remote commands are ignored during powered
ascent, deployment wait, landed, and failsafe. Manual servo commands also expire
quickly if packets stop arriving.

## Flash the ground-station board

From this folder:

```bash
pio run -t upload
```

Then either:

- connect your computer/phone to `PHOENIX-GROUND` and open `http://192.168.8.1/`, or
- open the serial monitor at 115200 baud.

The web page can:

- ping the rocket,
- send neutral,
- send small supervised Servo 1 / Servo 2 commands,
- send a target latitude/longitude,
- disable rocket remote control until rocket reboot,
- show the latest LoRa telemetry heard from the rocket.

## Serial commands

Type one command per line:

```text
help
ping
neutral
servo 0.20 0.00
servo 0.00 0.20
target 37.1234567 -122.1234567
disable
```

Command meanings:

- `ping` tells the rocket “ground station is alive.”
- `neutral` requests neutral brakes.
- `servo <servo1> <servo2>` requests physical Servo 1 and Servo 2 command
  fractions. Keep values small during early tests. The rocket clamps them again.
- `target <lat> <lon>` updates the rocket target over LoRa and saves it.
- `disable` disables LoRa remote control on the rocket until reboot.

## Safety behavior

- Remote servo commands only work in `GUIDED_DESCENT` or `FINAL_APPROACH`.
- Remote servo commands expire after a short timeout.
- If the rocket enters failsafe, the remote cannot override neutral brakes.
- LoRa loss does not crash guidance. It just stops receiving remote commands.
- The ground-station dashboard is not an arming bypass. The rocket still must
  detect launch, apogee, deployment wait, descent, and parafoil stabilization
  before it accepts steering.
- Use small commands first. The current ground-station slider is limited to the
  configured remote command cap.

Use this on an inert recovery module only until the recovery system has been
tested and logged repeatedly.
