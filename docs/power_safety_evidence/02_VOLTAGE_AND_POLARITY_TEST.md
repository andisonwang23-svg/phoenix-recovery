# Voltage and Polarity Test Form

Use a multimeter before connecting the ESP32 or servos. Set the meter to DC
voltage. If the meter is not auto-ranging, use a range above the expected
voltage, such as 20 V DC for a small LiPo or 5 V UBEC output.

Do continuity checks only with power disconnected.

Date:  
Battery ID:  
Charger used:  
Multimeter used:  
Adult reviewer:  

## Battery voltage

| Measurement | Expected | Measured | Pass | Notes |
| --- | --- | --- | --- | --- |
| Battery connector positive to ground | Matches battery cell count and charge state |  |  |  |
| Connector polarity | Red probe on positive gives positive voltage |  |  |  |
| Connector reverse check | Red probe on ground and black on positive gives negative voltage |  |  |  |

## UBEC output before electronics

Measure the UBEC output before plugging it into the ESP32 or servo rail.

| Measurement | Expected | Measured | Pass | Notes |
| --- | --- | --- | --- | --- |
| UBEC output positive to ground | About 5 V |  |  |  |
| UBEC polarity | Red probe on UBEC positive gives positive voltage |  |  |  |
| No-load stability | Voltage does not jump around |  |  |  |

## Continuity with power disconnected

| Check | Expected | Pass | Notes |
| --- | --- | --- | --- |
| Battery positive is not shorted to ground | No beep / high resistance |  |  |
| 5 V rail is not shorted to ground | No beep / high resistance |  |  |
| ESP32 ground and servo ground are common | Beep / low resistance |  |  |
| Sensor ground and ESP32 ground are common | Beep / low resistance |  |  |

## Power on sequence

| Step | Expected result | Actual result | Pass |
| --- | --- | --- | --- |
| Connect battery to UBEC only | No heat, smoke, smell, or unstable voltage |  |  |
| Connect ESP32 power | ESP32 boots |  |  |
| Confirm payload boot | Serial output or ground-board LoRa telemetry reports payload boot |  |  |
| Wait for ground Wi-Fi AP | `PHOENIX-GROUND` appears while the separate ground board is powered |  |  |
| Open ground dashboard | `http://192.168.8.1/` loads |  |  |
| Check payload link | Ground dashboard receives payload LoRa telemetry |  |  |

## Result

Pass or fail:  
Lowest measured battery voltage:  
Lowest measured 5 V rail voltage:  
Fixes required:  
Reviewer initials:  
