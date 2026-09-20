# Servo Load and Brownout Test Form

This test checks whether servo movement makes the ESP32, Wi-Fi, sensors, GPS,
or LoRa reset or disconnect. Use an inert recovery module only. Keep the servo
horns and brake lines arranged so nothing can strike the battery.

Date:  
Firmware commit or version:  
Battery ID:  
Servo model:  
UBEC model and rating:  
Adult reviewer:  

## Test setup

| Setup item | Required condition | Pass | Notes |
| --- | --- | --- | --- |
| Rocket motor removed | No motor or energetic device installed |  |  |
| Parafoil lines safe | Lines cannot wrap around fingers, battery, or electronics |  |  |
| Battery accessible | Battery can be unplugged quickly |  |  |
| Multimeter connected | Meter probes cannot slip and short anything |  |  |
| Wi-Fi dashboard open | Status page visible before servo motion |  |  |
| Servos neutral at start | Servo 1 and Servo 2 neutral |  |  |

## Static voltage readings

| Condition | Battery voltage | 5 V servo rail | ESP32 5 V input if accessible | Pass | Notes |
| --- | --- | --- | --- | --- | --- |
| Power on, servos neutral |  |  |  |  |  |
| Wi-Fi connected, servos neutral |  |  |  |  |  |
| GPS searching or fixed |  |  |  |  |  |

## Servo movement readings

Run each command at least three times. Record the lowest voltage seen during the
movement, not just the resting voltage after the movement finishes.

| Command | Lowest 5 V rail | Wi-Fi stayed connected | Sensors stayed valid | Reset happened | Servo returned neutral | Pass | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Servo 1 gentle command |  |  |  |  |  |  |  |
| Servo 2 gentle command |  |  |  |  |  |  |  |
| Both servos gentle command |  |  |  |  |  |  |  |
| Servo 1 maximum calibrated bench command |  |  |  |  |  |  |  |
| Servo 2 maximum calibrated bench command |  |  |  |  |  |  |  |
| Both servos maximum calibrated bench command |  |  |  |  |  |  |  |

## Brownout and reboot evidence

| Check | Expected result | Actual result | Pass |
| --- | --- | --- | --- |
| ESP32 reset reason recorded after reboot | Reason is known and written down |  |  |
| Servos on reboot | Both return neutral immediately |  |  |
| Wi-Fi after battery-only reboot | Network appears without USB |  |  |
| Sensors after reboot | IMU and barometer recover if wired |  |  |
| GPS after reboot | NMEA appears, then fix when outdoors long enough |  |  |
| Failsafe command | Both servos neutral |  |  |

## Stop conditions

Stop immediately if any of these happen:

- battery, UBEC, connector, or servo wire gets warm;
- ESP32 resets during normal servo motion;
- Wi-Fi disappears when a servo moves;
- IMU, barometer, GPS, or LoRa repeatedly disappears during servo motion;
- servo twitches continuously;
- servo arm, line, or wire gets close to the battery;
- voltage drops below the safe operating range for the electronics being used.

## Result

Pass or fail:  
Lowest 5 V rail measured:  
Did any reset happen:  
Did any sensor drop out:  
Fixes required before next test:  
Reviewer initials:  
