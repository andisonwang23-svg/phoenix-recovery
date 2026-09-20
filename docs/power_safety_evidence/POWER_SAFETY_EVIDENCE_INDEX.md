# Power Safety Evidence Index

Use this file as the main pass or fail record. Each row should point to a
completed form, photo, video, log file, or measurement note.

Project: PHOENIX Recovery  
Hardware revision:  
Firmware commit or version:  
Date opened: 2026-09-12  
Reviewer or adult supervisor:  

## Current conclusion

Do not mark the power system as ready until every required item below has real
evidence. If any item is unknown, treat it as not ready.

| Area | Required result | Evidence file or note | Status |
| --- | --- | --- | --- |
| Battery condition | No swelling, cuts, punctures, heat damage, leaking, odor, or crushed corners |  | Not started |
| Battery restraint | Battery cannot slide, rattle, hit screws, hit servo arms, or pull on its own wires |  | Not started |
| Polarity | Positive and ground verified before connection |  | Not started |
| UBEC output | 5 V rail measured before electronics are connected |  | Not started |
| Common ground | ESP32, UBEC, servos, sensors, and LoRa share a correct ground reference |  | Not started |
| ESP32 boot | Wi-Fi turns on from battery power without USB help |  | Not started |
| Sensor power | IMU, barometer, and GPS remain readable while servos move |  | Not started |
| Servo rail | Servo rail stays in the safe range during Servo 1 and Servo 2 movement |  | Not started |
| Brownout behavior | No unexpected reset, or reset is detected and servos return neutral |  | Not started |
| Heat check | Battery, UBEC, wires, connectors, and servos do not get warm during bench test |  | Not started |
| Failsafe neutral | Failsafe and reboot command both servos neutral |  | Not started |
| Log evidence | Test produces readable status/log evidence |  | Not started |

## Go or no go rule

The next test is **NO GO** if any of these are true:

- battery is damaged, hot, swollen, leaking, or smells unusual;
- UBEC voltage is wrong or unstable;
- polarity is uncertain;
- ESP32 Wi-Fi does not boot from battery power;
- sensors disappear when servos move;
- servos twitch continuously or fail to return neutral;
- brownout reset occurs and the cause is not understood;
- a wire, servo arm, line, screw, or payload edge can touch the battery pouch;
- no adult reviewer is present for powered drop testing.

## Evidence attachments

Record file names here after each test.

| Date | Attachment | What it proves | Result |
| --- | --- | --- | --- |
|  |  |  |  |
