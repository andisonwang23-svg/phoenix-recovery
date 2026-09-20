# Power Safety Go No Go Review

Complete this before moving from bench testing to any powered inert drop test.
This is not approval for powered rocket flight.

Date:  
Test article:  
Firmware commit or version:  
Reviewer or adult supervisor:  

## Evidence checklist

| Evidence item | Required before go | Evidence location | Pass |
| --- | --- | --- | --- |
| Pre power inspection completed | No unresolved mechanical or wiring hazards |  |  |
| Battery voltage and polarity completed | Correct voltage and polarity verified |  |  |
| UBEC output completed | About 5 V, correct polarity, stable before electronics |  |  |
| Servo load test completed | Servo motion does not reset ESP32 or drop sensors |  |  |
| Battery-only boot verified | Wi-Fi appears without USB power |  |  |
| Failsafe neutral verified | Both servos neutral during failsafe and reboot |  |  |
| Heat check completed | Battery, UBEC, wires, and servos stay cool |  |  |
| Logs readable | Dashboard, serial, or flash log evidence available |  |  |
| Battery restraint verified | Battery cannot move or be touched by sharp/moving parts |  |  |
| Adult review completed | Reviewer agrees evidence is enough for the next inert test |  |  |

## Decision

Choose one:

- [ ] GO for the next powered bench test only.
- [ ] GO for powered inert drop testing only.
- [ ] NO GO. More work is required.

## If no go

Reason:

Required fixes:

Retest required:

## Signoff

Builder:  
Reviewer or adult supervisor:  
Date:  
