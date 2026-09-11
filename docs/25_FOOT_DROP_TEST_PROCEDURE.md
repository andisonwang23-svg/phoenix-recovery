# Phoenix 25 Foot Drop Test Procedure

Purpose: safely test the removable recovery payload from the maximum available
drop height of 25 feet. This procedure checks packing, restraint, battery
protection, parafoil release, line snagging, and post-drop damage.

This is an inert test. Do not use a rocket motor, ejection charge, igniter,
pyrotechnic device, or live launch setup for this procedure.

Use `docs/25_FOOT_DROP_TEST_FORM.md` to record each individual drop.

## What this test can prove

A 25 foot drop can show whether:

- The payload cartridge stays restrained.
- The battery tray, strap, hard stops, and smooth cover protect the LiPo area.
- Wires, servo arms, and parafoil lines do not snag.
- The parafoil can start to release and inflate.
- The payload survives a low-height drop without damage.
- The firmware can log motion and keep servos neutral during a short drop.

## What this test cannot prove

A 25 foot drop is not enough to fully prove autonomous guidance. From 25 feet,
free fall takes only about 1.25 seconds before drag, so the parafoil may not have
enough time to fully inflate, stabilize, turn, and show useful glide behavior.

Do not use this test to claim:

- Final landing accuracy.
- Full parafoil glide ratio.
- Final steering gain.
- Safe flare behavior.
- Powered flight readiness.

## Required safety rules

1. Use an adult supervisor or experienced mentor.
2. Drop only in a clear outdoor area or approved indoor test space.
3. Keep people, pets, vehicles, glass, and fragile objects out of the drop zone.
4. Use a soft landing area such as grass, gym mats, or thick padding.
5. Do not drop over concrete, pavement, tile, stairs, or furniture.
6. Do not stand under the test article.
7. Do not catch the test article. Let it land.
8. Use a dummy battery mass for early tests.
9. Use the real LiPo only after dummy-mass drops pass inspection.
10. Stop immediately if the battery, wiring, payload frame, or parafoil is
    damaged.

## Recommended drop zone

Use a marked drop zone with:

- At least 25 feet of clear height.
- At least 50 feet of clear horizontal area when outdoors.
- A soft landing surface.
- No wind stronger than a light breeze for parafoil release tests.
- One safe release point that does not require leaning over an edge.

Avoid ladders if possible. A stable platform, balcony with a safe railing,
bleachers, or supervised test rig is better. Never climb somewhere unsafe just
to reach 25 feet.

## Required equipment

- Phoenix payload cartridge.
- Rocket payload bay or a same-size test tube section.
- Parafoil and lines.
- Flame blanket and wadding only if testing packing volume, not ejection heat.
- Dummy battery mass matching the real battery size and weight.
- Real LiPo only for later powered logging tests.
- Smooth battery cover installed.
- Retention pins or screws installed.
- Tape or cones to mark the drop zone.
- Phone or camera for slow-motion video.
- Notebook or test log.
- Basic repair tools.
- LiPo-safe storage location or metal tray for battery handling.

## Pre-test inspection

Before each drop, confirm:

- Payload cartridge cannot slide, twist, or rattle hard.
- Battery or dummy battery is strapped in the tray.
- Battery tray has hard stops at both ends.
- Smooth cover is installed and does not squeeze the battery.
- No screw tip, sharp edge, servo horn, or wire can touch the battery pouch.
- Battery lead has strain relief.
- Parafoil lines are separated from wires and servo arms.
- Servo arms cannot hit the battery or payload wall.
- Switch is reachable.
- Retention pins or screws are fully installed.
- Parafoil can be pulled out by hand without snagging.

No-go: if anything rubs, snags, pinches, shifts, or feels sharp, do not drop.

## Firmware configuration

For the first 25 foot drops:

- Keep autonomous guidance disabled.
- Keep flare disabled.
- Keep servos at neutral.
- Use logging only if the payload is powered.
- Do not command steering during the drop.

The first goal is a clean release and a survivable landing. Steering tests need
more altitude or a suspended/tethered test setup.

## Test stages

Complete each stage at least three times before moving on.

| Stage | Payload state | Goal | Pass criteria |
|---|---|---|---|
| 1 | Empty tube or dummy cartridge | Check drop-zone setup and landing surface | Article lands safely inside the marked zone |
| 2 | Cartridge with dummy battery, no power | Check restraint and battery protection | No shifting, cracks, sharp marks, or loose parts |
| 3 | Cartridge plus parafoil, dummy battery, no power | Check packing and line release | Parafoil exits cleanly; no line/wire snag |
| 4 | Real payload with dummy battery, no power | Check final mass layout | Same as Stage 3; no structural damage |
| 5 | Real payload with real LiPo, powered logging only | Check electronics survival and logs | Battery undamaged; system remains powered; log is readable |

Do not skip stages. If any stage fails, fix the cause and repeat that stage
before continuing.

## Drop sequence

1. Assign roles: release person, safety spotter, camera person, and recorder.
2. Clear the drop zone.
3. Read the stage goal and pass criteria out loud.
4. Inspect the payload using the checklist above.
5. Start video recording.
6. If powered, start logging and confirm neutral servos.
7. Hold the test article in the intended orientation.
8. Count down clearly from five.
9. Release cleanly without throwing.
10. Everyone stays clear until the article is fully stopped.
11. Turn off the payload if powered.
12. Photograph the landed state before touching anything.
13. Inspect the payload, battery area, lines, and cartridge.
14. Record pass/fail and notes.

## Data to record

For every drop, record:

- Date and test number.
- Stage number.
- Drop height.
- Wind or indoor conditions.
- Payload mass.
- Battery type: dummy or real.
- Power state: off or logging only.
- Parafoil packed: yes or no.
- Whether the parafoil exited cleanly.
- Whether any line snagged.
- Whether the payload stayed restrained.
- Battery tray and cover condition.
- Any cracks, rubbing marks, wire pulls, or loose screws.
- Video filename.
- Pass or fail.

## Pass criteria for the whole 25 foot program

The 25 foot test program passes only when:

- Three Stage 5 drops pass in a row.
- The battery does not move or show marks.
- The smooth cover stays in place.
- No wire or line touches the LiPo pouch.
- The cartridge remains locked during the drop.
- Retention pins or screws do not loosen.
- Parafoil lines do not snag on electronics, cover, switch, pins, or rails.
- Electronics remain powered during logging drops.
- The log can be read after the drop.
- There is no cracked structure or sharp new damage.

## Fail conditions

Stop testing for the day if any of these happen:

- Battery pouch is cut, dented, swollen, hot, or smells unusual.
- Connector or wire is damaged.
- Payload shifts enough to hit the tube wall hard.
- Retention pin or screw loosens.
- Smooth cover breaks or exposes a sharp edge.
- Parafoil line snags around a servo, wire, switch, screw, or battery area.
- Servo moves when it should be neutral.
- Firmware resets unexpectedly.
- The test article lands outside the safe zone.

Do not continue by saying "it was probably fine." Find the cause, fix it, and
repeat the failed stage.

## After-test actions

After the final drop:

1. Remove the payload cartridge.
2. Inspect the battery or dummy battery area on all sides.
3. Inspect the smooth cover, tray, strap, hard stops, and wire notch.
4. Inspect parafoil lines for fuzzing, cuts, knots, or heat/rub marks.
5. Inspect electronics connectors and servo mounts.
6. Save videos and logs.
7. Add results to the Phoenix project log.
8. Update the design before any higher-risk test.

## Next step after passing 25 feet

If the 25 foot tests pass, the next useful work is not aggressive steering. The
next step is a suspended or tethered test where the parafoil is inflated longer
and the servos can be commanded gently while the payload is restrained and easy
to inspect.

Only after repeated inert tests show clean deployment, stable descent, safe
battery restraint, and reliable logging should the system be considered for a
supervised powered rocket flight.

## References

- NAR Model Rocket Safety Code:
  https://www.nar.org/ModelRocketSafetyCode
- NAR safety resources:
  https://www.nar.org/safety-information/
- U.S. Fire Administration lithium-ion battery safety:
  https://www.usfa.fema.gov/a-z/lithium-ion-batteries/
- EPA used lithium-ion battery guidance:
  https://www.epa.gov/recycle/used-lithium-ion-batteries
