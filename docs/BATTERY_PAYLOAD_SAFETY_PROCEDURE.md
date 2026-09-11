# Electrical Engineering Battery and Payload Safety Procedure

Purpose: prevent LiPo/Li-ion battery cuts, punctures, shorts, overheating,
charging mistakes, and enclosure damage in any electronics project.

This applies to small electrical-engineering builds such as robots, RC vehicles,
rockets, drones, sensor payloads, breadboard prototypes, wearable devices, and
portable test rigs.

## Immediate rule

If a battery is cut, punctured, swollen, hot, leaking, hissing, smoking, sparking,
or smells unusual, stop using it immediately. Do not charge it. Do not reinstall
it. Treat it as damaged hazardous battery waste.

## Main hazards

LiPo and Li-ion batteries are useful because they store a lot of energy in a
small pouch or case. That also means small mistakes can become serious.

The common failure causes are:

- Puncture, slicing, crushing, bending, or impact.
- Short circuits between positive and negative leads.
- Charging with the wrong charger or wrong cell count.
- Charging while damaged.
- Charging unattended or on a flammable surface.
- Overheating from sunlight, motors, regulators, heaters, or enclosed spaces.
- Wires pulling directly on the battery pouch or connector.
- Sharp screws, printed edges, metal parts, or carbon fiber touching the cell.

## Design rules for any project

1. The battery must be mechanically restrained.
2. The battery must be electrically protected from shorts.
3. The battery must be removable or inspectable.
4. Charging must happen outside tight enclosures when practical.
5. The battery must not be used as a structural part.
6. Wires must have strain relief.
7. Sharp edges must be rounded or covered.
8. Metal hardware must not be able to touch battery terminals.
9. A damaged battery is retired, not repaired.
10. If the project is for flight, motion, or impact, use stronger restraint than
    you think you need.

## Required battery mount

Every battery should sit in a tray, cradle, compartment, or padded holder. The
mount should be attached to the project structure, not just stuck to a loose
panel.

The mount should include:

- A smooth bottom surface.
- Foam or rubber padding on the bottom and sides.
- A Velcro strap, fabric strap, zip tie, clamp, or battery door.
- Hard stops so the battery cannot slide forward/backward.
- A smooth guard or cover if wires, tools, screws, or moving parts are nearby.
- A strain-relief tie point for the battery lead.
- Enough room that the pouch is not squeezed or bent.

Foam is padding, not restraint. Tape is a temporary helper, not the main mount.
Hot glue should not be used directly on a LiPo pouch.

## Mechanical inspection checklist

Before powering the project, check:

- Battery cannot rattle, slide, twist, or fall out.
- No screw tips point toward the battery.
- No sharp printed plastic, carbon fiber, metal, or wood touches the pouch.
- No moving part can hit the battery.
- No wire or string can rub through the pouch.
- Battery lead has strain relief.
- Connector can be unplugged without pulling on the pouch.
- Battery can be removed for inspection.
- Enclosure does not crush the battery when closed.

No-go: if the battery can move more than a tiny amount, fix the mount before
testing.

## Electrical inspection checklist

Before powering the project, check:

- Correct battery chemistry and cell count for the charger and electronics.
- Correct polarity: red to positive, black to negative.
- No exposed conductor on the battery lead.
- No loose wire strands.
- No metal tools or screws near exposed terminals.
- Power switch is reachable.
- Fuse, current limit, or protected power path is used when practical.
- Motor/servo power wiring is separated from fragile sensor wiring when
  practical.
- Charger connector cannot short against the frame or enclosure.

No-go: if you are unsure of polarity, do not plug it in until it is verified.

## Charging procedure

1. Remove the battery from the project or open the enclosure enough to inspect it.
2. Inspect the pouch/case, connector, and wires.
3. Confirm the charger matches the battery chemistry and cell count.
4. Place the battery on a nonflammable surface, such as a ceramic plate, concrete
   floor, or metal tray.
5. Keep it away from paper, carpet, bedding, curtains, wood, foam, fuel, and
   loose plastic.
6. Plug the battery into the charger without forcing the connector.
7. Stay nearby while charging.
8. Stop charging if the battery gets warm, swells, smells unusual, or the charger
   behaves strangely.
9. When finished, unplug the charger and battery.
10. Reinstall the battery only after it is cool and passes inspection.

Do not charge a damaged battery. Do not charge a battery buried inside a closed
project box where you cannot see or remove it quickly.

## Storage and transport

- Store batteries partly charged if they will sit unused for a while, following
  the battery/charger instructions.
- Keep batteries away from metal objects.
- Cover or bag loose connectors so they cannot short.
- Do not store batteries in direct sun, hot cars, or near heaters.
- Do not store batteries pressed under tools, screws, or heavy objects.
- Keep damaged batteries separate from good batteries.

## Test procedure before using a new enclosure

Run these checks before the first real test:

1. Install the battery and shake the project gently by hand.
2. Open the enclosure and inspect for rubbing marks.
3. Power the project on while watching the battery and wiring.
4. Move servos, motors, or mechanisms through their full expected range.
5. Confirm nothing touches, pinches, or pulls the battery.
6. Run a short low-power test.
7. Inspect again.
8. Run the full expected load while monitoring heat.
9. Inspect the battery, wires, mount, and connector one more time.

No-go: if the battery shifts, gets warm unexpectedly, or shows marks from rubbing,
stop and redesign the mount.

## Special notes for moving or enclosed projects

For projects that fly, drive, fall, vibrate, or get packed tightly, add extra
protection:

- Use two restraints: for example, a tray plus a strap.
- Add end stops in the direction of acceleration or impact.
- Add a smooth cover over the battery-facing side.
- Keep battery wires away from hinges, servos, gears, wheels, propellers, and
  deployment lines.
- Use a pull handle or access tab attached to structure, not to wires.
- Make the battery removable for charging and inspection.
- Keep heat sources and flame paths away from the battery.

For rockets or ejection-based recovery systems, also use flame-resistant wadding
and a flame blanket between hot gas and the electronics/battery area.

## If the battery is damaged

1. Do not charge it.
2. Do not put it back in the project.
3. Move it to a nonflammable area if it is safe to do so.
4. If it is hot, smoking, hissing, swelling, leaking, or sparking, back away and
   get an adult or responsible supervisor immediately. Call emergency services if
   there is fire or heavy smoke.
5. Do not breathe smoke or fumes.
6. Do not touch leaking liquid. If it contacts skin, rinse with lots of water.
7. Once fully cool and quiet, tape the connector terminals separately with
   electrical tape.
8. Put the battery by itself in a clear plastic bag or non-metal container.
9. Take it to household hazardous waste or a battery recycler that accepts
   damaged lithium batteries.

Do not put damaged lithium batteries in household trash or curbside recycling.

## Design changes to make after any battery incident

- Add or redesign the battery tray.
- Add a strap or battery door.
- Add hard stops.
- Add a smooth guard over the battery-facing side.
- Round or cover every nearby sharp edge.
- Move screws, pin ends, and metal parts away from the battery.
- Add strain relief for the battery lead.
- Make the battery easier to remove and inspect.
- Move charging outside the project enclosure.
- Add a pre-power checklist to the project notes.

## References

- EPA used lithium-ion battery guidance:
  https://www.epa.gov/recycle/used-lithium-ion-batteries
- EPA used household battery guidance:
  https://www.epa.gov/recycle/used-household-batteries
- U.S. Fire Administration lithium-ion battery safety:
  https://www.usfa.fema.gov/a-z/lithium-ion-batteries/
- OSHA battery charging guidance:
  https://www.osha.gov/laws-regs/regulations/standardnumber/1917/1917.157
