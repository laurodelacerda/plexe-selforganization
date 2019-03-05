# plexe-signs
Dinamically creates platoons on the emergence of a road blocking (signs, accidents, blockage)

It adds the module SignPlatooningApp that identifies road sign shapes in SUMO simulation triggering formation of dynamic platoons.

Platoons emerge in a per-lane basis after detection of road sign blockage. Cars identify a dangerous scenario and then request for a joining maneuver to attach to vehicle fleet. The best maneuver for a given scenario is chosen based on a criteria: least distance, effective gap usage, first fit gap. Maneuvers applied can be the following:

- Join at Back
- Join at Front
- Join in the Middle
- Split
- Merge
- Multiple Join at Back
- Multiple Join at Front
- Multiple Join in the Middle

[More details coming soon]
