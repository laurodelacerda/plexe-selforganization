# plexe-selforganization
Dinamically creates platoons on the emergence of a road blocking (signs, accidents, blockage)

It adds the module SelfOrganizingApp that identifies road sign shapes in SUMO simulation triggering formation of dynamic platoons.

This application is based on the principle of Self-Organization. Self-Organizing systems tend to keep a structure with a functionality. Examples of self-organizing systems are perceived in [fish schooling](https://en.wikipedia.org/wiki/Shoaling_and_schooling) and [flock swarming](https://en.wikipedia.org/wiki/Swarm_behaviour).

Such as other self-organizing systems, car platoons can perform a valuable function in their environment (i.e. the road system) for both dangerous and harmless scenarios. 

From the dangerous perspective, think of circumnstances that emerge after a car accident. For the harmless case, a simple road narrowing street sign. Both scenarios can indicate a significant change in the road safety and throughput. 

As computerized cars have the power to react much quicker than human drivers, self-organizing autonomous platoons could be a valuable solution for offering safety and good flow to roads.

The SelfOrganizingApp organizes vehicles on a per-lane basis after the detection of a road blockage. Vehicles identify a dangerous scenario and then interact performing maneuvers in order reach a safer structure.

The best maneuver for a given scenario can be chosen based on a criteria: least distance, effective gap usage, first fit gap. Maneuvers applied can be the following:

- Join at Back [available]
- Join at Front [available]
- Join in the Middle [coming soon]
- Split [coming soon]
- Merge [coming soon]
- Multiple Join at Back [coming soon]
- Multiple Join at Front [coming soon]
- Multiple Join in the Middle [coming soon]

[More details coming soon]
