EKF Localization
================

There are three main types of robot localization:

1. **Local (or, position tracking)**: The initial position of the robot is known and the goal is to estimate the position of the robot by accomodating the noise in the sensors and dynamics of the robotic system.
1. **Global**: The initial position of the robot is unknown w.r.t to its environment. The goal is to localize itself and then drop into a position tracking approach. "Approaches to global localization cannot assume boundedness of the pose error." (p. 194)
1. **Kidnapped**: The kidnapped robot problem is a more difficult variant of the global localization problem. In this type of localization, the robot thinks it knows where it is, but then is "kidnapped or teleported" to a different location without informing the robot. The difference is that in global localization, "*the robot knows that it does not know where it is*" (p. 194).

In this homework example, the robot needs to perform local position tracking. Given its initial state of `x = (-5m, -3m, -90deg)` the robot is to localize itself with an EKF w.r.t a `20x20m` field using 3 beacons.
