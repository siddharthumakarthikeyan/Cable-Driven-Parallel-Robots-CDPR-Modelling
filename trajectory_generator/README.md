Packages related to generate the desired trajectory with desired position, velocity, acceleration.

The trajectory parameter is defined by `trajectory.yaml` in sdf folder.

The SDF file is loaded by the launch file directly when using `roslaunch`.

The 5 oders polynomial is implemented to generate one trajectory with updating time 0.01. It is set the same initial pose and desired position is [2,2,1].

The control algorithm is CTC which integrates the quadratic programming optimization method in order to get the feasible tension in cables.
