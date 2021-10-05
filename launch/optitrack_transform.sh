#!/bin/zsh

# Updated 2021 Sep 14
# If you run thecalibration and obtained an R that transforms optitrack frame to base, it is probably qx qy qz qw x y z
# We expect the transformation to be specified as " ... x y z qx qy qz qw world optitrack " here.
# Note that the height (z) is usually not optimized, but kept constant
# launcher "tycho_transform" "rosrun tf static_transform_publisher -1.070000, 0.08000, -0.00543 0.000, 0.00, 0.000, 1.0 world optitrack 10"
launcher "tycho_transform" "rosrun tf static_transform_publisher -1.07671013, 0.15280143, -0.00543 0.00398982, -0.00179225, 0.00139171, 0.99998947 world optitrack 10"
