#!/bin/zsh

# Updated 2022 May 30
# If you run thecalibration and obtained an R that transforms optitrack frame to base, it is probably qx qy qz qw x y z
# We expect the transformation to be specified as " ... x y z qx qy qz qw world optitrack " here.
# Note that the height (z) is usually not optimized, but kept constant
# launcher "tycho_transform" "rosrun tf static_transform_publisher -1.070000, 0.08000, -0.00543 0.000, 0.00, 0.000, 1.0 world optitrack 10"
launcher "tycho_transform" "rosrun tf static_transform_publisher -0.93919181, 0.16715677, -0.00317523 -0.00162249, 0.00215902, -0.00000002, 0.99999635 world optitrack 10"
