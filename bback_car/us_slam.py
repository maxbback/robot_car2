# slam module
# Collect data from ultrasonic
# create a vector diagram with measurement points
# store furthest away points when new object is covering it, to have an easy recovery point
# Only collect information of object that has been secured, so not far away that are unsure
# marking points if they are a guess like things far away
# so if something is far away, we collect the information to be investigated and verified by looking closer at them

# Try to determine a 90 degree angle to car to reflect the most accurate distance to the object
# one assumption is that most larger things follow a line, like a wall, and smaller can have any shape
#
# matrix map in cm
# x,y = {0 = verified, 1 = fixed, 2 = unverified, 3 = disappeared}
# We also keep track of how often same solid point has been identified
# if more than 5 times we consider it fixed and likely to be a fixed object or something seldom moved
#
# if we also have a second map with less resolution so instead of mm we have 10 cm
# This simplified map could be the main navigation map and also used to validate a point

# Correcting map
# distance is likely to be wrong due to incorrect information of speed
# distance need to be recalculated based on look ahead and look back
# example in first scan of 1 m we find an obstacle, at 0,5 m
# next run we find it at 0,4 m
# has it moved or is distance wrong
#
# Identifying position
# Assumption, follow a wall until a pattern is detected that matches the map
# if multiple patterns in map is recognised as a potential
# if a room is completely square and all walls are equal, matching must be done aginst other things
# but no homes are like that so it should not happen
#

