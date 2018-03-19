# path_planning
Model Documentation:

With this documentation I would like to describe my code for the path planning project.
In a first step I took the input from sensor fusion and the ego car’s localization data to
evaluate some properties of the ego car and further cars, so called co-cars. To locate cocars
relative to the ego car, I thought the lane and the distance in the driving direction
should be of largest relevance. With the calculated speed (m/s), s-position (in driving
direction) and lane (0, 1, 2) I define that a car should be flagged as car_ahead if it is
closer than 30m in front of the ego car. Furthermore I flag a car as car_left or car_right, if
it is in the range of 30m in front of or behind the ego car. The rationale behind this is,
that a lane change might be dangerous if a car is within this range.

Those evaluations are taken into account for behavior planning. The car is smoothly
accelerating with 5m/s (0.224 mph / 20ms) up to the speed limit of nearly 50mph. It
keeps this maximum legal speed until it finds a car that is flagged as car_ahead. In this
case it determines if it can undertake a lane change without slowing down. Therefore, it
checks the adjacent lanes 30m in front and behind itself. If those are not free, the ego car
slows down with -2.5m/s until its own speed equals the speed of the car ahead. Having
reached it the ego car follows the speed of the car ahead (Active Cruise Control) while
keeping a safe distance. In the meantime it always checks if a safe lane change becomes
possible in order to move faster. When the car ahead is gone (either because it
accelerated or the ego undertook a lane change) the ego car accelerates smoothly to the
speed limit again.

The ego car follows the described logic by visiting calculated points that lie in front of
the car. Every 20ms the car visits the next point so the distance between the points
determines the speed. Three target points that are equally spaced points in the future
are taken to build up a spline that works well for smooth lane changes without too much
jerk. Meanwhile, the points close by the car are taken as starting reference for the
heading direction. The points of the previous path that are not visited by the car are
reused within the next frame. To fill up the point lane I generate “add on” points which
are equally spaced compared to the existing point lane and in accordance to the spline.
