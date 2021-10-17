/*
Autonamous section (Eden)
Will be based on the camera and camera feedback
For visual corrections to position and trajectory

There will be a target position or target object the eyes should be locked on
The goal is to have the robot move according to how much of the target object is in view of the camera

Cade mentioned that we can use the amount of pixels to determine how much of the target object is in view
If the target object is too much to the left then the robot will move to the left
If the target object is too much to the right then the robot will move to the right
It will continually do this until the pixel displacement == 0

Also need to take into account how much to adjust the movement based on how off the target object is
*higher pixel displacement = higher motor speed?
 or
 higher pixel displacement = longer motor on time?
    --> *so then we also need to consider pixel conversion to motor movement


*adjusting the movement - (fake code?)

if (target object) is (to the left of camera view)
  robot must move left
    certain motor turns on to make this happen
until the object is not to the left

same for the right, up and down.
*/
