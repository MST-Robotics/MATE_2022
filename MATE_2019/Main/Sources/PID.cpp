/*

// Getting this information from the camera, I cant find a code for the camera so I
may not be using the correct variables. I'll make it easy to explain what I mean.


// initializing variables

pixel_displacement = 0     // the difference between camera view and target object.
                              (ex, if looking at a box, if the whole box is not in view, then there is pixel displacement) 
movement_speed = 0         // how fast the motors move our robot (Titanic)

// If there is no displacement, then theres no movement speed, and all of the motors are off.
if (pixel_displacement == 0)
{
  movement_speed == 0
  UFL == 0               // Up Front Left Motor
  UFR == 0               // Up Front Right Motor
  UBL == 0               // Up Back Left Motor
  UBR == 0               // Up Back Right Motor
  
  DFL == 0               // Down Front Left Motor
  DFR == 0               // Down Front Right Motor
  DBL == 0               // Down Back Left motor
  DBR == 0               // Down Back Right motor
 }
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
These are just my messy thoughts because who likes notebooks?

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
