/*

// Getting this information from the camera, I cant find a code for the camera so I
may not be using the correct variables. I'll make it easy to explain what I mean.


// initializing variables

X_pixel_displacement = 0    // the difference between camera view and target object.
Y_pixel_displacement = 0       (ex, if looking at a box, if the whole box is not in view, then there is pixel displacement)  
movement_speed = 0         // how fast the motors move our robot (Titanic) this should be proportional to the motor speed.
                              (ex, if movement speed is 2m/s each motor speed is ... ) motor speed is just how fast the motors will spin. 
                              not how fast the robot will move

X_pixel_displacement = 0
Y_pixel_displacement = 0

// If there is no displacement, then theres no movement speed, and all of the motors are on. The motors are on because we want to keep moving towards the target

if ((X_pixel_displacement) && (Y_pixel_displacement) == 0)
{
  movement_speed == k*(some speed??)  // k is some constant we have determined (wont be a variavle) that will tell us the ratio of motor speed to movement speed
  UFL == (some speed??)               // Up Front Left Motor
  UFR == (some speed??)               // Up Front Right Motor
  UBL == (some speed??)               // Up Back Left Motor
  UBR == (some speed??)               // Up Back Right Motor
  
  DFL == (some speed??)               // Down Front Left Motor
  DFR == (some speed??)               // Down Front Right Motor
  DBL == (some speed??)               // Down Back Left motor
  DBR == (some speed??)               // Down Back Right motor
 }
 
 
 
 // so now how do we tell if the pixel displacement if left, right, up , or down?
 // The camera code comes out tomorrow, but we can do something Axis based.
 // X - horizontal (left and right). A negative X displacement means that the camrea is looking too much to the left of the target
 // Y - vertical (up and down). A negative Y displacement means that the camera is looking somewhere lower than the target
 
 if (X_pixel_displacement > 0)       // deal with X situation (either too much to the right or left)
 {
  
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
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
