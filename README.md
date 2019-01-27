# Simulation of Soccerbots
<br>
<h2> Instructions to get it running:</h2>
<p> Install ROS kinetic on Ubuntu 16.04 (Developed and tested) </p>
<p> Install pygame, numpy for python 2.7 </p>
<p> Inside location_of_catkin_ws/src/ mkdir sbsim</p>
<p> Inside sbsim clone this repository</p>
<p> go to location_of_catkin_ws and catkin_make</p>

<br>
<h2> Work done </h2>


![Alt](sbnode.jpeg "Node structure")


<h3> Proposed Rules For the game:</h3>
    * Robot's ball possession is legal only for x distance travelled. (x is tbd)
    * If x is reached, bot must pass, shoot or kick ball and regain possession.
    * Incase of collision, bot possesing the ball given freekick
    * if ball goes out outside field of play, possesion is given to the opposition at the line.
