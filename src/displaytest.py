#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Pose

def posepub():
    pub = rospy.Publisher('robot1n1/pose', Pose, queue_size=10)
    rospy.init_node('tester', anonymous=True)
    pose  = Pose()
    rate = rospy.Rate(60) # 10hz
    x = 300
    y = 300
    while not rospy.is_shutdown():
        t = rospy.get_rostime()
        ti = t.nsecs
        xo = x + 30*math.sin(2*3.142*ti/1000000000)
        yo = y + 20*math.cos(2*3.142*ti/1000000000)
        pose.position.x = xo
        pose.position.y = yo
        pose.orientation.z = math.tan(2*3.142*ti/2000000000)
        pose.orientation.w =1
        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        posepub()
    except rospy.ROSInterruptException:
        pass