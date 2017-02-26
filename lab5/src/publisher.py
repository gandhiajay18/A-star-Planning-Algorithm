#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    x_up = -50.0
    y_up = -50.0
    while not rospy.is_shutdown():
	        
	x_up = rospy.get_param('x_up')
	y_up = rospy.get_param('y_up')
        #rospy.loginfo(hello_str)
	if x_up != -50.0:      
	    pub.publish(x_up)
	if y_up != -50.0:
	    pub.publish(y_up)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
