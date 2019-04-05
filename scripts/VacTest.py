#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool

rospy.init_node("VacTest", anonymous=True)
vacuum_publisher = rospy.Publisher("/GripItem", Bool, queue_size=1)
status = True

while True:
    msg = Bool()
    msg.data = status
    status = not status
    vacuum_publisher.publish(msg)
    rospy.sleep(10)
