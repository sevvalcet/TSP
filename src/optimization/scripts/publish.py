#!/usr/bin/env python3
# license removed for brevity
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def talker():
    pub = rospy.Publisher("goal_points", Path, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    Path_msg = Path()
    points=[(1.17,2.37), (25.20, 25.52),(13.72, 20.53),(26.64, 13.37),(1.64, 24.88),(-5.45, 6.99),(7.91, -5.22),(18.17, 5.611),(24.31, -4.70)]
    for i in points:
        temp_var = PoseStamped()
        temp_var.header.frame_id = "velodyne"
        temp_var.pose.position.x = i[0]
        temp_var.pose.position.y = i[1]
        Path_msg.poses.append(temp_var)

    Path_msg.header.frame_id = "velodyne"

    while not rospy.is_shutdown():
        pub.publish(Path_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
