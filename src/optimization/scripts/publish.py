#!/usr/bin/env python3
# license removed for brevity
from tracemalloc import start
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math



new_points = []

def generate(point1, point2):

    distance = math.sqrt(math.pow((point2[1] - point1[1]),2) + math.pow((point2[0] - point1[0]),2)) 

    if distance <= 10:
        step = 5
        for i in range(1, step):
            n_point = (point1[0] + (point2[0]-point1[0])*i/step , point1[1] + (point2[1]-point1[1])*i/step )
            new_points.append(n_point)
    elif distance >= 10 and distance <= 20 :
        step = 10
        for i in range(1, step):
            n_point = (point1[0] + (point2[0]-point1[0])*i/step , point1[1] + (point2[1]-point1[1])*i/step )
            new_points.append(n_point)
    
    new_points.append(point2)
        

def talker():
    pub = rospy.Publisher("goal_points", Path, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    Path_msg = Path()
    # points=[(1.17,2.37), (25.20, 25.52),(13.72, 20.53),(26.64, 13.37),(1.64, 24.88),(-5.45, 6.99),(7.91, -5.22),(18.17, 5.611),(24.31, -4.70)]
    points = [(0,0) , (10 , 0), (30,0), (20 , 10)]

    new_points.append(points[0])

    for i in range(len(points) - 1):
        generate(points[i] , points[i+1])

    print(new_points)

    for i in new_points:
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
