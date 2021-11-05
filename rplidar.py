import rospy
from sensor_msgs.msg import LaserScan


# def getMsg(msg):
#     for i in range(len(msg.ranges)):
#         print(msg.ranges[i])

def getMsg(msg):
    # msg.angle_max = 3.0
    angle = []
    for i in range(len(msg.ranges)):
        #if msg.ranges[i] < 1:
        print("point info")
        print(msg.ranges[i])
        print(msg.angle_increment)
        ang = msg.angle_min + i * msg.angle_increment
        angle.append(ang)
        print(i ,angle[i])


rospy.init_node("hogyun", anonymous=True)
pub = rospy.Publisher("lidar_pub", LaserScan, queue_size =10)
#print(self.control_data)
#print('controller').
while not rospy.is_shutdown():
	rospy.Subscriber("/scan", LaserScan, getMsg)
	rospy.spin()

# rospy.Subscriber('Obs', Obs, self.getObstacleMsg)
