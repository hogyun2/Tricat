import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion

def line_marker(fir, last, name, idd):
    scale = Vector3(0.04, 0.04, 0.04)
    line_mk = Marker()
    line_mk.header.frame_id = "world"
    line_mk.header.stamp = rospy.Time.now()
    line_mk.ns = name
    line_mk.id = idd
    line_mk.type = Marker.ARROW
    line_mk.action = Marker.ADD
    line_mk.pose.position.x = 0.0
    line_mk.pose.position.y = 0.0
    line_mk.pose.position.z = -0.1
    line_mk.pose.orientation.x = 0.0
    line_mk.pose.orientation.y = 0.0
    line_mk.pose.orientation.z = 0.0
    line_mk.pose.orientation.w = 0.0
    line_mk.scale = scale
    line_mk.color.r = 1.0
    line_mk.color.g = 0.0
    line_mk.color.b = 0.0
    line_mk.color.a = 1.0
    line_mk.points.append(fir)
    line_mk.points.append(last)

    return line_mk

    # line_mk.points.append([2.21831,-2.03464])
    # line_mk.points.append([6.50700,-4.43930])
    # line_mk.points.append([5.32394,-6.65894])
    # line_mk.points.append(fir),[2.21831,-2.03464],[6.50700,-4.43930],[5.32394,-6.65894]
    # line_mk.points.append(last)

    return line_mk

rospy.init_node("line", anonymous=True)
line1_pub = rospy.Publisher('line1', Marker, queue_size = 10)
line2_pub = rospy.Publisher('line2', Marker, queue_size = 10)
line3_pub = rospy.Publisher('line3', Marker, queue_size = 10)
line4_pub = rospy.Publisher('line4', Marker, queue_size = 10)
line5_pub = rospy.Publisher('line5', Marker, queue_size = 10)
line6_pub = rospy.Publisher('line6', Marker, queue_size = 10)
line7_pub = rospy.Publisher('line7', Marker, queue_size = 10)
line8_pub = rospy.Publisher('line8', Marker, queue_size = 10)
line9_pub = rospy.Publisher('line9', Marker, queue_size = 10)
line10_pub = rospy.Publisher('line10', Marker, queue_size = 10)
line11_pub = rospy.Publisher('line11', Marker, queue_size = 10)
line12_pub = rospy.Publisher('line12', Marker, queue_size = 10)
line13_pub = rospy.Publisher('line13', Marker, queue_size = 10)


# line2_pub = rospy.Publisher('line2', Marker, queue_size = 10)
# line3_pub = rospy.Publisher('line3', Marker, queue_size = 10)
# line4_pub = rospy.Publisher('line4', Marker, queue_size = 10)
first = Point()
first.x = 1.03524
first.y = -4.25429

second = Point()
second.x = 2.21831
second.y = -2.03464

three = Point()
three.x = 3.40137
three.y = 0.18500

four = Point()
four.x = 4.43661
four.y = 2.40466

five = Point()
five.x = 5.61968
five.y = 4.62431

six = Point()
six.x = 9.90846
six.y = 2.21965

seven = Point()
seven.x = 8.87322
seven.y = 0.000005

eight = Point()
eight.x = 7.69016
eight.y = -2.21964

nine = Point()
nine.x = 6.50700
nine.y = -4.43930

ten = Point()
ten.x = 5.32394
ten.y = -6.65894


# line1 = line_marker("line1", 1, 3.77112, -4.34679, 1, 0, 0)
line1 = line_marker(first, second, "line1", 1)
line2 = line_marker(second, three, "line2", 1)
line3 = line_marker(three, four, "line3", 1)
line4 = line_marker(four, five, "line4", 1)
line5 = line_marker(five, six, "line5", 1)
line6 = line_marker(six, seven, "line6", 1)
line7 = line_marker(seven, eight, "line7", 1)
line8 = line_marker(eight, nine, "line8", 1)
line9 = line_marker(nine, ten, "line9", 1)
line10 = line_marker(ten, first, "line10", 1)
line11 = line_marker(second, nine, "line11", 1)
line12 = line_marker(three, eight, "line12", 1)
line13 = line_marker(four, seven, "line13", 1)
# line2 = line_marker("line2", 2, 4.95421, -2.12715, 0, 1, 0)
# line3 = line_marker("line3", 3, 6.10034, -0.09252, 0, 0, 1)
# line4 = line_marker("line4", 4, 7.20952, 2.31216, 0.5, 0.5, 1)
# while not rospy.is_shutdown():
#     line1_pub.publish(line1)
#     rospy.spin()

while True:
    line1_pub.publish(line1)
    line2_pub.publish(line2)
    line3_pub.publish(line3)
    line4_pub.publish(line4)
    line5_pub.publish(line5)
    line6_pub.publish(line6)
    line7_pub.publish(line7)
    line8_pub.publish(line8)
    line9_pub.publish(line9)
    line10_pub.publish(line10)
    line11_pub.publish(line11)
    line12_pub.publish(line12)
    line13_pub.publish(line13)


# line2_pub.publish(line2)
# line3_pub.publish(line3)
# line4_pub.publish(line4)
