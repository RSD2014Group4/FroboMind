import rospy
from geometry_msgs.msg import Pose, Point, Quaternion 


pos = Point(1,2,0)
ori = Quaternion(3,4,5,6)
p = Pose(pos,ori)
print p