import rospy
import numpy as np
from visualization_msgs.msg import Marker
from scipy.spatial.transform import Rotation as scipyR
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, Point, Quaternion

from tycho_env.utils import q_mult, q_inv
from tycho_demo_ros.msg import ChopPose

# For visualization purpose, set the half chopsticks length
# (meters)
# No need to be super accurate
HALF_CHOPSTICKS_LENGTH_M = 0.23 / 2

class ChopPosePublisher():
  def __init__(self, topic_name, queue_size=20):
    self.publisher = rospy.Publisher(
      topic_name, ChopPose, queue_size=queue_size)
    self.marker = ChopPose()

  def update(self, transformation, chop_open):
    self.marker.header.stamp = rospy.Time.now()
    x,y,z = np.array(transformation[0:3,3]).reshape(-1)
    qx, qy, qz, qw = scipyR.from_matrix(transformation[:3,:3]).as_quat()
    self.marker.pose = Pose(Point(x,y,z),Quaternion(qx,qy,qz,qw))
    self.marker.open = chop_open
    self.publisher.publish(self.marker)

  def update_vector(self, eepose_vector):
    self.marker.header.stamp = rospy.Time.now()
    x,y,z = eepose_vector[0:3]
    qx, qy, qz, qw = eepose_vector[3:7]
    self.marker.pose = Pose(Point(x,y,z),Quaternion(qx,qy,qz,qw))
    self.marker.open = eepose_vector[7]
    self.publisher.publish(self.marker)

def update_marker_pose(marker, pos, ori, w):
  x,y,z = pos
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  x,y,z = ori
  marker.pose.orientation.x = x;
  marker.pose.orientation.y = y;
  marker.pose.orientation.z = z;
  marker.pose.orientation.w = w;


class ChopPublisher:

  def __init__(self, publishPoints=False):
    self.pub = rospy.Publisher("Chopstick", Marker, queue_size=10)
    self.markerTop = self._make_chop_marker(1)
    self.markerBottom = self._make_chop_marker(2)
    if publishPoints: # For Debug
      self.markerPoints = [
        self._make_point_marker(3),
        self._make_point_marker(4),
        self._make_point_marker(5)]

  def _make_chop_marker(self, my_id):
    marker = Marker()
    marker.header.frame_id = "optitrack"
    # marker.header.stamp = rospy.Time.now()
    marker.ns = "chopstick"
    marker.id = my_id;
    marker.type = marker.MESH_RESOURCE
    marker.action = marker.ADD
    update_marker_pose(marker, (0.,0.,0.), (0.,0.,0.),1.)
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.a = 0.1
    marker.color.r = 0.753
    marker.color.g = 0.753
    marker.color.b = 0.753
    marker.mesh_resource = "package://tycho_description/meshes/chopstick.stl"
    return marker

  def _make_point_marker(self, my_id):
    marker = Marker()
    marker.header.frame_id = "optitrack"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "chopstick"
    marker.id = my_id;
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    update_marker_pose(marker, (0.,0.,0.), (0.,0.,0.),1.)
    marker.scale.x = 0.005
    marker.scale.y = 0.005
    marker.scale.z = 0.005
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.1
    marker.color.b = 0.1
    return marker

  def _set_chopsticks_color(self, color):
    self.markerTop.color = color
    self.markerBottom.color = color

  def updateMarker(self, points):
    chopZero = np.array([HALF_CHOPSTICKS_LENGTH_M, 0, 0])

    # three-points chopstick
    bottomNow = points[0] - points[2]
    bottomNow /= np.linalg.norm(bottomNow)
    bottomPos = points[2] + bottomNow * HALF_CHOPSTICKS_LENGTH_M # center of chopstick
    bottomOriXYZ  = np.cross(chopZero, bottomNow)
    bottomOriW = np.linalg.norm(bottomNow) * np.linalg.norm(chopZero) + np.dot(bottomNow, chopZero)
    update_marker_pose(self.markerBottom, bottomPos, bottomOriXYZ, bottomOriW)

    # two-points chopstick
    topNow = points[3] - points[4]
    topNow /= np.linalg.norm(topNow)
    topPos = points[3] - topNow * (HALF_CHOPSTICKS_LENGTH_M - 0.072) # center of chopstick
    topOriXYZ  = np.cross(chopZero, topNow)
    topOriW = np.linalg.norm(topNow) * np.linalg.norm(chopZero) + np.dot(topNow, chopZero)
    update_marker_pose(self.markerTop, topPos, topOriXYZ, topOriW)

    if not rospy.is_shutdown():
      self.pub.publish(self.markerTop)
      self.pub.publish(self.markerBottom)

  def updateMarkerPoints(self, choppose):
    _p = choppose.pose.position
    _q = choppose.pose.orientation
    _o = choppose.open
    position = np.array([_p.x, _p.y, _p.z])
    rotation_matrix = scipyR.from_quat([_q.x, _q.y, _q.z, _q.w]).as_matrix().T
    x_axis, y_axis, z_axis = rotation_matrix
    # pm = end of chopsticks - 90 mm
    p1 = position + x_axis * 0.138 # Tip of bottom chopsticks on robot
    pm = position + x_axis * 0.013 + y_axis * 0.02372 # EE rotating center
    # note that scipy rotation assumes (x,y,z,w) but we use (w,x,y,z)
    p_delta = p1 - pm
    q1 = np.array([0] + list(p_delta))
    rotate = _o + 0.598 # offset of chopstick open / close. after this, 0 = close
    rx, ry, rz = z_axis
    q2 = np.array([np.cos(rotate/2), rx * np.sin(rotate/2), ry * np.sin(rotate/2), rz * np.sin(rotate/2)])
    qprod = q_mult(q_mult(q2, q1), q_inv(q2))
    p2 = qprod[1:4] + pm
    p_tip = (p1 + p2)/2

    update_marker_pose(self.markerPoints[0], p2, [0,0,0], 1)
    update_marker_pose(self.markerPoints[1], p1, [0,0,0], 1)
    update_marker_pose(self.markerPoints[2], pm, [0,0,0], 1)
    for pt in self.markerPoints:
      self.pub.publish(pt)

  def setChopsticksToGreen(self):
    green = ColorRGBA(0.0, 1.0, 0.0, 1.0)
    self._set_chopsticks_color(green)

  def setChopsticksToRed(self):
    red = ColorRGBA(1.0, 0.0, 0.0, 1.0)
    self._set_chopsticks_color(red)

  def setChopsticksToGrey(self):
    grey = ColorRGBA(0.753, 0.753, 0.753, 1.0)
    self._set_chopsticks_color(grey)

  def setChopsticksColorRGBA(self, r,g,b,a):
    color = ColorRGBA(r,g,b,a)
    self._set_chopsticks_color(color)
