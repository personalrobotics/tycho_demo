import rospy
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

def construct_object_point(frame_id="world"):
  point = PointStamped()
  point.header.stamp = rospy.Time.now()
  point.header.frame_id = frame_id
  return point

def update_point_position(point, x, y, z):
  point.point.x = x
  point.point.y = y
  point.point.z = z

def construct_marker(
    frame_id="world", marker_id=1, marker_type=9, # text
  ):
  marker = Marker()
  marker.header.frame_id = frame_id
  marker.header.stamp = rospy.Time.now()
  marker.id = marker_id
  marker.type = marker_type
  marker.action = Marker.ADD
  marker.pose.orientation.x=0
  marker.pose.orientation.y=0
  marker.pose.orientation.z=0
  marker.pose.orientation.w=1
  marker.scale.x = .005
  marker.scale.y = .005
  marker.scale.z = .03
  marker.color.a = 1.0
  marker.color.r = 1.0
  marker.color.g = 1.0
  marker.color.b = 1.0
  return marker

def update_marker_position(marker, position):
  marker.pose.position.x = position[0]
  marker.pose.position.y = position[1]
  marker.pose.position.z = position[2]

class PointPublisher:
  def __init__(self, topic_name):
    self.publisher = rospy.Publisher(topic_name, PointStamped, queue_size=10)
    self.point = construct_object_point()

  def update(self, x, y, z):
    update_point_position(self.point, x, y, z)
    self.publisher.publish(self.point)

class TextPublisher:
  def __init__(self, topic_name=''):
    self.publisher = rospy.Publisher(topic_name, Marker, queue_size=10)
    self.marker = construct_marker()

  def update(self, position, msg, color=None):
    self.marker.text = msg
    update_marker_position(self.marker, position)
    if color is not None:
      r,g,b = color
      self.marker.color.r = r
      self.marker.color.g = g
      self.marker.color.b = b
    self.publisher.publish(self.marker)
