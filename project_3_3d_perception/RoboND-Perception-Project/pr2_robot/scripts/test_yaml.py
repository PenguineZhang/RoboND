import rospy
import yaml

from geometry_msgs.msg import Pose
from std_msgs.msg import String

from rospy_message_converter import message_converter
from std_msgs.msg import Int32
a = Pose()
b = String()
a.position.x = 1
a.position.y = 2
a.position.z = 3
a = message_converter.convert_ros_message_to_dictionary(a)
b.data = 'hello'
print b
b = message_converter.convert_ros_message_to_dictionary(b)
c = Int32()
c.data = 123
c = message_converter.convert_ros_message_to_dictionary(c)
data_dict = [{'a':a, 'b':b}]
data_dict.append([{'c':c}])

data = {'object list': data_dict}
with open('test_yaml.yaml', 'w') as outfile:
    yaml.dump(data, outfile, default_flow_style=False)


