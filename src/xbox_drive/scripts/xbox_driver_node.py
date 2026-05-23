#!/usr/bin/env python3
import struct, select
import rospy
from geometry_msgs.msg import Twist

file = '/dev/input/js0'
turn_trigger_index = 0
forward_trigger_index = 5
back_trigger_index = 2
left_full_sign = -1
forward_full_sign = 1
back_full_sign = 1
full_value = 32767

class XboxTeleop:
    def __init__(self, js):
        self.v_max = rospy.get_param("~v_max", 0.6)
        self.turn_k = rospy.get_param("~turn_k", 60.0)
        self.period = rospy.get_param("~period", 0.02)
        self.js = js
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  
        self.timer = rospy.Timer(rospy.Duration(self.period), self.pub)      
        self.axes = {}

    def read(self):
        time_ms, value, event_type, number = struct.unpack('IhBB', self.js.read(8))
        event_type = event_type & 0x7f
        if event_type == 1:
            # 按键
            self.axes[number] = value
        elif event_type == 2:
            # 扳机和摇杆
            self.axes[number] = value

    def pub(self, event):
        v_forward = self.axes.get(forward_trigger_index, 0) * forward_full_sign
        v_back = self.axes.get(back_trigger_index, 0) * back_full_sign
        v = (v_forward - v_back) / full_value * self.v_max / 2
        turn = self.axes.get(turn_trigger_index, 0) * left_full_sign / full_value * self.turn_k

        t = Twist()
        t.linear.x = v
        t.angular.z = turn
        self.publisher.publish(t)

    def loop(self):
        while not rospy.is_shutdown():
            self.read()

def main():
    with open(file, 'rb') as js:
        rospy.init_node('xbox_teleop')
        teleop = XboxTeleop(js)
        teleop.loop()
        
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
        exit(1)