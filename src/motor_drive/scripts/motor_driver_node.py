#!/usr/bin/env python3
import rospy
import math
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from serial_imu.msg import Imu_0x91_msg


# 小车固有参数，用于差速转换
wheel_base = 0.088  # 轮距 (m)
wheel_radius = 0.0172  # 轮半径 (m) 

class Encoder:
    def __init__(self):
        self.leftA = 20
        self.leftB = 21
        self.rightA = 16
        self.rightB = 26
        self.left_count = 0
        self.right_count = 0

        GPIO.setup(self.leftA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.leftB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.rightA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.rightB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.leftA, GPIO.RISING, callback=self.left_encoder_callback)
        GPIO.add_event_detect(self.rightA, GPIO.RISING, callback=self.right_encoder_callback)

    def left_encoder_callback(self, channel):
        if GPIO.input(self.leftA):
            if not GPIO.input(self.leftB):
                self.left_count += 1
            else: self.left_count -= 1

    def right_encoder_callback(self, channel):
        if GPIO.input(self.rightA):
            if not GPIO.input(self.rightB):
                self.right_count += 1
            else: self.right_count -= 1

    def get_counts(self):
        return self.left_count, self.right_count

class WheelOdomPublisher:
    def __init__(self, encoder):
        self.encoder = encoder
        self.frame_id = rospy.get_param("~frame_id", "odom")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")
        self.update_rate = rospy.get_param("~update_rate", 0.01)  # 更新频率 (s)
        self.ticks_per_rev = 205  # 编码器每转的脉冲数
        self.m_per_tick = 2.0 * math.pi * wheel_radius / float(self.ticks_per_rev) # 每个脉冲对应的线距离(m)

        self.prev_l = None
        self.prev_r = None
        self.prev_t = None
        self.x = 0.0
        self.y = 0.0

        self.pub_odom = rospy.Publisher("/wheel_odom", Odometry, queue_size=10)

    def update(self, data):
        left_count, right_count = self.encoder.get_counts()
        now = rospy.Time.now()
        current_left = left_count * self.m_per_tick
        current_right = right_count * self.m_per_tick

        if self.prev_t is None:
            self.prev_t, self.prev_l, self.prev_r = now, current_left, current_right
            return

        d_t = (now - self.prev_t).to_sec()
        if d_t <= 0.0:
            return
        d_left = current_left - self.prev_l
        d_right = current_right - self.prev_r
        d_s = (d_left + d_right) * 0.5
        dx = d_s * math.cos(data.eul_y * math.pi / 180.0)  # 转成弧度
        dy = d_s * math.sin(data.eul_y * math.pi / 180.0)  # 转成弧度
        v = d_s / d_t

        self.x += dx
        self.y += dy

        self.prev_t, self.prev_l, self.prev_r = now, current_left, current_right

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = data.quat_x
        odom.pose.pose.orientation.y = data.quat_y
        odom.pose.pose.orientation.z = data.quat_z
        odom.pose.pose.orientation.w = data.quat_w

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = data.gyr_z * math.pi / 180.0  # 转成弧度

        # 协方差：只让融合器“更信任”x/vx，其它不信
        odom.pose.covariance = [0.01,0,0,0,0,0,
                                0,0.01,0,0,0,0,
                                0,0,999,0,0,0,
                                0,0,0,999,0,0,
                                0,0,0,0,999,0,
                                0,0,0,0,0,0.05]
        odom.twist.covariance = [0.01,0,0,0,0,0,
                                0,999,0,0,0,0,
                                0,0,999,0,0,0,
                                0,0,0,999,0,0,
                                0,0,0,0,999,0,
                                0,0,0,0,0,0.02]
        self.pub_odom.publish(odom)    

class MotorDriver:
    def __init__(self):
        self.GPIO_init()        

        self.pwmLeft = GPIO.PWM(self.PWMA, self.pwm_hz)
        self.pwmRight = GPIO.PWM(self.PWMB, self.pwm_hz)
        self.pwmLeft.start(0)
        self.pwmRight.start(0)
        self.motor_stop()  # 初始状态停止
        self.v_max = rospy.get_param("~v_max", 0.5)

    def GPIO_init(self):
        # ---------- GPIO 初始化 ----------
        GPIO.setmode(GPIO.BCM) 
        GPIO.setwarnings(False)
        # 定义电机控制引脚，序号为 BCM 模式
        self.STBY = 25
        self.PWMA = 13
        self.AIN1 = 5
        self.AIN2 = 6
        self.PWMB = 19
        self.BIN1 = 23
        self.BIN2 = 24
        self.pwm_hz = 1000  # PWM 频率       

        GPIO.setup(self.STBY, GPIO.OUT)
        GPIO.setup(self.PWMA, GPIO.OUT)
        GPIO.setup(self.AIN1, GPIO.OUT)
        GPIO.setup(self.AIN2, GPIO.OUT)
        GPIO.setup(self.PWMB, GPIO.OUT)
        GPIO.setup(self.BIN1, GPIO.OUT)
        GPIO.setup(self.BIN2, GPIO.OUT)
        GPIO.output(self.STBY, GPIO.HIGH)

    def motor_stop(self):
        GPIO.output(self.AIN1, GPIO.LOW)
        GPIO.output(self.AIN2, GPIO.LOW)
        GPIO.output(self.BIN1, GPIO.LOW)
        GPIO.output(self.BIN2, GPIO.LOW)
        self.pwmLeft.ChangeDutyCycle(0)
        self.pwmRight.ChangeDutyCycle(0)

    def drive(self, v, w):
        # 左右轮线速度
        v_left  = v - w * wheel_base/2
        v_right = v + w * wheel_base/2
        # 占空比
        duty_left  = 100 * v_left  / self.v_max
        duty_right = 100 * v_right / self.v_max
        # 电机反转判断
        if duty_left > 0:
            GPIO.output(self.AIN1, GPIO.LOW)
            GPIO.output(self.AIN2, GPIO.HIGH)
        else:
            GPIO.output(self.AIN1, GPIO.HIGH)
            GPIO.output(self.AIN2, GPIO.LOW)
            duty_left = - duty_left  # 反转时占空比取正
        if duty_right > 0:
            GPIO.output(self.BIN1, GPIO.LOW)
            GPIO.output(self.BIN2, GPIO.HIGH)
        else:
            GPIO.output(self.BIN1, GPIO.HIGH)
            GPIO.output(self.BIN2, GPIO.LOW)
            duty_right = - duty_right  # 反转时占空比取正
        # 若超出最大速度，按比例缩放
        m = max(duty_left, duty_right)
        if m > 100:
            duty_left  = duty_left  * 100 / m
            duty_right = duty_right * 100 / m
        # 设置占空比
        self.pwmLeft.ChangeDutyCycle(duty_left)
        self.pwmRight.ChangeDutyCycle(duty_right)

class Cmd_vel_Subscriber:
    def __init__(self):
        self.cmd_topic = rospy.get_param("~topic", "/cmd_vel")
        self.cmd_timeout = rospy.get_param("~cmd_timeout", 0.5)
        self.last_cmd_time = None

        self.sub = rospy.Subscriber(self.cmd_topic, Twist, self.cmd_cb, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.02), self.watchdog_cb)  # 50Hz

        self.motor_driver = MotorDriver()

    def cmd_cb(self, msg: Twist):
        self.last_cmd_time = rospy.Time.now()

        v = msg.linear.x    # m/s
        w = msg.angular.z   # rad/s

        # TODO: 把 v,w 转成电机控制器指令（差速/阿克曼等）
        self.send_to_motor(v, w)

    def watchdog_cb(self, _event):
        if self.last_cmd_time is None:
            return
        if (rospy.Time.now() - self.last_cmd_time).to_sec() > self.cmd_timeout:
            # 触发超时，停止小车
            self.motor_driver.motor_stop()
            self.last_cmd_time = None  # 防止持续重复 stop

    def send_to_motor(self, v, w):
        self.motor_driver.drive(v, w)

class imu_subscriber:
    def __init__(self, wheelOdomPublisher):
        self.sub = rospy.Subscriber("imu_0x91_package", Imu_0x91_msg, self.imu_cb, queue_size=10)
        self.wheelOdomPublisher = wheelOdomPublisher

    def imu_cb(self, msg: Imu_0x91_msg):
        data = msg.imu_data
        self.wheelOdomPublisher.update(data)

def main():
    rospy.init_node("motor_driver_node", anonymous=False)
    _ = Cmd_vel_Subscriber()
    encoder = Encoder()
    p = WheelOdomPublisher(encoder)
    __ = imu_subscriber(p)
    rospy.spin()

if __name__ == "__main__":
    main()