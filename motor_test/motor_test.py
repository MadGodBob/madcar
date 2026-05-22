# motorTest_keyboard.py
# 按住 WASD 移动，松开停止，Q 或 Ctrl+C 退出

import RPi.GPIO as GPIO
import time
import sys
import tty
import termios
import select

# ---------- GPIO 初始化 ----------
GPIO.setmode(GPIO.BCM)

STBY = 25
PWMA = 13
AIN1 = 5
AIN2 = 6
PWMB = 19
BIN1 = 23
BIN2 = 24

GPIO.setup(STBY, GPIO.OUT)
GPIO.setup(PWMA, GPIO.OUT)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(PWMB, GPIO.OUT)
GPIO.setup(BIN1, GPIO.OUT)
GPIO.setup(BIN2, GPIO.OUT)

pwma = GPIO.PWM(PWMA, 300)
pwmb = GPIO.PWM(PWMB, 300)
pwma.start(0)
pwmb.start(0)
GPIO.output(STBY, GPIO.HIGH)

speed = 50

# ---------- 电机控制函数 ----------
def motor_forward():
    GPIO.output(AIN1, GPIO.LOW)
    GPIO.output(AIN2, GPIO.HIGH)
    GPIO.output(BIN1, GPIO.LOW)
    GPIO.output(BIN2, GPIO.HIGH)
    pwma.ChangeDutyCycle(speed)
    pwmb.ChangeDutyCycle(speed)

def motor_backward():
    GPIO.output(AIN1, GPIO.HIGH)
    GPIO.output(AIN2, GPIO.LOW)
    GPIO.output(BIN1, GPIO.HIGH)
    GPIO.output(BIN2, GPIO.LOW)
    pwma.ChangeDutyCycle(speed)
    pwmb.ChangeDutyCycle(speed)

def motor_left():
    GPIO.output(AIN1, GPIO.HIGH)
    GPIO.output(AIN2, GPIO.LOW)
    GPIO.output(BIN1, GPIO.LOW)
    GPIO.output(BIN2, GPIO.HIGH)
    pwma.ChangeDutyCycle(speed)
    pwmb.ChangeDutyCycle(speed)

def motor_right():
    GPIO.output(AIN1, GPIO.LOW)
    GPIO.output(AIN2, GPIO.HIGH)
    GPIO.output(BIN1, GPIO.HIGH)
    GPIO.output(BIN2, GPIO.LOW)
    pwma.ChangeDutyCycle(speed)
    pwmb.ChangeDutyCycle(speed)

def motor_stop():
    GPIO.output(AIN1, GPIO.LOW)
    GPIO.output(AIN2, GPIO.LOW)
    GPIO.output(BIN1, GPIO.LOW)
    GPIO.output(BIN2, GPIO.LOW)
    pwma.ChangeDutyCycle(0)
    pwmb.ChangeDutyCycle(0)

# ---------- 终端设置 ----------
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
tty.setraw(fd)

print("按住 WASD 控制 | 松开停止 | Q / Ctrl+C 退出")
print("当前速度：{}%".format(speed))

try:
    while True:
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == 'q' or key == '\x03':   # \x03 = Ctrl+C
                print("\n退出...")
                break
            elif key == 'w':
                motor_forward()
                print("\r前进    ", end='')
            elif key == 's':
                motor_backward()
                print("\r后退    ", end='')
            elif key == 'a':
                motor_left()
                print("\r左转    ", end='')
            elif key == 'd':
                motor_right()
                print("\r右转    ", end='')
        else:
            motor_stop()

        time.sleep(0.02)

finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    pwma.stop()
    pwmb.stop()
    GPIO.cleanup()
    print("GPIO 已清理，安全退出。")