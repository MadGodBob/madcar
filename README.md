# 驱动

##### **下载ubuntu server 20.04.05** https://cdimage.ubuntu.com/releases/20.04.5/release/ubuntu-20.04.5-preinstalled-server-arm64+raspi.img.xz

```
sudo passwd root
sudo apt-get update
sudo apt update
sudo apt-get install -y libopencv-dev python3-opencv python3-pip v4l-utils net-tools libraspberrypi-bin
sudo apt install -y lua5.1 libatopology2 libfftw3-single3 libsamplerate0 alsa-utils fswebcam pi-bluetooth bluez bluez-tools rfkill dkms
sudo pip3 install numpy
```

**分配固定IP地址**

cd /etc/netplan && sudo vim 50-cloud-init.yaml

```
# 修改成固定ip:192.168.137.100
optional: true
            access-points:
                SUPERPC:
                    password: Mind@123
            addresses: [192.168.137.100/24]
            gateway4: 192.168.137.1
            nameservers:
                addresses: [8.8.8.8, 144.144.144.144]
            dhcp4: true
```

**下载raspi-config的deb包**

```
wget http://mirrors.ustc.edu.cn/archive.raspberrypi.org/debian/pool/main/r/raspi-config/raspi-config_20201108_all.deb
```

**安装deb包**

```
mkdir ~/Downloads/ && cd ~/Downloads/ && sudo dpkg -i raspi-config_20201108_all.deb
```

### **相机驱动**

```
sudo vim /boot/config.txt
# 注释掉，并在最后all加上(如果没有)
# camera_auto_detect=1
gpu_mem=128
start_x=1

sudo vim /etc/modules
# 结尾加上
bcm2835-v4l2
```

**查看挂载**

```
df -h
```

![image-20250417194018890](C:\Users\aaaaa\AppData\Roaming\Typora\typora-user-images\image-20250417194018890.png)

**root用户运行**

```
sudo mount /dev/mmcblk0p1 /boot
```

**添加到用户群组（在usename处填自己的用户名）**

```
sudo usermod -aG video pi
```

 **打开raspi-config配置**(运行前一定要sudo mount /dev/mmcblk0p1 /boot)

```
sudo raspi-config
```

选择3 Interface Options回车进入，再回车进入camera，打开即可

 **显示设备列表**

```
v4l2-ctl --list-devices
```

 **重启，查看是否连接上CSI摄像头** 显示supported=1 detected=1 则成功

```
vcgencmd get_camera
```

 **拍照测试

```
fswebcam --no-banner -r 640x480 image_test.jpg
```

 **openCV例程，给主机实时显示图片**

```
import cv2
camera = cv2.VideoCapture(0) #打开摄像头(只有一个摄像头则编号为0，若有2个则依次为0,1)
cv2.namedWindow('Video Cam', cv2.WINDOW_NORMAL) #创建窗口"Video Cam"
i=0
while cv2.waitKey(1)!=27: #esc键 持续间隔1ms等待按键,若有按键跳出循环
      success, frame =camera.read() #读取摄像头数据
      cv2.imshow('Video Cam', frame) # 显示在窗口"Video Cam"上
      if cv2.waitKey(1)==32: #空格键存图像
         i=i+1
         cv2.imwrite(str(i)+".jpg",frame) #存图像
camera.release() #断开摄像头
cv2.destroyAllWindows() #释放所有窗口
```

# **安装ros** 

```
wget http://fishros.com/install -O fishros && . fishros
```

安装noetic最小基础板，然后安装部分包

```
sudo apt install -y ros-noetic-rviz ros-noetic-turtlesim qtbase5-dev libqt5widgets5 ros-noetic-robot-localization ros-noetic-slam-gmapping ros-noetic-slam-karto ros-noetic-map-server
```

新建**工作空间** 

```
mkdir -p ~/racecar/src
cd ~/racecar/src
catkin_init_workspace
cd ~/racecar
catkin_make

cd ~/racecar/src
catkin_create_pkg main roscpp rospy std_msgs

catkin_make --only-pkg-with-deps main
```

**设置环境变量** 应将其加入到~/.bashrc中

```
source ~/racecar/devel/setup.bash
echo "source ~/racecar/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**复制ls01b包**并编译测试

```
ls -l /dev/ttyUSB*
sudo chmod 777 /dev/ttyUSB*
cd ~/racecar
source devel/setup.bash
roslaunch ls01b_v2 ls01b_v2.launch
```

在ls01b_v2.launch加上（以实现静态TF树发布

```
<node pkg="tf2_ros" type="static_transform_publisher" name="base_to_laser"
      args="0 0 0 0 0 0 base_link laser_link" />
```

为了防止karto启动报错LaserRangeScan contains 1440 range readings, expected 1441

在src/ls01x/src/ls01b.cpp找到

```
msg.angle_min = 0.0;
msg.angle_max = 2 * M_PI;
msg.angle_increment = (msg.angle_max - msg.angle_min) / count;
```

修改为

```
msg.angle_min = 0.0;
msg.angle_increment = (2.0 * M_PI) / static_cast<double>(count);
msg.angle_max = msg.angle_min + (count - 1) * msg.angle_increment;
```

**安装serial**

#git clone添加DNS,git失败就重启
sudo vim /etc/hosts

```
192.30.255.112  github.com git 
185.31.16.184 github.global.ssl.fastly.net 
```

```
cd ~/racecar/src && git clone https://github.com/wjwwood/serial.git
cd serial && make
sudo make install
roscd serial
```

**清理编译，这样可以添加新包**

```
cd ~/racecar && rm -rf devel build
```

**IMU**导入包，修改serial_imu.cpp串口波特率。BCM1接imu的TXD1

```
#launch修改为
<arg name="imu_package" default="0x91" doc="package type [spec,0x91,0x62]"/>

roslaunch imu_launch imu_msg.launch
```

添加协方差，在src/serial_imu/src/serial_imu.cpp

```
boost::array<double, 9> covariance_li = {{-1,0,0,0,0,0,0,0,0}};
boost::array<double, 9> covariance_or = {{1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6}};
boost::array<double, 9> covariance_an = {{1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6}};
imu_data->orientation_covariance = covariance_or;
imu_data->angular_velocity_covariance = covariance_an;
imu_data->linear_acceleration_covariance = covariance_li;
```

# 电机驱动

![BCM编码](E:\树莓派项目\BCM编码.jpg)

![电机接线](E:\树莓派项目\电机接线.jpg)

[树莓派4 UART 多串口配置通信 | 树莓派实验室](https://shumeipai.nxez.com/2021/08/09/raspberry-pi-4-activating-additional-uart-ports.html)

安装GPIO库

```
sudo apt-get -y install python3-rpi.gpio
sudo pip3 install --upgrade RPI.GPIO
import RPi.GPIO as GPIO
```

```
# 编码器
spin_count = 0
A = 20
B = 21
GPIO.setup(A, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(B, GPIO.IN, pull_up_down = GPIO.PUD_UP)

def my_callback(channel):
    global spin_count
    if GPIO.input(A):
        if not GPIO.input(B):
            spin_count += 1
        else: spin_count -= 1
    print(spin_count)

GPIO.add_event_detect(A, GPIO.RISING, callback=my_callback)
```

打开其它串口

```
sudo vim /boot/firmware/config.txt

[UART]
dtoverlay=uart2
dtoverlay=uart3
dtoverlay=uart4
dtoverlay=uart5

ls /dev/ttyAMA* 
```

对应关系(GPIO序号是BCM序号)

GPIO14 = TXD0 -> ttyAMA0
GPIO0  = TXD2 -> ttyAMA1
GPIO4  = TXD3 -> ttyAMA2
GPIO8  = TXD4 -> ttyAMA3
GPIO12 = TXD5 -> ttyAMA4

GPIO15 = RXD0 -> ttyAMA0
GPIO1  = RXD2 -> ttyAMA1
GPIO5  = RXD3 -> ttyAMA2
GPIO9  = RXD4 -> ttyAMA3
GPIO13 = RXD5 -> ttyAMA4

修改serial_imu.cpp串口为/dev/ttyAMA1即可从GPIO0/1接收imu信息 ·

测试串口0(短接TX和RX)

```
sudo apt install picocom
sudo picocom -b 115200 /dev/serial0
```

新建**电机驱动包** 

```
cd ~/racecar/src
catkin_create_pkg motor_drive roscpp rospy geometry_msgs std_msgs
cd ~/racecar/ && catkin_make --only-pkg-with-deps motor_drive
roscd motor_drive
mkdir -p scripts launch && touch scripts/motor_driver_node.py && touch launch/motor_driver.launch
```

在motor_driver_node.py写代码（略）

在launch/motor_driver.launch

```
<launch>
  <node pkg="motor_drive" type="motor_driver_node.py" name="motor_driver">
    <param name="cmd_topic" value="/cmd_vel"/>
    <param name="cmd_timeout" value="0.5"/>
  </node>
</launch>
```

在CMakeLists.txt

```
catkin_install_python(PROGRAMS
  scripts/motor_driver_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

启动

```
roslaunch motor_drive motor_driver.launch
```

终端测试

```
timeout 0.5s rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

# EKF融合里程计

```
cd ~/racecar/src/main && mkdir -p scripts launch config && touch config/ekf.yaml && touch launch/ekf.launch
```

在ekf.yaml

```
frequency: 30
sensor_timeout: 0.1
two_d_mode: true

# TF frame 名字
map_frame: map
odom_frame: odom
base_link_frame: base_link
world_frame: odom

publish_tf: true
print_diagnostics: true

# 1) 轮式里程计输入
odom0: /wheel_odom
odom0_queue_size: 10
odom0_differential: false
odom0_relative: false

# 使用 wheel_odom 的哪些字段：
# [x, 	y, 		z, 
#	 roll, 	pitch, 	yaw, 
#	 vx, 	vy, 	vz,  
#	 vroll, vpitch, vyaw, 
#	 ax, 	ay, 	az]
# 这里只使用x y vx
odom0_config: [true,  true, false,
              false, false, false,
              true,  false, false,
              false, false, false,
              false, false, false]

# 2) IMU 输入
imu0: /IMU_data
imu0_queue_size: 50
imu0_differential: false
imu0_relative: false
imu0_remove_gravitational_acceleration: true

# 利用IMU的全部姿态（roll, pitch, yaw）、全部角速度和X/Y轴加速度，但忽略Z轴加速度
imu0_config: [false, false, false,
              true, true, true,
              false, false, false,
              true, true, true,
              false, false, false]
```

在ekf.launch

```
<launch>
  <arg name="debug" default="false"/>

  <!-- 加载 EKF 参数到 ekf_filter_node 命名空间 -->
  <rosparam file="$(find main)/config/ekf.yaml" command="load" />

  <!-- 启动 EKF -->
  <node pkg="robot_localization"
        type="ekf_localization_node"
        name="ekf_filter_node"
        output="screen"
        clear_params="true">
  </node>
</launch>
```

# 使用建图

## Gmapping

```
cd ~/racecar/src/main && touch launch/gmapping.launch && touch config/gmapping.yaml
```

在gmapping.launch

```
<launch>
  <arg name="scan_topic" default="/scan"/>

  <!-- 启动 gmapping，并加载配置 -->
  <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
    <rosparam command="load" file="$(find main)/config/gmapping.yaml" />
    <remap from="scan" to="$(arg scan_topic)"/>
  </node>
</launch>
```

## **karto**

```
cd ~/racecar/src/main && touch launch/karto.launch && touch config/karto.yaml
```



# 上位机配置

##### 安装ubuntu 20.04.06 [repo.huaweicloud.com/ubuntu-releases/20.04.6/](https://repo.huaweicloud.com/ubuntu-releases/20.04.6/)

安装ros

```
wget http://fishros.com/install -O fishros && . fishros
```

## 主从机配置

#### 树莓派配置

sudo chmod 777 /etc/hosts && vim /etc/hosts

```
192.168.137.107 ubuntu20
```

```
echo "export ROS_HOSTNAME=192.168.137.100" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://192.168.137.100:11311" >> ~/.bashrc
source ~/.bashrc
```

#### 虚拟机配置

打开控制面板->网络共享中心->更改适配器设置，删除<传入的连接>

切换成桥接模式，在虚拟机网络编辑器中将VMnet0设置已桥接至电脑的wifi模块

打开虚拟机查看ip，比如192.168.137.107

进入ipv4，配置静态地址，地址和DNS都写成上述地址，子网掩码和网关也填，重启

sudo chmod 777 /etc/hosts && sudo gedit /etc/hosts 

```
192.168.137.100 pi
```

```
echo "export ROS_HOSTNAME=192.168.137.107" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://192.168.137.100:11311" >> ~/.bashrc
source ~/.bashrc
```

# 蓝牙

使用外挂蓝牙模块(树莓派4B的板载蓝牙ubuntu下貌似用不了)

```
sudo vim /boot/firmware/config.txt
# 添加
dtoverlay=disable-bt
```

```
sudo vim /etc/bluetooth/main.conf
# 在[General]下添加
Privacy = device
```

然后

```
sudo systemctl disable --now hciuart.service
sudo systemctl disable --now bluetooth.service
sudo reboot
sudo systemctl enable --now bluetooth.service
sudo systemctl restart bluetooth
bluetoothctl list
```

然后正常蓝牙连接手柄，过程略

### 安装xpadneo驱动

```
cd ~ && git clone -b v0.9.2 https://github.com/atar-axis/xpadneo.git
cd xpadneo && sudo ./install.sh
sudo modprobe hid_xpadneo
# 检查是否成功
lsmod | grep xpadneo
```

### 新建驱动包

```
cd ~/racecar/src
catkin_create_pkg xbox_drive roscpp rospy std_msgs
cd ~/racecar/ && catkin_make --only-pkg-with-deps xbox_drive
roscd xbox_drive
mkdir -p scripts launch && touch scripts/xbox_driver_node.py && touch launch/xbox_driver.launch
sudo chmod 666 scripts/xbox_driver_node.py
```

在xbox_driver.launch

```
<launch>
  <node pkg="xbox_drive" type="xbox_driver_node.py" name="xbox_driver" output="screen">
    <param name="v_max" value="0.5"/>
    <param name="turn_k" value="5.0"/>
  </node>
</launch>
```

