# ROS配置和使用Xbox One无线手柄

环境：Ubuntu16.04 + ROS kinetic

## 安装joy package

joy package为通用的Linux操纵杆提供了ROS驱动，它包括了一个joy_node节点，可以让Linux操纵杆和ROS交互．这个节点发布一个"Joy"消息，包含了操纵杆每一个按钮和轴的当前状态．

安装这个joy package:

```bash
$ sudo apt-get install ros-kinetic-joy
```

## 配置Xbox One无线手柄

安装好后先用usb线将手柄与电脑连接，测试有线连接的情况下，Linux是否能够识别你的游戏手柄．

```bash
$ ls /dev/input/
```

你会看到所有输入设备的列表，如果出现了jsX的话(在我的电脑中显示的是js0)，说明Linux成功识别了你的游戏手柄．

![my_photo](https://wx3.sinaimg.cn/mw1024/007jahDFly1fv6s8n7davj314c0o675m.jpg)

下面对手柄进行测试：

```bash
sudo jstest /dev/input/jsX
```

你会在终端中看到手柄的输出信息，移动手柄的可以看到数据的变化．

```
Axes:  0:  1982  1:   894  2:-32767  3:   392  4:   438  5:-32767  6:     0  7:     0 
Buttons:  0:off  1:off  2:off  3:off  4:off  5:off  6:off  7:off  8:off  9:off 10:off 
```

每个按钮和轴对应的索引可以在[http://wiki.ros.org/joy](http://wiki.ros.org/joy)查到，当然也可以直接试出来．

接下来，要让ROS的节点joy_node可以使用手柄，我们需要更改手柄的权限．

```bash
$ ls -l /dev/input/jsX
```

你会看到跟这样类似的输出：

```
crw-rw-XX-+ 1 root input 13, 0 9月  12 14:45 /dev/input/js0
```

如果XX是rw的话，说明设备配置成功．
如果XX是--或者其他的话，说明没有配置成功，你需要：

```bash
$ sudo chmod a+rw /dev/input/jsX
```

## 启动joy_node节点

要使手柄的数据发布在ROS上，我们需要启动joy package中的joy_node节点．首先，我们需要将设备名这个参数设置为当前的设备名，默认为js0.

```bash
$ roscore
$ rosparam set joy_node/dev "/dev/input/jsX"
```

然后可以启动joy_node这个节点：

```bash
$ rosrun joy joy_node
```

终端的输出如下：

![photo](https://wx1.sinaimg.cn/mw690/007jahDFly1fv6wjbtfgkj314a0pigni.jpg)

然后在新的终端用`rostopic echo`查看joy这个话题的数据：

```bash
rostopic echo joy
```

移动你的手柄，你会看到数据在不断变化，说明你的手柄有线配置成功．

![photo](https://wx1.sinaimg.cn/mw690/007jahDFly1fv6x3xo03jj314g0peq5u.jpg)

## 蓝牙无线连接

打开Ubuntu16.04的蓝牙，找到Xbox Wireless Controller连接．

![bluetooth](https://wx1.sinaimg.cn/mw690/007jahDFly1fv6xeu6b78j31bc0po76k.jpg)

如果蓝牙连上之后马上掉线的话，可以尝试一下步骤：

- 在终端中运行下面的命令

```bash
sudo apt-get update
sudo apt-get install build-essential linux-headers-generic
```

- 下载这个[repo](https://github.com/loimu/rtbth-dkms)的zip文件，解压到桌面

- 将终端的当前工作目录更改为解压的目录

- 在终端运行以下命令

```bash
make
sudo make install
sudo cp -r ~/Desktop/rtbth-dkms /usr/src/rtbth-3.9.3
sudo apt-get install dkms
sudo dkms install rtbth/3.9.3
sudo nano /etc/modules
```

- 修改`/etc/modules`文件，添加这一行

```
rtbth
```

- 退出，重启

## 测试和校准

jstest-gtk是一个可以测试手柄的工具，可以显示哪个按钮和轴被按下，可以校准和重新为手柄设置每个按钮的索引

- 安装jstest-gtk：

```bash
sudo apt-get install jstest-gtk
```

- 在终端输入以下命令，打开jstest-gtk的GUI：

```bash
$ jstest-gtk
```

- 点击属性按钮，可以进行测试

## Xbox One手柄无线控制turtlesim中的小龟

### 在catkin工作空间创建一个package

```bash
$ cd ~/catkin_ws/src
$ catkin_create_pkg learning_joy roscpp turtlesim joy
$ cd ~/catkin_ws/
$ catkin_make
```

### 写一个节点

在learning_joy/src/下创建一个文件turtle_teleop_joy.cpp．

```c++
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

// create the TeleopTurtle class and define the joyCallback function that will take a joy msg
class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;   // used to define which axes of the joystick will control our turtle
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};


TeleopTurtle::TeleopTurtle(): linear_(1), angular_(2)
{
  //  initialize some parameters
  nh_.param("axis_linear", linear_, linear_);  
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  // create a publisher that will advertise on the command_velocity topic of the turtle
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

  // subscribe to the joystick topic for the input to drive the turtle
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);
}


void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;

  // take the data from the joystick and manipulate it by scaling it and using independent axes to control the linear and angular velocities of the turtle
  twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];

  vel_pub_.publish(twist); 
}


int main(int argc, char** argv)
{
  // initialize our ROS node, create a teleop_turtle, and spin our node until Ctrl-C is pressed
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  ros::spin();
}
```

### 编译和运行

在CMakeLists.txt中加入以下行：

```
add_executable(turtle_teleop_joy src/turtle_teleop_joy.cpp)
target_link_libraries(turtle_teleop_joy ${catkin_LIBRARIES})
```

创建一个launch文件夹，并新建一个launch文件turtle_joy.launch．

```
<launch>
 <!-- Turtlesim Node-->
  <node pkg="turtlesim" type="turtlesim_node" name="sim"/>

 <!-- joy node -->
  <node respawn="true" pkg="joy"
        type="joy_node" name="turtle_joy" >
    <param name="dev" type="string" value="/dev/input/jsX" />
    <param name="deadzone" value="0.12" />
  </node>

 <!-- Axes -->
  <param name="axis_linear" value="1" type="int"/>
  <param name="axis_angular" value="0" type="int"/>
  <param name="scale_linear" value="2" type="double"/>
  <param name="scale_angular" value="2" type="double"/>
  <node pkg="learning_joy" type="turtle_teleop_joy" name="teleop"/>
</launch>
```

不能忘了，还要catkin_make一下．

最后一步，launch！

```bash
$ roslaunch learning_joy turtle_joy.launch
```

![photo](https://wx1.sinaimg.cn/mw690/007jahDFly1fv70ey8iyzj314g0o6djr.jpg)

移动游戏手柄，可以看到小龟在移动．

![photo](https://wx4.sinaimg.cn/mw690/007jahDFly1fv70exmpx0j30ds0f8gls.jpg)



### 参考资料

[http://wiki.ros.org/joy](http://wiki.ros.org/joy)

[http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick)

[https://www.quora.com/How-do-I-fix-Ubuntu-16-04-Bluetooth-issues](https://www.quora.com/How-do-I-fix-Ubuntu-16-04-Bluetooth-issues)

[https://jstest-gtk.gitlab.io/](https://jstest-gtk.gitlab.io/)

[http://wiki.ros.org/joy/Tutorials/WritingTeleopNode](http://wiki.ros.org/joy/Tutorials/WritingTeleopNode)
