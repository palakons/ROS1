# Build a ROS node to *teleop* TurtleBot3 with the keybapord's Arrow keys

**Description**: This tutorial covers how to write a publisher node in python that controls the turtlebot3 with arrow keys.

**Tutorial Level**: BEGINNER


**Author**: palakon.k_s20@vistec.ac.th

**Table of content:**

- [Build a ROS node to *teleop* TurtleBot3 with the keybapord's Arrow keys](#build-a-ros-node-to-teleop-turtlebot3-with-the-keybapords-arrow-keys)
  - [Getting started](#getting-started)
    - [Prerequisites](#prerequisites)
  - [Writing *teleop_key_arrow* Node](#writing-teleop_key_arrow-node)
    - [Go to the package folder](#go-to-the-package-folder)
    - [Write the code](#write-the-code)
      - [Update the *CMakeLists.txt*](#update-the-cmakeliststxt)
  - [*Make* the package](#make-the-package)
  - [Test our new *teleop_key_arrow* Node](#test-our-new-teleop_key_arrow-node)
    - [Test on laptop](#test-on-laptop)
    - [Test on TurtleBot3](#test-on-turtlebot3)
  - [Understanding the code](#understanding-the-code)
  - [Additional Resources](#additional-resources)



## Getting started

### Prerequisites

- Finished the [Building a ROS Package
](http://wiki.ros.org/ROS/Tutorials/BuildingPackages) turtorial, and make sure the *beginner_tutorials* package was reachable by ROS. Type below: 

```shell
roscd beginner_tu [press tab key]
```
if you have the *beginner_tutorials* package inside ROS, command line should auto-complete to the command below

```shell
roscd beginner_tutorials/
```
- Basic skills on Linux CLI (Command Line Interface) is assumed.
The plan is that we are going to add a new node inside the existing package that we just made in the [Building a ROS Package
](http://wiki.ros.org/ROS/Tutorials/BuildingPackages) turtorial. 

This new node, called *teleop_key_arrow*, will publish messages, commanded linear and angular velocity, to the topic */cmd_vel*. Turtlebot3s will listen to */cmd_vel* and traverse accoridnlgy.

This new node is basd on an exising python [*turtlebot3_teleop_key*](https://github.com/ROBOTIS-GIT/turtlebot3/blob/master/turtlebot3_teleop/nodes/turtlebot3_teleop_key) code with small, but tricky, and crucial tweak to make the code works with the arrow keys.

The arrow keys, unlike character keys, is one of the *control keys*. they send out three values with one arrow key press, the first two values, namely 27, and 91, just to tell the software that the coming key is the *contorl key*. We will handle this in this tutorial as well.

Lastly, we will *make* and test our new node on the terminal, and with a Turtlebot3.

The final step will be deep diving into the code we just made and understand them line-by-line.


## Writing *teleop_key_arrow* Node

### Go to the package folder
First, we go to the folder containing *beginner_tutorials* package:

```shell
roscd beginner_tutorials/
cd scripts
```
if you see mesage below:

```shell
bash: cd: scripts: No such file or directory
```

it means the *scripts* folder does not existed yet. If it is the case, simply create one:

```shell
mkdir scripts
cd scripts
```
Under the package folder, *beginner_tutorials* folder, the *scripts* fodler is where python codes for the nodes inside this pakage is located. (In contrast, the C source codes are in the *src* folder.)

### Write the code

We have two files to updates, the python source file, and the *CMakeLists.txt*.

With your favourite text editor, write the code below and save it into *beginner_tutorials/scripts/key_teleop_arrow.py*

```python
#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from geometry_msgs.msg import Twist
import sys
import select
import os
if os.name == 'nt':
    import msvcrt
else:
    import tty
    import termios

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
           (UP)
(LEFT)    (SPACE)    (RIGHT)
           (DOWN)

(UP ARROW)/(DOWN ARROW) : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
(LEFT ARROW)/(RIGTH ARROW) : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

(SAPCE BAR) : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""


def getKey():
    if os.name == 'nt':
        key = 224
        while ord(key) in [224]:
            if sys.version_info[0] >= 3:
                key = msvcrt.getch().decode()
            else:
                key = msvcrt.getch()
        return key

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:

        key = sys.stdin.read(1)
        while ord(key) in [91, 27]:
            key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel, target_angular_vel)


def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input

    return input


def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "burger":
        vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
        vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
        vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel


def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
        vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
        vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
        vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel


if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('turtlebot3_teleop_arrow')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    turtlebot3_model = rospy.get_param("model", "burger")

    status = 0
    target_linear_vel = 0.0
    target_angular_vel = 0.0
    control_linear_vel = 0.0
    control_angular_vel = 0.0

    try:
        print(msg)
        while(1):
            key = getKey()
            if len(key) > 0:
                key_value = ord(key[0])
                # print('  -key',ord(key[0]))
                if key_value == 65:
                    # print("  UP")
                    target_linear_vel = checkLinearLimitVelocity(
                        target_linear_vel + LIN_VEL_STEP_SIZE)
                    status = status + 1
                    print(vels(target_linear_vel, target_angular_vel))
                elif key_value == 66:
                    # print("  DOWN")
                    target_linear_vel = checkLinearLimitVelocity(
                        target_linear_vel - LIN_VEL_STEP_SIZE)
                    status = status + 1
                    print(vels(target_linear_vel, target_angular_vel))
                elif key_value == 68:
                    # print("  LEFT")
                    target_angular_vel = checkAngularLimitVelocity(
                        target_angular_vel + ANG_VEL_STEP_SIZE)
                    status = status + 1
                    print(vels(target_linear_vel, target_angular_vel))
                elif key_value == 67:
                    # print("  RIGHT")
                    target_angular_vel = checkAngularLimitVelocity(
                        target_angular_vel - ANG_VEL_STEP_SIZE)
                    status = status + 1
                    print(vels(target_linear_vel, target_angular_vel))
                elif key == ' ':
                    target_linear_vel = 0.0
                    control_linear_vel = 0.0
                    target_angular_vel = 0.0
                    control_angular_vel = 0.0
                    print(vels(target_linear_vel, target_angular_vel))
                else:
                    if (key == '\x03'):
                        break

                if status == 20:
                    print(msg)
                    status = 0

                twist = Twist()
                twist.linear.x = target_linear_vel
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = target_angular_vel

                pub.publish(twist)

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

```

#### Update the *CMakeLists.txt*
Also, ad the line below to the end of your *beginner_tutorials/CMakeLists.txt*. This will ensure that ROS recognize your new node the ext time you *make* the *beginner_tutorials* package

```
catkin_install_python(PROGRAMS scripts/key_teleop_arrow.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```


## *Make* the package

Go back to *catkin_ws* directory, and make:

```
cd ~/catkin_ws
catkin_make
```
The output should indicate that there is no error and the code is 100% built, otherwise, check carefully that there is no typo in the code above.

## Test our new *teleop_key_arrow* Node
We will test in two ways, on laptop and on the real TurtleBot3.

### Test on laptop

- **Terminal #1**: Bring up ROS MASTER node:
```shell
roscore
```
- **Terminal #2**: Run our new node:
```shell
rosrun beginner_tutorials key_teleop_arrow.py
```
We should see greeting message below:
```
Control Your TurtleBot3!
---------------------------
Moving around:
           (UP)
(LEFT)    (SPACE)    (RIGHT)
           (DOWN)

(UP ARROW)/(DOWN ARROW) : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
(LEFT ARROW)/(RIGTH ARROW) : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

(SAPCE BAR) : force stop

CTRL-C to quit
```

- **Terminal #3**: Monitor */cmd_vel* to confirm the messages published:

```shell
rostopic echo /cmd_vel
```

Then, when you press the UP ARROW, there will be output displayed on the **Terminal #3** below:

```shell
linear: 
  x: 0.01
  y: 0.0
  z: 0.0
angular: 
  x: 0.0
  y: 0.0
  z: 0.0
---
```
and the output in the **Terminal #2** will be below:

```shell
currently:      linear vel 0.01  angular vel 0.0 
```
### Test on TurtleBot3

With the *roscore* runnig from the previous step, *bring up* the TurtleBot3.

First, access to the TurtleBot3 and enter the password:

```shell
ssh pi@<turtlebot IP address>
pi@<turtlebot IP address>'s password: 
```
The default password for TurtleBot3 is *turtlebot*.


Then bring up the TurtleBot3
```shell
roslaunch turtlebot3_bringup turtlebot3_robot.launch 
```

With the **Terminal #2** already running form the last test, try todrive the robot, and see its motion. The video of such test is shown below:

[![Testing key_teleop_arrow on TurtleBot3](https://img.youtube.com/vi/bCpEevklcrg/0.jpg)](https://www.youtube.com/watch?v=bCpEevklcrg "Testing key_teleop_arrow on TurtleBot3")

## Understanding the code


```python
#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
```
ROS Project utilizes BSD 3-clause licensing. It allows redistributions for both the source code, and the binaries, but the original license text must be retained. The 3rd clause mentioned about endorsements.


```python

import rospy
from geometry_msgs.msg import Twist
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
           (UP)
(LEFT)    (SPACE)    (RIGHT)
           (DOWN)

(UP ARROW)/(DOWN ARROW) : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
(LEFT ARROW)/(RIGTH ARROW) : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

(SAPCE BAR) : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""
```
This part imports relavant libraries, and define constants and texts to be used in our software.

```python
def getKey():
    if os.name == 'nt':
      key = 224
      while ord (key) in [224]:
        if sys.version_info[0] >= 3:
          key = msvcrt.getch().decode()
        else:
          key =  msvcrt.getch()
      return key

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:

        key = sys.stdin.read(1)
        while ord(key) in [ 91,27]:
            key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
```
`getKey()` takes output the user-input keys. This was modified from the original code to handle arrow keys by skipping `ESC`, and '[' keys (with values 27, and 91) that the Linux OS sends out as an arrow key is pressed.

This modification is not tested on Windows.

```python
def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)


def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel
```
`vels()` formats the velocity command for display.

`checkLinearLimitVelocity()`, `checkAngularLimitVelocity`, together with `constrain()` limit the velocity for the specific robot models.

These four functions was not modified from the original code.

```python
rospy.init_node('turtlebot3_teleop_arrow')
```
The name of the new node is *turtlebot3_teleop_arrow*.


```python
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
```
A publisher was declared. It publishes to the topic *cmd_vel* which the TurtleBot3 will listen.

Message type is *Twist*, and maximum number of message to keep in the queu, in case of netowrk congestion, is 10.


```python
turtlebot3_model = rospy.get_param("model", "burger")

status = 0
target_linear_vel   = 0.0
target_angular_vel  = 0.0
control_linear_vel  = 0.0
control_angular_vel = 0.0
```
Get the the model name of the TurtleBot3 from the ROS parameters server called `model`. If the parameter is not found, assumed the model is `burger`.

Then initialize the velocities.

```python
print(msg)
    while(1):
        key = getKey()
        if len(key) > 0:
            key_value = ord(key[0])
```
After print the welcome instructions, perform infinite loop and check for the user input. If there is a key, convert the key to the corresponding integer.

```python
if key_value == 65:
    # print("  UP")
    target_linear_vel = checkLinearLimitVelocity(
        target_linear_vel + LIN_VEL_STEP_SIZE)
    status = status + 1
    print(vels(target_linear_vel, target_angular_vel))
elif key_value == 66:
    # print("  DOWN")
    target_linear_vel = checkLinearLimitVelocity(
        target_linear_vel - LIN_VEL_STEP_SIZE)
    status = status + 1
    print(vels(target_linear_vel, target_angular_vel))
elif key_value == 68:
    # print("  LEFT")
    target_angular_vel = checkAngularLimitVelocity(
        target_angular_vel + ANG_VEL_STEP_SIZE)
    status = status + 1
    print(vels(target_linear_vel, target_angular_vel))
elif key_value == 67:
    # print("  RIGHT")
    target_angular_vel = checkAngularLimitVelocity(
        target_angular_vel - ANG_VEL_STEP_SIZE)
    status = status + 1
    print(vels(target_linear_vel, target_angular_vel))
elif key == ' ':
    target_linear_vel = 0.0
    control_linear_vel = 0.0
    target_angular_vel = 0.0
    control_angular_vel = 0.0
    print(vels(target_linear_vel, target_angular_vel))
else:
    if (key == '\x03'):
        break
```
For the key recieved, adjust the velocities accordingly with each arrow key pressed. The node is also counting the number of command recieved.
Pressing a Spacebar will reset the velocities to zeros.
If Ctrl-C is recieved (coded by `'\x03'`), stop the infinite loop.

```python
if status == 20:
    print(msg)
    status = 0
```
If the user command count reaches 20, display the instructions again.


```python
twist = Twist()
twist.linear.x = target_linear_vel
twist.linear.y = 0.0
twist.linear.z = 0.0
twist.angular.x = 0.0
twist.angular.y = 0.0
twist.angular.z = target_angular_vel

pub.publish(twist)
```
As the message type to be published is `Twist`, the variable was declared, and filled in the correct value.

The `twist` is then published to the ROS environment.

```python
except:
    print(e)

finally:
    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)
```
If any part of the code above has an error, print the error message, `e`.

When the infinite loop is finished, either with or without error, zero-`twist` command will be sent out.


## Additional Resources
For additional resources please visit [ROS communities](http://wiki.ros.org/).
