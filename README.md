# Virtual-FloriBot4.0
In diesem Git-Repository liegen die Dateien für die Simulation von FloriBot 4.0 in einem viertuellen Maisfeld.

Das Repository lässt sich mit den nachfolgenden Befehlen inklusive Submodulen klonen:
```
git clone https://github.com/Team-FloriBot/Virtual-FloriBot4.0.git ~/catkin_ws
```
```
cd ~/catkin_ws
```
```
git submodule init
```
```
git submodule update
```    


# Benötigte Software
**ROS Noetic:**

http://wiki.ros.org/noetic/Installation/Ubuntu

**ROS Velocity Controllers:**
```
sudo apt install ros-noetic-velocity-controllers
```    
**ROS Ddynamic Reconfigure:**
```
sudo apt install ros-noetic-ddynamic-reconfigure
```    
**Gazebo 11:** 
```
curl -sSL http://get.gazebosim.org | sh
```    
**Python:** 
```
sudo apt install python-is-python3
```    
**XACRO:**
```
sudo apt install ros-noetic-xacro
```
**Realsense Dependencies:**

https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

# Bauen des Workspaces
```
cd ~/catkin_ws 
```
```
catkin_make
```    
```
source ~/catkin_ws/devel/setup.bash
```    
Sollte es aufgrund von fehlenden Abhänigkeiten zu Fehlen kommen, können diese mit rosdep in der Regel behoben werden:
```
cd ~/catkin_ws 
```
```
rosdep install --from-paths src --ignore-src -r -y
```
# Starten der Simulation

Zuerst muss eine Welt erstellt werden:
```
rosrun virtual_maize_field generate_world.py fre22_task_navigation_mini
```
Mehr Vorschläge für die Generierung von Welten findet man unter: https://github.com/FieldRobotEvent/virtual_maize_field#sample-worlds

Anschließend kann die Simulation gestartet werden:
```
roslaunch floribot_simulation FloriBot.launch 
```
