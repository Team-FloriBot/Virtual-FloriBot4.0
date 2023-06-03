# Virtual-FloriBot4.0
In diesem Git liegen die Dateien für die Floribot Simulation ab.
Um das Repository zu klonen folgenden Befehl benutzen:

    git clone https://github.com/Team-FloriBot/Virtual-FloriBot4.0.git catkin_ws

Existiert der Ordner bereits, muss er zuerst gelöscht werden. Optional könnte man auch das <catkin_ws> durch einen anderen Zielordnernamen ersetzen.

Um die Daten im Submodul virtual_maize_field herunterzuladen muss man sich im catkin_ws Ordner befinden und der Befehl

    git submodule init
    
ausgeführt werden und anschließend

    git submodule update
    
Es werden dann alle Daten des Submoduls heruntergeladen. 

# Benötigte Software
ROS Noetic: 
    http://wiki.ros.org/noetic/Installation/Ubuntu

ROS Velocity Controllers: 

    sudo apt install ros-noetic-velocity-controllers
    
ROS Ddynamic Reconfigure:

    sudo apt install ros-noetic-ddynamic-reconfigure
    
Gazebo 11: 

    curl -sSL http://get.gazebosim.org | sh
    
Python: 

    sudo apt install python-is-python3
    
XACRO:

    sudo apt install ros-noetic-xacro

Realsense Dependencies:
    https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

# Starten der Simulation
In dieser Anleitung wird davon ausgegangen, dass der Catkin Workspace im Ordner catkin_ws abliegt. <br>
Ein neues Terminal öffnen und folgenden Befehl eingeben:

    cd ~/catkin_ws && catkin_make && source ~/.bashrc 

Nachdem der Befehl eingegeben wurde, wirde der Catkin Workspace über catkin_make erstellt und es erscheinen im Ordner catkin_ws zwei neue Unterordner: "build" und "devel". <br>
Bevor nun die Simulation gestartet werden kann, müssen noch zuerst weitere Abhängigkeiten für das Maize Field installiert werden und eine Welt generiert werden. Die Abhängigkeiten werden mit:

    rosdep install virtual_maize_fiel

Die Welt erstellt man dann beispielsweise mit:

    rosrun virtual_maize_field generate_world.py fre22_task_navigation_mini

Mehr Vorschläge für die Generierung von Welten findet ihr unter: https://github.com/FieldRobotEvent/virtual_maize_field#sample-worlds

Anschließend kann die Simulation mit dem folgendem Befehel gestartet werdeb:

    roslaunch floribot_simulation FloriBot.launch 

Sobald die Simulation läuft, kann mit dem Roboter mit folgendem Befehl eine Geschwindigkeit vorgegeben werden, dazu muss jedoch ein neues Terminal geöffnet werden: 

    rostopic pub /cmd_vel geometry_msgs/Twist -r 10 -- '[0.2, 0.0, 0.0]' '[0.0, 0.0, 0.0]' 
    
Die Simulation kann mit der Tastenkombination Strg+C beendet werden. 
