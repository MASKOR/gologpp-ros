# Prerequisites
## Golog++
```
cd
wget https://github.com/RRL-ALeRT/gologpp-ros/raw/ros2/gologpp.deb
sudo dpkg -i gologpp.deb
rm gologpp.deb
```
## Eclipse
```
cd /opt
sudo wget https://eclipseclp.org/Distribution/Builds/7.1_13/x86_64_linux/eclipse_basic.tgz
sudo mkdir -p eclipse_basic && cd eclipse_basic
sudo tar xpfz ../eclipse_basic.tgz
sudo rm ../eclipse_basic.tgz
sudo ./RUNME # Hit all enters
```
```
echo '
export LD_LIBRARY_PATH=/usr/local/lib:/opt/eclipse_basic/lib/x86_64_linux:$LD_LIBRARY_PATH
export PATH=/opt/eclipse_basic/bin/x86_64_linux:$PATH' >> ~/.bashrc
. ~/.bashrc
```
## To install .gpp VSCode extension
```
wget https://github.com/RRL-ALeRT/gologpp-ros/raw/ros2/gologpp-0.0.1.vsix && code --install-extension gologpp-0.0.1.vsix && rm gologpp-0.0.1.vsix
```

## For Blocksworld example in webots with spot and manipulator, run following commands in terminal one by one. Use separate tab for each.
```
ros2 launch webots_spot spot_launch.py
```
```
ros2 launch webots_spot moveit_launch.py
```
```
ros2 service call /Spot/blocksworld_pose webots_spot_msgs/srv/SpotMotion
ros2 run webots_spot gpp_blocksworld_server
```
```
ros2 launch webots_spot blocksworld_launch.py
```
Profit!
