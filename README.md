# `otv_bf7` ROS 2 package
Built for a university class.
### Clone the packages
>It is assumed that the workspace is `~/ros2_ws/`.
``` 
cd ~/ros2_ws/src
```
```
git clone https://github.com/Sl4yM4yd4y/otv_bf7
```
## Build this ROS 2 package
>It is assumed that the workspace is still `~/ros2_ws/`.
```
cd ~/ros2_ws
```
```
colcon build --packages-select otv_bf7 --symlink-install
```
## Run this ROS 2 package
<details>
<summary> Don't forget to source before ROS commands.</summary>
source ~/ros2_ws/install/setup.bash
</details>

Follow the order of instructions for optimal use:
>To display the rules of the game in a new overlay window:
```
1. ros2 run otv_bf7 overlay
```
>To draw the track for the game
```
2. ros2 run otv_bf7 draw_node
```
>To gain control over the turtle
```
3. ros2 run otv_bf7 iranyitas
```
**If the turtle doesn't move with 'w','a','s','d' , split the screen, one side wsl/terminal, other side turtlesim, click into the terminal and try the movements again!**

## Structure

## Graph





