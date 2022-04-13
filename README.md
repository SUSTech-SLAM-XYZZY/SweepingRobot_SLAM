## How to run?

**需要 ： ** turtlebot3模拟工具包， https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

**可能还需要 : ** turtlebot3全部工具包，https://github.com/ROBOTIS-GIT/turtlebot3.git

以上两个**可能**可以通过apt安装



> 如果代码有更改，请重新跑
>
> ```shell
> sh setup.sh
> ```

**导入环境变量 : **

```shell
source setup.sh
```

## Run

**开启gazebo可视化 : ** 

```shell
roslaunch slam gazebo_world.launch
```

**开启cmd控制 : **

```shell
roslaunch slam cmd_control.launch
```

**开启rviz可视化 : **

```shell
roslaunch slam rviz_setup.launch
```

**开启cartographer + rviz : **

```shell
roslaunch slam cartographer_init.launch
```

**开启wander_bot : **

```shell
roslaunch slam wander_bot.launch
```

