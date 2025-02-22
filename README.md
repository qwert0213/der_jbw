# `der_jbw` package

ROS 2 C++ package. [![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)](https://docs.ros.org/en/humble/)

## Packages and build

It is assumed that the workspace is `~/ros2_ws/`.

### Clone the packages

```r
cd ~/ros2_ws/src
```

```r
git clone https://github.com/qwert0213/der_jbw
```

### Build ROS 2 packages

```r
cd ~/ros2_ws
```

```r
colcon build --packages-select der_jbw --symlink-install
```

### Start turtlesim

```r
ros2 run turtlesim turtlesim_node
```

### On a different terminal start drawing the time
```r
ros2 run der_jbw sevensegment
```
