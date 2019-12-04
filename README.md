# PL-VIO
##  Tightly-Coupled Monocular Visual–Inertial Odometry Using Point and Line Features

Compared with point features, lines provide significantly more geometry structure information of the environment. We proposed PL-VIO a tightly-coupled monocular visual-inertial odometry system exploiting
both point and line features. This code runs on **Linux**, and is fully integrated with **ROS**. 

## 1. Prerequisites
1.1 **Ubuntu** and **ROS**
Ubuntu 16.04. ROS Kinetic, [ROS Installation](http://wiki.ros.org/indigo/Installation/Ubuntu)
additional ROS pacakge

```
	sudo apt-get install ros-YOUR_DISTRO-cv-bridge ros-YOUR_DISTRO-tf ros-YOUR_DISTRO-message-filters ros-YOUR_DISTRO-image-transport
```
If you install ROS Kinetic, please update **opencv3** with 
```
    sudo apt-get install ros-kinetic-opencv3
```

1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html), remember to **make install**.

## 2. Build PL-VIO on ROS
Clone the repository and catkin_make:
```
    cd ~/catkin_ws/src
    git clone https://github.com/HeYijia/PL-VIO.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 3. Run with Simulation Dataset

we have provide the simulation data, you can test plvio with it.

```c++
	roslaunch plvio_estimator simdata_fix.launch 
	roslaunch plvio_estimator vins_rviz.launch 
```

green for trajectory, blue for point and line landmarks.

![simdata](doc/image/simdata.png)

Notice: if you want to generate your own data, you can refer to the code below.

```c++
	https://github.com/HeYijia/vio_data_simulation
```



## 4.Performance on EuRoC dataset

### 4.1 Run with feature.bag
Since line detection and matching are time consuming, we can record the feature results from the front-end module as feature.bag and test PL-VIO's back-end module. You can find mh05_feature.bag in config fold and test our system with it: 

    	roslaunch plvio_estimator euroc_fix_offline.launch 
    	roslaunch plvio_estimator vins_rviz.launch 
    	rosbag play -r 5 config/mh05_feature.bag 
![plvio](doc/image/plvio.gif)
### 4.2 Run with EuRoC dataset directly
4.2.1 Download [EuRoC MAV Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). Although it contains stereo cameras, we only use one camera.

4.2.2 Open three terminals, launch the vins_estimator , rviz and play the bag file respectively. Take MH_05 as example

```c++
    roslaunch plvio_estimator euroc_fix_extrinsic.launch 
    roslaunch plvio_estimator vins_rviz.launch 
    rosbag play -r 0.2 YOUR_PATH_TO_DATASET/MH_05_difficult.bag 
```
(If you fail to open vins_rviz.launch, just open an empty rviz, then load the config file: file -> Open Config-> YOUR_VINS_FOLDER/config/vins_rviz_config.rviz)
**Notice: ** Please play your bag with 0.2x speed since the time consuming from line detection.

## 5 Related Papers

- **PL-VIO: Tightly-Coupled Monocular Visual–Inertial Odometry Using Point and Line Features**, Yijia He, Ji Zhao, Yue Guo, Wenhao He and Kui Yuan.

```
@article{he2018pl,
  title={Pl-vio: Tightly-coupled monocular visual--inertial odometry using point and line features},
  author={He, Yijia and Zhao, Ji and Guo, Yue and He, Wenhao and Yuan, Kui},
  journal={Sensors},
  volume={18},
  number={4},
  pages={1159},
  year={2018},
  publisher={Multidisciplinary Digital Publishing Institute}
}
```

*If you use PL-VIO for your academic research, please cite our related papers.*

## 6. Acknowledgements

We use [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) as our base line code. Thanks Dr. Qin Tong, Prof. Shen etc very much.

## 7. Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

We are still working on improving the code reliability. For any technical issues, please contact Yijia He <heyijia_2013@163.com>.