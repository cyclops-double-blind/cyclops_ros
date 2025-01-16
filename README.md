# cyclops_ros
ROS1 wrapper package for [cyclops](https://github.com/cyclops-double-blind/cyclops).

## Build & installation
The below instruction is tested under Ubuntu 20.04 and ROS noetic.

* Install the prerequisites:
  ``` bash
  $ sudo apt-get update && \
    sudo apt-get install -y libgflags-dev libgoogle-glog-dev
  ```
* Clone the repository:
  ``` bash
  $ mkdir -p ~/catkin_ws/src && cd ~/catkin_ws
  $ git clone https://github.com/cyclops-double-blind/cyclops_ros \
    src/cyclops_ros --recursive
  ```
  Note the `--recursive` argument at the end of the clone command. Since this
  repository contains a git submodule, it is necessary to initialize it. In case
  you omitted this argument while cloning, just manually run
  ```
  $ cd src/cyclops_ros && git submodule update --init --recursive && cd -
  ```
* Build the workspace:
  ``` bash
  $ catkin_make -DCMAKE_BUILD_TYPE=Release -Dcyclops_native_build=yes
  ```

## Usage
See
[cyclops_playground](https://github.com/cyclops-double-blind/cyclops_playground)
for an example usage on the EuRoC-MAV dataset, including our recommended
parameter settings.
