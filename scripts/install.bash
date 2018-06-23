#!/bin/bash

echo "You should have already:"
echo "\t1) Installed NVIDIA drivers and Cuda 8.0"
echo "\t2) Installed ROS Indigo"
echo "Are you sure you want to proceed? (y/N):"
read confirmation

if [ "$confirmation" == "y" ] || [ "$confirmation" == "Y" ]; then
    echo "Proceeding with installation..."
else
    echo "Terminating installation."
    exit
fi

# start at "Code"
cd ~/Code

echo "Installing basics..."
sudo apt-get install git
sudo apt-get install libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler
sudo apt-get install libatlas-base-dev
sudo apt-get install python-dev
sudo apt-get install libgflags-dev libgoogle-glog-dev liblmdb-dev
sudo apt-get install libode-dev
sudo apt-get install libbullet-dev libbullet-extras-dev 
sudo apt-get install libopenni2-dev
sudo apt-get install ros-indigo-dynamixel-controllers 

echo "Replacing occurances of \"james\" with \"csrobot\"..."
#find /home/www -type f -print0 | xargs -0 sed -i 's/subdomainA\.example\.com/subdomainB.example.com/g'

echo "Installing PCL..."
cd ~/Code/pcl-pcl-1.8.0/build
make
sudo make install

echo "Installing OpenSceneGraph..."
cd ~/Code/OpenSceneGraph/build
make
sudo make install

echo "Installing FCL..."
cd ~/Code/fcl/build
make
sudo make install

echo "Installing libccd..."
cd ~/Code/libccd/build
make
sudo make install

echo "Installing gurobi702..."
cd ~/Code/gurobi702/linux64
sudo python setup.py install

echo "Installing InfiniTAM..."
cd ~/Code/InfiniTAM/build
make
sudo make install

echo "Installing trajopt..."
cd ~/Code/trajopt/build
make
sudo make install

echo "Installing openrave..."
make
sudo make install

echo "Installing caffe..."
cd ~/Code/caffe/build
make
sudo make install

echo "----------Finished installing dependencies----------"
echo "Go to the \"arm_wkspc\" and run \"catkin_make\""

