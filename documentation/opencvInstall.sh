#!/bin/bash

echo "starting opencv install"
echo "thanks https://www.pyimagesearch.com/2016/04/18/install-guide-raspberry-pi-3-raspbian-jessie-opencv-3/"
sudo apt-get install build-essential cmake pkg-config -y
sudo apt-get install libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev -y
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev -y
sudo apt-get install libxvidcore-dev libx264-dev -y
sudo apt-get install libgtk2.0-dev -y
sudo apt-get install libatlas-base-dev gfortran -y
echo "now i'll collect the things"
cd ~
wget -O opencv.zip https://github.com/opencv/opencv/archive/3.4.3.zip
unzip opencv.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/3.4.3.zip
unzip opencv_contrib.zip
echo "now i will start building opencv"
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib-3.4.3/modules \
    -D BUILD_EXAMPLES=ON ..
echo "make finished; now attempt to compile with all cores"
echo "***** THIS WILL TAKE AN HOUR OR SO *****************"
make VERBOSE=1
sudo make install
sudo ldconfig