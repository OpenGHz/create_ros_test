#!/bin/bash
now_dir=$(pwd)
cd ..
if [[ "$1" == *"/"* ]];then
    if [ ! -d "$1" ];then echo -e "\e[1;31mPlease make sure the target dir exists and is absolute path starting with root /.\e[0m";fi
    target_dir=$1
elif [ -z "$1" ];then
    target_name='arm_control'
    target_dir=$(pwd)/${target_name}
else
    target_name=$1
    target_dir=$(pwd)/$1
fi

if [ ! -d "$target_name" ];then
    echo -e "\e[1;31mPlease put this folder together with arm_control in the same dir.\e[0m" && exit 0
fi

mkdir -p airbot_play_ros_test/src/airbot_play_test && cd airbot_play_ros_test || exit 0
ln -s "${target_dir}"/models src/airbot_play_test/models || exit 0
ln -s "${target_dir}"/src src/airbot_play_test/src || exit 0

cd "${now_dir}" || exit 0
cp CMakeLists.txt ../airbot_play_ros_test/src/airbot_play_test/ || exit 0
cp package.xml ../airbot_play_ros_test/src/airbot_play_test/ || exit 0
cp test.cpp ../airbot_play_ros_test/src/airbot_play_test/ || exit 0

echo -e "\e[1;32mNow you can easiliy test your code with ROS in this work space!\e[0m"
