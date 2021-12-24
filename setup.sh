#! /bin/sh
if [ "$ROS_DOT_SETUPED" ];
then
    echo "The package is already setup!!, if you are facing any problem feel free to rise an issue"
else
    echo "alias Fcatkin_make='cd src/Swarm-Bot-Hardware && git pull && cd ../.. && catkin_make'" >> ~/.bashrc
    echo "dtoverlay=disable-bt" >> /boot/config.txt
    echo "enable_uart=1" >> /boot/config.txt
    echo "gpu_mem=16" >> /boot/config.txt
    sudo systemctl disable pigpiod && sudo systemctl disable hciuart
    echo "export ROS_DOT_SETUPED=1" >> ~/.bashrc
    echo "Reboot the raspi now"
fi
