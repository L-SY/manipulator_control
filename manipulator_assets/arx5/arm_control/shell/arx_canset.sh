#!/bin/bash

# 加载用户的环境配置文件
source ~/.bashrc

# 启动 slcand 并设置 CAN 接口
sudo -S slcand -o -f -s8 /dev/arxcan0 can0

# 激活 can0 接口
sudo ifconfig can0 up

