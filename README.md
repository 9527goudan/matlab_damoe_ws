# matlab_damoe_ws

## 安装依赖
---bash
 sudo apt-get update 
 sudo apt-get upgrade
 sudo apt-get install gcc g++ gdb make cmake libjsoncpp-dev libboost-all-dev 
---

## 解压文件
---
 sudo uzip xxx.zip
---

## 编译 && 运行
===
 mkdir build
 cd build
 cmake .. && make
 ./matlab_damo
===

## 串口权限
---bash
 sudo chmod 777 /dev/ttyUSB*
---


## 修改串口配置

串口配置从配置文件中修改，修改配置文件后无需重新编译
---bash
 cd ~/matlab_damo_ws/config/serial.json
---



