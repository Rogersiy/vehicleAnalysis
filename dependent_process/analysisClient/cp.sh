#!/bin/sh

exe="rtmpClient" #发布的程序名称

des="/home/bx/projects/rtmpClient/lib/" #你的路径（lib文件夹需要自己创建）

deplist=$(ldd $exe | awk '{if (match($3,"/")){ printf("%s "),$3 } }')

cp $deplist $des
