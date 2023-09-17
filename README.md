### 无人机分析程序环境安装步骤

1. 刷机
2. deepstream安装(官网)
3. qt安装
```bash
   sudo apt-get install qt5-default qtcreator -y
```
4. gst-rtsp-server安装 
```bash
   sudo apt-get install libgstrtspserver-1.0-dev
   sudo apt-get install gtk-doc-tools
```
5. dji SDK依赖安装
* ffmpeg安装
```bash
   sudo apt install libavcodec-dev libavformat-dev libavfilter-dev libswresample-dev
```
* libssl安装
```bash
   sudo apt-get install libssl-dev 
```
6. socket.io-cpp安装
```bash
   cmake ./
   make install
```
7. curl安装
```bash
   sudo apt-get install libcurl4-openssl-dev
```
8. opus安装
```bash
   sudo apt-get install libopus-dev libopus0 opus-tools
```
9. gtk and gtk3
```bash
   sudo apt install libcanberra-gtk-module libcanberra-gtk3-module
```
10. liveTool,vehicleAnalysis,analysisClient,rtmpClient程序编译

jetpack5.1.1视频流不显示:
https://forums.developer.nvidia.com/t/jetpack5-1-1-does-not-support-h265-gdr-decoding/258939

###无人机程序打包
1. pack.sh
```bash
   #!/bin/sh
   exe="vehicleAnalysis" 
   des="/home/bx/projects/vehicleAnalysis/bin/"
   deplist=$(ldd $exe|awk '{if (match($3,"/")){printf("%s "),$3}}')
   cp $deplist $des
```
2. 配置动态库路径
```bash
   vim etc/ld.so.conf
   ldconfig
```

###无人机开机自启设置
1. /opt/liveTool/liveTool.sh
```bash
   #!/bin/sh
   export QT_QPA_PLATFORM='offscreen'
   cd /home/bx/projects/liveTool/build/
   ./liveTool 
```

2. /etc/systemd/system/liveTool.service
```bash
   [Unit]
   Description=My terraform_start APP
   After=syslog.target

   [Service]
   ExecStart=/bin/bash /opt/liveTool/liveTool.sh
   SuccessExitStatus=143
   RemainAfterExit=yes

   [Install]
   WantedBy=multi-user.target
```
3. 服务控制
```bash
   systemctl enable liveTool
   systemctl disable liveTool
   systemctl start liveTool
   systemctl stop liveTool
   systemctl status liveTool
```
