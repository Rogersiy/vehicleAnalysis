#ifndef GLOBAL_H
#define GLOBAL_H
#include <iostream>
#include <string.h>
#include <thread>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <semaphore.h>
#include <fcntl.h>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;

//共享内存结构体
struct StreamShmMsg
{
    uint8_t data[60000];        //测试得知每一个片段的数据最大大概在54000左右
    int rtmpUse;
    int analysisUse;
    int len;
};

struct SemMsg
{
    sem_t streamSem;                  //信号量，同步功能(结构体中信号量注意内存对齐问题，强制1字节或2字节内存对齐会导致The futex facility returned an unexpected error code)
    sem_t droneInfoSem;
};

//目标信息
struct ObjectInfo {
    int id;                         //目标ID
    int classId;                    //类别ID
    double confidence;              //置信度
    string className;               //类别名称
    cv::Rect rect;                  //目标框
};

//目标检测信息
struct DetectionInfo {
    int width;
    int height;
    cv::Mat pic;
    vector<ObjectInfo> objectList;
};

//事件信息
struct EventInfo {
    int taskID;
    int eventType;
    float latitude;
    float longitude;
    string UUID;
    string createTime;
    string eventDescribe;
    string pictureCode;
    cv::Mat picture;
};

//全局变量
extern StreamShmMsg *g_shared_stream;
extern SemMsg *g_shared_sem;
extern string g_platform_event_address;
extern string g_application_path;

#endif // GLOBAL_H

