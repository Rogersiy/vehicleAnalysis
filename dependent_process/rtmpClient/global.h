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

//全局变量
extern StreamShmMsg *g_shared_stream;
extern SemMsg *g_shared_sem;

#endif // GLOBAL_H

