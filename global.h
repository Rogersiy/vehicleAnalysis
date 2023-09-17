#ifndef GLOBAL_H
#define GLOBAL_H
#include <iostream>
#include <string.h>
#include <set>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <semaphore.h>
#include <fcntl.h>
#include <math.h>
#include <sys/time.h>
#include <QCoreApplication>
#include <QDebug>
#include <QDateTime>
#include <glib.h>
#include <thread>
#include "LoopQueue.h"
#include <pthread.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <qjsonobject.h>
#include <qjsondocument.h>
#include <qjsonarray.h>
#include "dji_fc_subscription.h"
using namespace std;

#define KILL_WAITING_TIME_USEC 10000000    //the waiting time for stoping the analysis process

//共享内存结构体
struct StreamShmMsg
{
    uint8_t data[60000];        //测试得知每一个片段的数据大概在54000左右
    int rtmpUse;
    int analysisUse;
    int len;
};

struct SemMsg
{
    sem_t streamSem;                  //信号量，同步功能(结构体中信号量注意内存对齐问题，强制1字节或2字节内存对齐会导致The futex facility returned an unexpected error code)
    sem_t droneInfoSem;
};

typedef struct Msg
{
    uint8_t* data;
    int len;
    virtual ~Msg()
    {
        if(data != nullptr){
            delete data;
            data = nullptr;
        }
    }
} msg;

typedef struct imgMsg
{
    int height;
    int width;
    int channels;
    cv::Mat img;
} imgmsg;

typedef struct user_meta
{
    string uri;
} user_meta;

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

//无人机相关信息
struct DroneInfo {
    int aircraftType;                    //无人机类型
    string networkOperator;              //网络运营商
    float linkQuality;                   //链路质量
    int signalQuality;                   //信号质量
    int algorithmStatus;                 //算法启用状态
    int flightStatus;                    //飞行状态
    int displayMode;                     //飞行模式
    float latitude;
    float longitude;
    int RTKConnectStatus;
    float relativeHeight;                //对地高度（10m以内）
    float relativeAltitude;              //相对高度 (相对home点, 融合数据)
    float fusedAltitude;                 //融合高度 (相对海平面, 融合数据)
    int visibleSatelliteNumber;
    float velocityX;
    float velocityY;
    float velocityZ;
    int firstBatteryCapacityPercent;
    int firstBatteryVoltage;
    int firstBatteryTemperature;
    int secondBatteryCapacityPercent;
    int secondBatteryVoltage;
    int secondBatteryTemperature;
    int gimbalStatus;
    int gimbalMode;
    float gimbalYaw;
    float gimbalPitch;
    float gimbalRoll;
    string time;
};

//相机相关信息
struct CameraInfo {
    uint8_t cameraType;                                         //相机类型
    string cameraFirmwareVersion;                               //相机固件版本
    bool tapZoomEnabled;                                        //相机指点变焦功能是否可用
    uint8_t tapZoomMultiplier;                                  //相机当前指点变焦倍数
    uint8_t cameraSource;                                       //相机视频流源
    uint8_t cameraWorkMode;                                     //相机工作模式
    uint8_t cameraShootPhotoMode;                               //相机拍照模式
    uint8_t cameraFocusMode;                                    //相机聚焦模式
    float currentOpticalZoomFactor;                             //相机当前变焦倍数
    float maxOpticalZoomFactor;                                 //相机最大变焦倍数
    int cameraRecordStatus;                                     //相机录像任务执行状态
    int cameraZoomStatus;                                       //相机变焦任务执行状态
    int cameraDownloadStatus;                                   //相机下载任务执行状态
};

struct CameraFocusPoint {
    float x;
    float y;
};

struct CameraTapZoomPoint {
    float x;
    float y;
    uint8_t multiplier;
};

struct CameraFileInfo {
    string fileName;
    uint32_t fileSize;          //单位:byte
    string fileIndex;
    string createTime;
    int fileType;
    uint32_t videoDuration;     //视频时长(fileType为2和3时有效)
    /*
    typedef enum {
        DJI_CAMERA_FILE_TYPE_JPEG = 0, < JPEG
        DJI_CAMERA_FILE_TYPE_DNG = 1, < DNG
        DJI_CAMERA_FILE_TYPE_MOV = 2, /*!< MOV
        DJI_CAMERA_FILE_TYPE_MP4 = 3, /*!< MP4
        DJI_CAMERA_FILE_TYPE_UNKNOWN = 255, /*!< 未知类型。
    } E_DjiCameraMediaFileType;
    */
};

struct DownloadFileInfo {
    string fileIndex;
    uint8_t progressInPercent;
    uint8_t downloadFileEvent;
    uint16_t dataLen;
    string data;
};

//飞行控制相关信息
struct DronePositionOffset {
    float x;
    float y;
    float z;
    float yawDesiredInDeg;
};

struct DroneVelocityTimeOffset {
    float x;
    float y;
    float z;
    float yawRate;
    uint32_t timeMs;
};

//航线任务相关信息
struct WayPointActionInfo {
    //actuatorIndex指云台位置
    uint16_t wayPointIndex;                                     //航点
    uint8_t actionActuatorType;                                 //动作类型（1：拍照 2：开始录像 3：停止录像 4：悬停 5: 云台向下 6: 云台回中）---- 在同一航点，拍照和录像动作只能执行一次
    uint8_t hoverTime;                                          //悬停时间（动作类型为4时有效）

};

struct WayPointInfo {
    int index;
    float longitude;
    float latitude;
    float relativeHeight;
};

struct WayPointMissionInfo {
    int taskID;                                                 //任务ID
    int finishAction;                                           //完成动作
    /*
    DJI_WAYPOINT_V2_FINISHED_NO_ACTION = 0,
    DJI_WAYPOINT_V2_FINISHED_GO_HOME = 1,
    DJI_WAYPOINT_V2_FINISHED_AUTO_LANDING = 2,
    DJI_WAYPOINT_V2_FINISHED_GO_TO_FIRST_WAYPOINT = 3,
    DJI_WAYPOINT_V2_FINISHED_CONTINUE_UNTIL_STOP = 4
    */
    int repeatTimes;                                            //重复次数
    float idleVelocity;                                         //巡航速度
    uint16_t goHomeAltitude;                                    //返航高度
    vector<WayPointInfo> wayPoint;
    vector<WayPointActionInfo> wayPointAction;
};

struct WayPointStateInfo {
    int16_t curWaypointIndex;
    uint16_t velocity;
    uint8_t state;
    /*
    DJI_WAYPOINT_V2_MISSION_STATE_UNKNOWN = -1,
    DJI_WAYPOINT_V2_MISSION_STATE_DISCONNECTED = 0,
    DJI_WAYPOINT_V2_MISSION_STATE_READY_TO_EXECUTE = -1,
    DJI_WAYPOINT_V2_MISSION_STATE_EXECUTING = 2,
    DJI_WAYPOINT_V2_MISSION_STATE_INTERRUPTED = 3,
    DJI_WAYPOINT_V2_MISSION_STATE_RESUME_AFTER_INTERRUPTED = 4,
    DJI_WAYPOINT_V2_MISSION_STATE_EXIT_MISSION = 5,
    DJI_WAYPOINT_V2_MISSION_STATE_FINISHED_MISSION = 6,
    */
};

struct WayPointExecuteInfo {
    int taskID;
    int curWaypointIndex;
    string time;
    float latitude;
    float longitude;
    float relativeAltitude;              //相对高度
    float fusedAltitude;                 //融合高度
    float velocityX;
    float velocityY;
    float velocityZ;
    float linkQuality;                   //链路质量
    string networkOperator;              //网络运营商
};

struct WayPointMissionExecuteInfo {
    int wayPointTotalCount;
    int repeatTimes;
    int currentWayPointIndex;
    int currentRepeatTimes;
    string startTime;
    string endTime;
};

struct WayPointMissionReportInfo {
    int taskID;
    int executeResult;
    string exceptionReason;
    string createTime;
    string startTime;
    string endTime;
    int currentWayPointIndex;
    int currentRepeatTimes;
};

struct LogInfo {
    string createTime;
    int logLevel;
    string logDetail;
};


//异常及返回信息
struct ErrorInfo {
    int module;
    string content;
    string createTime;
};

struct ControlReturnInfo {
    int module;             //模块  0:云台  1:相机  2:飞控  3:航线任务
    int type;               //控制类型
    int status;             //状态
    string reason;          //异常原因
};

//// global variable
//信号量,互斥锁
extern SemMsg *g_shared_sem;
extern pthread_mutex_t myMutex;             // 初始化互斥锁
extern pthread_cond_t  cond;                // 初始化条件变量
//视频流
extern LoopQueue<msg*> loopqueue;
extern LoopQueue<imgmsg*> loopqueueImage;
extern StreamShmMsg *g_shared_stream;
//无人机状态信息
extern DroneInfo *g_shared_drone;
//航线
extern WayPointMissionInfo g_wp_mission_info;                   //记录当前的航线信息
extern WayPointMissionExecuteInfo g_wp_mission_exe_info;        //用于飞行报告的生成
//数据推送
extern int g_data_integrity_abnormal_count;
extern int g_cache_size;
//算法
extern bool g_analysis;
//配置文件
extern QString g_serial_number;
extern QString g_network_operator;
extern QString g_mqtt_address;
extern QString g_platform_address;
extern QString g_platform_ip;
extern QString g_platform_data_push_address;
extern QString g_rtmp_client_output;
extern QString g_rtmp_client_process_path;
extern QString g_rtmp_client_address_source;
extern QString g_rtmp_client_address;
extern QString g_analysis_enable;
extern QString g_analysis_algorithm_type;
extern QString g_analysis_client_output;
extern QString g_analysis_client_process_path;
extern QString g_analysis_client_address_source;
extern QString g_analysis_client_address;

class App
{
public:
    static void readConfig();           //读取配置文件,在main函数最开始加载程序载入
    static void writeConfig();          //写入配置文件,在更改配置文件程序关闭时调用
    static bool checkConfig();          //校验配置文件
};
#endif // GLOBAL_H


