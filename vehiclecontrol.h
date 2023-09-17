#ifndef VEHICLECONTROL_H
#define VEHICLECONTROL_H

#include <QThread>
#include <QObject>
#include "global.h"
#include "tools.h"
#include <fstream>
#include <iostream>
//psdk
#include <math.h>
#include "application.hpp"
#include <utils/util_misc.h>
#include "dji_typedef.h"
#include "dji_aircraft_info.h"
#include "dji_fc_subscription.h"
#include "dji_logger.h"
#include "dji_platform.h"
#include "dji_liveview.h"
#include "dji_gimbal_manager.h"
#include "dji_camera_manager.h"
#include "dji_flight_controller.h"
#include "flightControl.h"
#include "dji_waypoint_v2.h"
#include "dji_power_management.h"

/* Private constants ---------------------------------------------------------*/
#define FC_SUBSCRIPTION_TASK_FREQ         (1)
#define FC_SUBSCRIPTION_TASK_STACK_SIZE   (2048)

using namespace std;

class VehicleControl: public QObject
{
    Q_OBJECT //The Q_OBJECT macro must appear in the private section of a class definition that declares its own signals and slots or that uses other services provided by Qt's meta-object system
public:
    VehicleControl();
    ~VehicleControl();

public Q_SLOTS:
    //init
    void slotInitVehicle();
    void slotDeInitVehicle();
    //flightControl
    void slotFlightControlTakeoff();
    void slotFlightControlLanding();
    void slotFlightControlConfirmLanding();
    void slotFlightControlForceLanding();
    void slotFlightCancelLanding();
    void slotFlightControlGoHome();
    void slotFlightControlGoHomeAndConfirmLanding();
    void slotFlightControlGoHomeAndConfirmLandingNative();
    void slotFlightCancelGoHome();
    void slotFlightControlMoveByPositionOffset(DronePositionOffset);
    void slotFlightControlVelocityAndYawRateCtrl(DroneVelocityTimeOffset);
    void slotFlightPositionTest();
    void slotFlightVelocityTest();
    //wayPoint
    void slotWayPointMissionTest();
    void slotRunWayPointMission(WayPointMissionInfo wayPointMission);
    void slotUploadWayPointMission(WayPointMissionInfo wayPointMission);
    void slotStartWayPointMission();
    void slotStopWayPointMission();
    void slotPauseWayPointMission();
    void slotResumeWayPointMission();
    //gimbal
    void slotResetGimbal();
    void slotRotateGimbalPitchPositive(int angle);
    void slotRotateGimbalPitchNegative(int angle);
    void slotRotateGimbalYawPositive(int angle);
    void slotRotateGimbalYawNegative(int angle);
    //camera
    void slotChangeLiveViewCameraSource(string cameraSourceName);
    void slotShootSinglePhoto();
    void slotShootAEBPhoto(int count);          //count 3 5 7
    void slotChangeCameraWorkMode(int workMode);
    void slotStartRecordVideo();
    void slotStopRecordVideo();
    void slotStartContinuousZoom(int direction);    //0:out, 1:In  开始连续变焦后一定要停止变焦，否则会出问题
    void slotStopContinuousZoom();
    void slotSetOpticalZoomParam(float factor);
    void slotSetFoucsPoint(CameraFocusPoint);
    void slotSetTapZoomPoint(CameraTapZoomPoint);
    void slotSetFoucsAuto();
    void slotGetDownloadFileList();
    void slotDeleteFile(uint32_t fileIndex);
    void slotDownloadFile(uint32_t fileIndex);

Q_SIGNALS:
    void sendDroneInfo(DroneInfo);
    void sendCameraInfo(CameraInfo);
    void sendCameraFileList(vector<CameraFileInfo>);
    void sendCameraFileData(DownloadFileInfo);
    void sendWayPointStateInfo(WayPointStateInfo);
    void sendWayPointExecuteInfo(WayPointExecuteInfo);
    void sendWayPointMissionReportInfo(WayPointMissionReportInfo);
    void sendLogInfo(LogInfo);
    void sendControlReturnInfo(ControlReturnInfo);
    void sendAnalysisDeInit();
    void sendDeviceAbnormal(ErrorInfo);
    void sendStreamInitSuccess();

private:
    static VehicleControl *static_vehicleControl;
    QThread thread_;
    //共享内存
    void *sem = NULL;        //共享存储段连接的实际地址
    int semid;               //共享内存标识符
    void *shm = NULL;        //共享存储段连接的实际地址
    int shmid;               //共享内存标识符
    void *shmdrone = NULL;        //共享存储段连接的实际地址
    int shmdroneid;               //共享内存标识符
    //无人机信息
    bool psdkInitFlag;                                  //psdk初始化标志
    E_DjiMountPosition mountPosition;                   //主云台位置(相机所在云台)
    bool infoFlag;                                          //无人机信息获取标志
    pthread_mutex_t vehicleInfoMutex;                       //初始化互斥锁
    DroneInfo droneInfo;                                    //无人机信息
    //无人机相机信息
    bool cameraFlag;                                            //相机信息获取标志
    pthread_mutex_t cameraInfoMutex;                            //初始化互斥锁
    CameraInfo cameraInfo;                                      //相机信息
    E_DjiCameraType cameraType;                                 //相机类型
    E_DjiLiveViewCameraSource cameraSource;                     //相机视频流源
    int cameraRecordStatus;                                     //相机录像任务执行状态
    int cameraZoomStatus;                                       //相机变焦任务执行状态
    int cameraDownloadStatus;                                   //相机下载任务执行状态
    DownloadFileInfo* downloadFileInfoCache;                    //相机文件下载缓存
    int downloadFileInfoCacheCount;                             //相机文件下载缓存片段数量
    int downloadFileInfoFirstCacheStatus;                       //相机文件下载是否为第一个缓存片段
    long int cameraDownloadTid;                                 //记录当前下载任务正在进行的TID
    std::thread cameraDownloadThread;                           //无人机的上传下载任务只允许有一个正在进行
    static T_DjiReturnCode cameraDownloadFileCallback(T_DjiDownloadFilePacketInfo packetInfo, const uint8_t *data, uint16_t dataLen);
    ofstream  writeFile;
    //无人机视频流相关
    bool streamFlag;                                            //视频流获取标志
    bool startCameraStream();
    bool stopCameraStream();
    static void liveViewH264Cb(E_DjiLiveViewCameraPosition position, const uint8_t *buf, uint32_t bufLen);
    //无人机数据获取相关
    bool getDroneInfo();
    bool getCameraInfo();
    void getLaserRangingInfo();
    //无人机控制相关
    long int flightControlTid;                                         //记录当前正在进行的TID
    std::thread flightControlThread;                                   //无人机的飞行控制任务只允许有一个正在进行
    void flightControlTakeoff();
    void flightControlLanding();
    void flightControlForceLanding();
    void flightCancelLanding();
    void flightControlGoHome();
    void flightControlGoHomeAndConfirmLanding();
    void flightControlGoHomeAndConfirmLandingNative();
    void flightControlConfirmLanding();
    void flightCancelGoHome();
    void flightControlAvoidParam();
    void flightControlRtkParam();
    void flightControlMoveByPositionOffset(DronePositionOffset);
    void flightControlVelocityAndYawRateCtrl(DroneVelocityTimeOffset);
    void flightPositionTest();
    void flightVelocityTest();
    static T_DjiReturnCode flightControlJoystickCtrlAuthSwitchEventCallback(T_DjiFlightControllerJoystickCtrlAuthorityEventInfo eventData);     //飞行事件将实时返回
    //无人机航点任务相关
    int wayPointTaskUploadStatus;                                                                                                               //航线任务上传状态
    int wayPointTaskExecuteStatus;                                                                                                              //航线任务执行状态
    int missionAutoPauseCount;                                                                                                                  //记录一次任务中自动暂停出现的次数
    bool missionManuPauseFlag;                                                                                                                  //任务手动暂停的标志
    bool missionEntryFlag;                                                                                                                      //是否进入航线的标志(航线状态为执行中时)
    bool missionObstacleAvoidanceFlag;                                                                                                          //任务执行过程中是否触发避障的标志
    QDateTime missionEndTime;                                                                                                                   //任务结束时间,用于过滤结束回调
    WayPointMissionInfo uploadWayPointMission;                                                                                                  //上传的航线任务
    WayPointStateInfo lastMissionState;                                                                                                         //最近任务状态信息
    WayPointStateInfo currentMissionState;                                                                                                      //当前任务状态信息
    void flightRunWaypoints(WayPointMissionInfo wayPointMission);
    static T_DjiReturnCode waypointV2EventCallback(T_DjiWaypointV2MissionEventPush eventData);                                                  //航线任务事件将实时返回
    static T_DjiReturnCode waypointV2StateCallback(T_DjiWaypointV2MissionStatePush stateData);                                                  //航线任务状态将实时返回(100ms)
    void sendWayPointState(WayPointStateInfo wayPointStateInfo);            //发送航线状态
    void wayPointMissionEnd();                                              //航线任务结束处理函数
    void wayPointExceptionPauseProcess();                                   //航线任务异常暂停处理函数
    void wayPointResumeProcess(int& resumeCount);                           //航线任务恢复处理函数
    //电源管理
    void sendPowerOff();
    static T_DjiReturnCode powerOffNotificationCallback(bool *powerOffPreparationFlag);
    //信息发布
    void sendLog(int logLevel, string logDetail);
    void sendCameraFile(DownloadFileInfo downloadFileInfo);
    void sendReturn(int module, int type, int status, string reason);
    void sendWayPoint(int curWaypointIndex);
    void sendDeviceAbnormal(int module, string content);
};

#endif // ANALYSIS_H


/*
 * 无人机相关
typedef enum {
    DJI_AIRCRAFT_TYPE_UNKNOWN = 0, /*!< Aircraft type is unknown.
    DJI_AIRCRAFT_TYPE_M200_V2 = 44, /*!< Aircraft type is Matrice 200 V2.
    DJI_AIRCRAFT_TYPE_M210_V2 = 45, /*!< Aircraft type is Matrice 220 V2.
    DJI_AIRCRAFT_TYPE_M210RTK_V2 = 46, /*!< Aircraft type is Matrice 210 RTK V2.
    DJI_AIRCRAFT_TYPE_M300_RTK = 60, /*!< Aircraft type is Matrice 300 RTK.
} E_DjiAircraftType;

/**
 * @brief Some base information of aircraft system, mainly including some constant parameters information of system.
typedef struct {
    E_DjiAircraftType aircraftType; /*!< Aircraft type.
    E_DjiSdkAdapterType djiAdapterType; /*!< DJI adapter type.
    E_DjiMountPosition mountPosition; /*!< Payload mount position.
} T_DjiAircraftInfoBaseInfo;

typedef enum {
    DJI_MOUNT_POSITION_UNKNOWN = 0,
    DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1 = 1,
    DJI_MOUNT_POSITION_PAYLOAD_PORT_NO2 = 2,
    DJI_MOUNT_POSITION_PAYLOAD_PORT_NO3 = 3,
    DJI_MOUNT_POSITION_EXTENSION_PORT = 4
} E_DjiMountPosition;

 * 相机相关
@brief Liveview camera stream source.
typedef enum {
    DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT = 0,
    DJI_LIVEVIEW_CAMERA_SOURCE_H20_WIDE = 1,
    DJI_LIVEVIEW_CAMERA_SOURCE_H20T_WIDE = 1,
    DJI_LIVEVIEW_CAMERA_SOURCE_H20_ZOOM = 2,
    DJI_LIVEVIEW_CAMERA_SOURCE_H20T_ZOOM = 2,
    DJI_LIVEVIEW_CAMERA_SOURCE_H20T_IR = 3
} E_DjiLiveViewCameraSource;

typedef enum {
    DJI_CAMERA_TYPE_UNKNOWN = 0, /*!< Camera type is unknown
    DJI_CAMERA_TYPE_Z30 = 20, /*!< Camera type is Z30.
    DJI_CAMERA_TYPE_XT2 = 26, /*!< Camera type is XT2.
    DJI_CAMERA_TYPE_PSDK = 31, /*!< Camera type is third party camera based on Payload SDK.
    DJI_CAMERA_TYPE_XTS = 41, /*!< Camera type is XT S.
    DJI_CAMERA_TYPE_H20 = 42, /*!< Camera type is H20.
    DJI_CAMERA_TYPE_H20T = 43, /*!< Camera type is H20T.
    DJI_CAMERA_TYPE_P1 = 50, /*!< Camera type is P1.
    DJI_CAMERA_TYPE_L1, /*!< Camera type is L1.
} E_DjiCameraType;

@brief CameraModule work modes.
typedef enum {
    /*!
     - Capture mode. In this mode, the user can capture pictures.
    DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO = 0,
    /*!
     - Record mode. In this mode, the user can record videos.
    DJI_CAMERA_MANAGER_WORK_MODE_RECORD_VIDEO = 1,
    /*!
     - Playback mode. In this mode, the user can preview photos and videos, and
     can delete files. It is supported by Phantom 3 Professional camera, X3, X5
     and X5R cameras on aircraft and Phantom 4 camera. Playback mode is not
     supported by Z30, X5S, X4S, Phantom 4 Pro, Mavic Pro, Phantom 3 Standard,
     Phantom 3 Advanced, Phantom 3 4K and Osmo series.
    DJI_CAMERA_MANAGER_WORK_MODE_PLAYBACK = 2,
    /*!
     - In this mode, the user can download media to the Mobile Device. Not
     supported by X5 camera nor X5R camera while mounted on aircraft.
    DJI_CAMERA_MANAGER_WORK_MODE_MEDIA_DOWNLOAD = 3,
    /*!
     - In this mode, live stream resolution and frame rate will be 1080i50 (PAL)
     or 720p60 (NTSC). In this mode videos can be recorded. Still photos can
     also be taken only when video is recording. The only way to exit broadcast
     mode is to change modes to RECORD_VIDEO. Only supported by Inspire 2.
    DJI_CAMERA_MANAGER_WORK_MODE_BROADCAST = 4,
    /*!
     * The camera work mode is unknown.
    DJI_CAMERA_MANAGER_WORK_MODE_WORK_MODE_UNKNOWN = 0xFF,
} E_DjiCameraManagerWorkMode;


/*! @brief The ShootPhoto mode itself can have several modes. The default
 * value is SINGLE.
typedef enum {
    /*!
     - Sets the camera to take a single photo.
    DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE = 0x01,
    /*!
     - Sets the camera to take an HDR photo. X5 camera, X5R camera, XT camera,
     Z30 camera, Phantom 4 Pro camera, X4S camera and X5S camera do not support
     HDR mode.
    DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_HDR = 0x02,
    /*!
     - Set the camera to take multiple photos at once. XT camera does not
     support Burst mode.
    DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_BURST = 0x04,
    /*!
     - Automatic Exposure Bracketing (AEB) capture. In this mode you can quickly
     take multiple shots (the default is 3) at different exposures without
     having to manually change any settings between frames. XT camera and Z30
     camera does not support AEB mode.
    DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_AEB = 0x05,
    /*!
     - Sets the camera to take a picture (or multiple pictures) continuously at
     a set time interval. The minimum interval for JPEG format of any quality is
     2s. For all cameras except X4S, X5S and Phantom 4 Pro camera: The minimum
     interval for RAW or RAW+JPEG format is 10s. For the X4S, X5S and Phantom 4
     Pro cameras the minimum interval for RAW or RAW+JPEG dformat is 5s.
    DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_INTERVAL = 0x06,
    /*!
     - Sets the camera to take a burst of RAW photos. Use getRAWPhotoBurstCount
     to check how many photos have been shot. Only supported by X5S.
    DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_RAW_BURST = 0x09,
    /*!
     - 	Sets the camera to take an regional photos. It is supported by H20/H20T.
    DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_REGIONAL_SR = 0x16,
    /*!
     - The shoot photo mode is unknown.
    DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_UNKNOWN = 0xFF,
} E_DjiCameraManagerShootPhotoMode;

/*! @breif CameraModule focus mode. If the physical AF switch on the camera is
 * set to auto.
typedef enum {
    /*!
     - The camera's focus mode is set to manual. In this mode, user sets the
     focus ring value to adjust the focal distance.
    DJI_CAMERA_MANAGER_FOCUS_MODE_MANUAL = 0,
    /*!
     - The camera's focus mode is set to auto. For the Z30 camera, the focus is
     calculated completely automatically. For all other cameras, a focus target
     can be set by the user, which is used to calculate focus automatically.
    DJI_CAMERA_MANAGER_FOCUS_MODE_AUTO = 1,
    /*!
     - The camera's focus mode is set to Continuous AF. It is only supported by
     Mavic Pro with firmware version V01.03.0000 or above, X4S camera, Mavic 2
     Zoom camera and Mavic 2 Pro camera.
    DJI_CAMERA_MANAGER_FOCUS_MODE_AFC = 2,
    /*!
     - The camera's focus mode is unknown.
    DJI_CAMERA_MANAGER_FOCUS_MODE_UNKNOWN = 0xFF,
} E_DjiCameraManagerFocusMode;

* @brief Camera zoom direction.
typedef enum {
    DJI_CAMERA_ZOOM_DIRECTION_OUT = 0, /*!< The lens moves in the far direction, the zoom factor becomes smaller.
    DJI_CAMERA_ZOOM_DIRECTION_IN = 1, /*!< The lens moves in the near direction, the zoom factor becomes larger.
} E_DjiCameraZoomDirection;

 *云台相关
typedef struct Rotation {
    uint8_t rotationMode;
    float pitch;
    float roll;
    float yaw;
    double time;
} Rotation;

typedef enum {
    DJI_GIMBAL_MODE_FREE = 0, //!< Free mode, fix gimbal attitude in the ground coordinate, ignoring movement of aircraft.
    DJI_GIMBAL_MODE_FPV = 1, //!< FPV (First Person View) mode, only control roll and yaw angle of gimbal in the ground coordinate to follow aircraft.
    DJI_GIMBAL_MODE_YAW_FOLLOW = 2, //!< Yaw follow mode, only control yaw angle of gimbal in the ground coordinate to follow aircraft.
} E_DjiGimbalMode;

typedef enum {
    DJI_MOUNT_POSITION_UNKNOWN = 0,
    DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1 = 1,
    DJI_MOUNT_POSITION_PAYLOAD_PORT_NO2 = 2,
    DJI_MOUNT_POSITION_PAYLOAD_PORT_NO3 = 3,
    DJI_MOUNT_POSITION_EXTENSION_PORT = 4
} E_DjiMountPosition;

typedef struct GimbalSingleData
{
    float32_t pitch;
    float32_t roll;
    float32_t yaw;
    uint32_t status;
    uint8_t mode;
} GimbalSingleData;

typedef enum {
    DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE = 0, //!< Relative angle rotation mode, represents rotating gimbal specified angles based on current angles.
    DJI_GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE = 1, //!< Absolute angle rotation mode, represents rotating gimbal to specified angles in the ground coordinate.
    DJI_GIMBAL_ROTATION_MODE_SPEED = 2, //!< Speed rotation mode, specifies rotation speed of gimbal in the ground coordinate.
} E_DjiGimbalRotationMode;

 *航点任务相关
航点动作actions-航点动作为飞机到达航点后执行的动作，比如悬停等待，调整姿态，云台控制、拍照等动作，仅支持OSDK本身定义的actions动作:
typedef enum {
    DJI_WAYPOINT_V2_FINISHED_NO_ACTION,
    DJI_WAYPOINT_V2_FINISHED_GO_HOME,
    DJI_WAYPOINT_V2_FINISHED_AUTO_LANDING,
    DJI_WAYPOINT_V2_FINISHED_GO_TO_FIRST_WAYPOINT,
    DJI_WAYPOINT_V2_FINISHED_CONTINUE_UNTIL_STOP
} E_DJIWaypointV2MissionFinishedAction;

//航点动作参数设置相关--触发器(暂时只使用到点触发和关联触发)
typedef enum {
    DJI_WAYPOINT_V2_ACTION_TRIGGER_ACTION_ASSOCIATED = 2,
    DJI_WAYPOINT_V2_ACTION_TRIGGER_TYPE_TRAJECTORY,
    DJI_WAYPOINT_V2_ACTION_TRIGGER_TYPE_INTERVAL,
    DJI_WAYPOINT_V2_ACTION_TRIGGER_TYPE_SAMPLE_REACH_POINT,
    DJI_WAYPOINT_V2_ACTION_TRIGGER_TYPE_UNKNOWN = 0xFF
} E_DJIWaypointV2ActionTriggerType;

typedef struct {
    uint16_t waypointIndex;
    uint16_t terminateNum;
} T_DJIWaypointV2SampleReachPointTriggerParam;

typedef struct {
    uint8_t actionAssociatedType: 7;
    uint8_t waitTimeUint: 1;
    uint8_t waitingTime;
    uint16_t actionIdAssociated;
} T_DJIWaypointV2AssociateTriggerParam;

//航点动作参数设置相关--执行器(暂时只涉及拍照，录像以及悬停功能)
typedef enum {
    DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_CAMERA = 1,
    DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_GIMBAL = 2,
    DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_AIRCRAFT_CONTROL = 4,
    DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_UNKNOWN = 0xFF
} E_DJIWaypointV2ActionActuatorType;

typedef enum {
    DJI_WAYPOINT_V2_ACTION_ACTUATOR_CAMERA_OPERATION_TYPE_TAKE_PHOTO = 1,
    DJI_WAYPOINT_V2_ACTION_ACTUATOR_CAMERA_OPERATION_TYPE_START_RECORD_VIDEO,
    DJI_WAYPOINT_V2_ACTION_ACTUATOR_CAMERA_OPERATION_TYPE_STOP_RECORD_VIDEO,
    DJI_WAYPOINT_V2_ACTION_ACTUATOR_CAMERA_OPERATION_TYPE_FOCUS,
    DJI_WAYPOINT_V2_ACTION_ACTUATOR_CAMERA_OPERATION_TYPE_FOCUL_LENGTH,
    DJI_WAYPOINT_V2_ACTION_ACTUATOR_CAMERA_OPERATION_TYPE_UNKNOWN = 0xFF,
} E_DJIWaypointV2ActionActuatorCameraOperationType;

typedef enum {
    DJI_WAYPOINT_V2_ACTION_ACTUATOR_GIMBAL_OPERATION_TYPE_ROTATE_GIMBAL = 1,
    DJI_WAYPOINT_V2_ACTION_ACTUATOR_GIMBAL_OPERATION_TYPE_UNKNOWN = 0xFF,
} E_DJIWaypointV2ActionActuatorGimbalOperationType;

typedef enum {
    DJIWaypointV2ActionActuatorAircraftControlOperationTypeRotateYaw = 1,
    DJIWaypointV2ActionActuatorAircraftControlOperationTypeFlyingControl = 2,
    DJIWaypointV2ActionActuatorAircraftControlOperationTypeUnknown = 0xFF,
} E_DJIWaypointV2ActionActuatorAircraftControlOperationType;
*/
