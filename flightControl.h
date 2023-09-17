#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include "global.h"
#include "dji_typedef.h"
#include "dji_flight_controller.h"
#include "dji_fc_subscription.h"
#include "dji_platform.h"
#include "dji_logger.h"
#include "dji_waypoint_v2_type.h"
#include <math.h>

#pragma pack(1)
typedef struct {
    dji_f32_t x;
    dji_f32_t y;
    dji_f32_t z;
} T_DjiTestFlightControlVector3f; // pack(1)
#pragma pack()

typedef struct {
    E_DjiFcSubscriptionDisplayMode displayMode;
    char *displayModeStr;
} T_DjiTestFlightControlDisplayModeStr;

typedef struct {
    uint8_t eventID;
    char *eventStr;
} T_DjiTestWaypointV2EventStr;

typedef struct {
    uint8_t missionState;
    char *stateStr;
} T_DjiTestWaypointV2StateStr;

#ifdef __cplusplus
extern "C" {
#endif

static const double s_earthCenter = 6378137.0;
static const double s_degToRad = 0.01745329252;
static const T_DjiTestFlightControlDisplayModeStr s_flightControlDisplayModeStr[] = {
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE, .displayModeStr = "attitude mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS, .displayModeStr = "p_gps mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ASSISTED_TAKEOFF, .displayModeStr = "assisted takeoff mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_TAKEOFF, .displayModeStr = "auto takeoff mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING, .displayModeStr = "auto landing mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME, .displayModeStr = "go home mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING, .displayModeStr = "force landing mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ENGINE_START, .displayModeStr = "engine start mode"},
    {.displayMode = (E_DjiFcSubscriptionDisplayMode)0xFF, .displayModeStr = "unknown mode"}
};

static uint32_t s_missionID = 12345;

//reference note of "T_DjiWaypointV2MissionEventPush"
static const T_DjiTestWaypointV2EventStr s_waypointV2EventStr[] = {
    {.eventID = 0x01, .eventStr = "Interrupt Event"},
    {.eventID = 0x02, .eventStr = "Resume Event"},
    {.eventID = 0x03, .eventStr = "Stop Event"},
    {.eventID = 0x10, .eventStr = "Arrival Event"},
    {.eventID = 0x11, .eventStr = "Finished Event"},
    {.eventID = 0x12, .eventStr = "Avoid Obstacle Event"},
    {.eventID = 0x30, .eventStr = "Action Switch Event"},
    {.eventID = 0xFF, .eventStr = "Unknown"}
};

//reference note of "T_DjiWaypointV2MissionStatePush"
static const T_DjiTestWaypointV2StateStr s_waypointV2StateStr[] = {
    {.missionState = 0x00, .stateStr = "Ground station not start"},
    {.missionState = 0x01, .stateStr = "Mission prepared"},
    {.missionState = 0x02, .stateStr = "Enter mission"},
    {.missionState = 0x03, .stateStr = "Execute mission"},
    {.missionState = 0x04, .stateStr = "Pause Mission"},
    {.missionState = 0x05, .stateStr = "Enter mission after ending pause"},
    {.missionState = 0x06, .stateStr = "Exit mission"},
    {.missionState = 0xFF, .stateStr = "Unknown"}
};

static uint8_t DjiTest_FlightControlGetDisplayModeIndex(E_DjiFcSubscriptionDisplayMode displayMode);
static bool DjiTest_FlightControlMotorStartedCheck(void);
static bool DjiTest_FlightControlTakeOffInAirCheck(void);
static bool takeoffFinishedCheck(void);
static bool DjiTest_FlightControlLandFinishedCheck(void);
static bool DjiTest_FlightControlCheckActionStarted(E_DjiFcSubscriptionDisplayMode mode);

static T_DjiFcSubscriptionFlightStatus DjiTest_FlightControlGetValueOfFlightStatus(void);
static T_DjiFcSubscriptionDisplaymode DjiTest_FlightControlGetValueOfDisplayMode(void);
static T_DjiFcSubscriptionAvoidData DjiTest_FlightControlGetValueOfAvoidData(void);
static T_DjiFcSubscriptionQuaternion DjiTest_FlightControlGetValueOfQuaternion(void);
static T_DjiFcSubscriptionPositionFused DjiTest_FlightControlGetValueOfPositionFused(void);
static dji_f32_t DjiTest_FlightControlGetValueOfRelativeHeight(void);

static T_DjiTestFlightControlVector3f DjiTest_FlightControlQuaternionToEulerAngle(T_DjiFcSubscriptionQuaternion quat);
static T_DjiTestFlightControlVector3f DjiTest_FlightControlLocalOffsetFromGpsAndFusedHeightOffset(T_DjiFcSubscriptionPositionFused target,
                                                            T_DjiFcSubscriptionPositionFused origin,
                                                            dji_f32_t targetHeight,
                                                            dji_f32_t originHeight);
static T_DjiTestFlightControlVector3f DjiTest_FlightControlVector3FSub(T_DjiTestFlightControlVector3f vectorA, T_DjiTestFlightControlVector3f vectorB);
static void DjiTest_FlightControlHorizCommandLimit(dji_f32_t speedFactor, dji_f32_t *commandX, dji_f32_t *commandY);
static dji_f32_t DjiTest_FlightControlVectorNorm(T_DjiTestFlightControlVector3f v);
static int DjiTest_FlightControlSignOfData(dji_f32_t data);

static void DjiTest_WaypointV2SetDefaultSetting(T_DjiWaypointV2 *waypointV2);
static uint8_t DJiTest_WaypointV2GetMissionEventIndex(uint8_t eventID);
static uint8_t DjiTest_WaypointV2GetMissionStateIndex(uint8_t state);
//无人机状态检查
static uint8_t DjiTest_FlightControlGetDisplayModeIndex(E_DjiFcSubscriptionDisplayMode displayMode)
{
    uint8_t i;

    for (i = 0; i < sizeof(s_flightControlDisplayModeStr) / sizeof(T_DjiTestFlightControlDisplayModeStr); i++) {
        if (s_flightControlDisplayModeStr[i].displayMode == displayMode) {
            return i;
        }
    }

    return i;
}

static bool DjiTest_FlightControlTakeOffInAirCheck(void)
{
    int stillOnGround = 0;
    int timeoutCycles = 110;
    T_DjiOsalHandler *s_osalHandler = DjiPlatform_GetOsalHandler();

    while (DjiTest_FlightControlGetValueOfFlightStatus() != DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR &&
           (DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ASSISTED_TAKEOFF ||
            DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_TAKEOFF) &&
           stillOnGround < timeoutCycles) {
        stillOnGround++;
        s_osalHandler->TaskSleepMs(100);
    }

    return stillOnGround != timeoutCycles ? true : false;
}

static bool takeoffFinishedCheck(void)
{
    T_DjiOsalHandler *s_osalHandler = DjiPlatform_GetOsalHandler();

    while (DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_TAKEOFF ||
           DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ASSISTED_TAKEOFF) {
        s_osalHandler->TaskSleepMs(1000);
    }

    return (DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS ||
            DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE) ? true : false;
}

static bool DjiTest_FlightControlLandFinishedCheck(void)
{
    T_DjiOsalHandler *s_osalHandler = DjiPlatform_GetOsalHandler();

    while (DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING ||
           DjiTest_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR) {
        s_osalHandler->TaskSleepMs(1000);
    }

    return (DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS ||
            DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE) ? true : false;
}

static bool DjiTest_FlightControlCheckActionStarted(E_DjiFcSubscriptionDisplayMode mode){
    int actionNotStarted = 0;
    int timeoutCycles = 20;
    T_DjiOsalHandler *s_osalHandler = DjiPlatform_GetOsalHandler();


    while (DjiTest_FlightControlGetValueOfDisplayMode() != mode && actionNotStarted < timeoutCycles) {
        actionNotStarted++;
        s_osalHandler->TaskSleepMs(100);
    }

    if (actionNotStarted == timeoutCycles) {
        USER_LOG_ERROR("%s start failed, now flight is in %s.",
                       s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(mode)].displayModeStr,
                       s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex((E_DjiFcSubscriptionDisplayMode)(DjiTest_FlightControlGetValueOfDisplayMode()))].displayModeStr);
        return false;
    } else {
        USER_LOG_INFO("Now flight is in %s.",
                      s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(mode)].displayModeStr);
        return true;
    }
}

static bool DjiTest_FlightControlMotorStartedCheck(void)
{
    int motorsNotStarted = 0;
    int timeoutCycles = 20;
    T_DjiOsalHandler *s_osalHandler = DjiPlatform_GetOsalHandler();

    while (DjiTest_FlightControlGetValueOfFlightStatus() != DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_ON_GROUND &&
           DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ENGINE_START &&
           motorsNotStarted < timeoutCycles) {
        motorsNotStarted++;
        s_osalHandler->TaskSleepMs(100);
    }
    return motorsNotStarted != timeoutCycles ? true : false;
}

//无人机数据获取
static T_DjiFcSubscriptionFlightStatus DjiTest_FlightControlGetValueOfFlightStatus(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionFlightStatus flightStatus;
    T_DjiDataTimestamp flightStatusTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                                      (uint8_t *) &flightStatus,
                                                      sizeof(T_DjiFcSubscriptionFlightStatus),
                                                      &flightStatusTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic flight status error, error code: 0x%08X", djiStat);
        flightStatus = 0;
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", flightStatusTimestamp.millisecond,
                       flightStatusTimestamp.microsecond);
        USER_LOG_DEBUG("Flight status: %d.", flightStatus);
    }
    return flightStatus;
}

static T_DjiFcSubscriptionDisplaymode DjiTest_FlightControlGetValueOfDisplayMode(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionDisplaymode displayMode;
    T_DjiDataTimestamp displayModeTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
                                                      (uint8_t *) &displayMode,
                                                      sizeof(T_DjiFcSubscriptionDisplaymode),
                                                      &displayModeTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic display mode error, error code: 0x%08X", djiStat);
        displayMode = 0;
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", displayModeTimestamp.millisecond,
                       displayModeTimestamp.microsecond);
        USER_LOG_DEBUG("Display mode : %d.", displayMode);
    }

    return displayMode;
}

static T_DjiFcSubscriptionAvoidData DjiTest_FlightControlGetValueOfAvoidData(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionAvoidData avoidData = {0};
    T_DjiDataTimestamp avoidDataTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_AVOID_DATA,
                                                      (uint8_t *) &avoidData,
                                                      sizeof(T_DjiFcSubscriptionAvoidData),
                                                      &avoidDataTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic avoid data error, error code: 0x%08X", djiStat);
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", avoidDataTimestamp.millisecond,
                       avoidDataTimestamp.microsecond);
        USER_LOG_DEBUG("Avoid downwards distance is %f m", avoidData.down);
    }

    return avoidData;
}

static T_DjiFcSubscriptionQuaternion DjiTest_FlightControlGetValueOfQuaternion(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionQuaternion quaternion = {0};
    T_DjiDataTimestamp quaternionTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                                      (uint8_t *) &quaternion,
                                                      sizeof(T_DjiFcSubscriptionQuaternion),
                                                      &quaternionTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic quaternion error, error code: 0x%08X", djiStat);
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", quaternionTimestamp.millisecond,
                       quaternionTimestamp.microsecond);
        USER_LOG_DEBUG("Quaternion: %f %f %f %f.", quaternion.q0, quaternion.q1, quaternion.q2, quaternion.q3);
    }

    return quaternion;
}

static T_DjiFcSubscriptionPositionFused DjiTest_FlightControlGetValueOfPositionFused(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionPositionFused positionFused = {0};
    T_DjiDataTimestamp positionFusedTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
                                                      (uint8_t *) &positionFused,
                                                      sizeof(T_DjiFcSubscriptionPositionFused),
                                                      &positionFusedTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic position fused error, error code: 0x%08X", djiStat);
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", positionFusedTimestamp.millisecond,
                       positionFusedTimestamp.microsecond);
        USER_LOG_DEBUG("PositionFused: %f, %f,%f,%d.", positionFused.latitude, positionFused.longitude,
                       positionFused.altitude, positionFused.visibleSatelliteNumber);
    }

    return positionFused;
}

static dji_f32_t DjiTest_FlightControlGetValueOfRelativeHeight(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionAltitudeFused altitudeFused = 0;
    T_DjiFcSubscriptionAltitudeOfHomePoint homePointAltitude = 0;
    dji_f32_t relativeHeight = 0;
    T_DjiDataTimestamp relativeHeightTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
                                                      (uint8_t *) &homePointAltitude,
                                                      sizeof(T_DjiFcSubscriptionAltitudeOfHomePoint),
                                                      &relativeHeightTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic altitude of home point error, error code: 0x%08X", djiStat);
        return -1;
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", relativeHeightTimestamp.millisecond,
                       relativeHeightTimestamp.microsecond);
    }

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
                                                      (uint8_t *) &altitudeFused,
                                                      sizeof(T_DjiFcSubscriptionAltitudeFused),
                                                      &relativeHeightTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic altitude fused error, error code: 0x%08X", djiStat);
        return -1;
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", relativeHeightTimestamp.millisecond,
                       relativeHeightTimestamp.microsecond);
    }

    relativeHeight = altitudeFused - homePointAltitude;

    return relativeHeight;
}

//无人机控制计算
static T_DjiTestFlightControlVector3f DjiTest_FlightControlQuaternionToEulerAngle(const T_DjiFcSubscriptionQuaternion quat)
{
    T_DjiTestFlightControlVector3f eulerAngle;
    double q2sqr = quat.q2 * quat.q2;
    double t0 = -2.0 * (q2sqr + quat.q3 * quat.q3) + 1.0;
    double t1 = (dji_f64_t) 2.0 * (quat.q1 * quat.q2 + quat.q0 * quat.q3);
    double t2 = -2.0 * (quat.q1 * quat.q3 - quat.q0 * quat.q2);
    double t3 = (dji_f64_t) 2.0 * (quat.q2 * quat.q3 + quat.q0 * quat.q1);
    double t4 = -2.0 * (quat.q1 * quat.q1 + q2sqr) + 1.0;
    t2 = (t2 > 1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;
    eulerAngle.x = asin(t2);
    eulerAngle.y = atan2(t3, t4);
    eulerAngle.z = atan2(t1, t0);
    return eulerAngle;
}

static T_DjiTestFlightControlVector3f DjiTest_FlightControlLocalOffsetFromGpsAndFusedHeightOffset(const T_DjiFcSubscriptionPositionFused target,
                                                            const T_DjiFcSubscriptionPositionFused origin,
                                                            const dji_f32_t targetHeight,
                                                            const dji_f32_t originHeight)
{
    T_DjiTestFlightControlVector3f deltaNed;
    double deltaLon = target.longitude - origin.longitude;
    double deltaLat = target.latitude - origin.latitude;
    deltaNed.x = deltaLat * s_earthCenter;
    deltaNed.y = deltaLon * s_earthCenter * cos(target.latitude);
    deltaNed.z = targetHeight - originHeight;

    return deltaNed;
}

static T_DjiTestFlightControlVector3f DjiTest_FlightControlVector3FSub(const T_DjiTestFlightControlVector3f vectorA,
                                 const T_DjiTestFlightControlVector3f vectorB)
{
    T_DjiTestFlightControlVector3f result;
    result.x = vectorA.x - vectorB.x;
    result.y = vectorA.y - vectorB.y;
    result.z = vectorA.z - vectorB.z;
    return result;
}

static void DjiTest_FlightControlHorizCommandLimit(dji_f32_t speedFactor, dji_f32_t *commandX, dji_f32_t *commandY)
{
    if (fabs(*commandX) > speedFactor)
        *commandX = speedFactor * DjiTest_FlightControlSignOfData(*commandX);
    if (fabs(*commandY) > speedFactor)
        *commandY = speedFactor * DjiTest_FlightControlSignOfData(*commandY);
}

static dji_f32_t DjiTest_FlightControlVectorNorm(T_DjiTestFlightControlVector3f v)
{
    return sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
}

static int DjiTest_FlightControlSignOfData(dji_f32_t data)
{
    return data < 0 ? -1 : 1;
}

//航点任务相关
static void DjiTest_WaypointV2SetDefaultSetting(T_DjiWaypointV2 *waypointV2)
{
    waypointV2->waypointType = DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_IN_STRAIGHT_AND_STOP;
    waypointV2->headingMode = DJI_WAYPOINT_V2_HEADING_MODE_AUTO;
    waypointV2->config.useLocalMaxVel = 0;
    waypointV2->config.useLocalCruiseVel = 0;
    waypointV2->dampingDistance = 40;
    waypointV2->heading = 0;
    waypointV2->turnMode = DJI_WAYPOINT_V2_TURN_MODE_CLOCK_WISE;

    waypointV2->pointOfInterest.positionX = 0;
    waypointV2->pointOfInterest.positionY = 0;
    waypointV2->pointOfInterest.positionZ = 0;
    waypointV2->maxFlightSpeed = 15;
    waypointV2->autoFlightSpeed = 5;
}

static uint8_t DJiTest_WaypointV2GetMissionEventIndex(uint8_t eventID)
{
    uint8_t i;

    for (i = 0; i < sizeof(s_waypointV2EventStr) / sizeof(T_DjiTestWaypointV2EventStr); i++) {
        if (s_waypointV2EventStr[i].eventID == eventID) {
            return i;
        }
    }

    return i;
}

static uint8_t DjiTest_WaypointV2GetMissionStateIndex(uint8_t state)
{
    uint8_t i;

    for (i = 0; i < sizeof(s_waypointV2StateStr) / sizeof(T_DjiTestWaypointV2StateStr); i++) {
        if (s_waypointV2StateStr[i].missionState == state) {
            return i;
        }
    }

    return i;
}

#ifdef __cplusplus
}
#endif

#endif
