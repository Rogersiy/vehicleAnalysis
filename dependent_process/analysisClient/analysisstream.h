#ifndef RTMPSTREAM_H
#define RTMPSTREAM_H

#include "global.h"
#include <QDebug>
#include <QObject>
#include <QDateTime>
#include <gst/gst.h>
#include <glib.h>
#include <cuda_runtime_api.h>
#include "gstnvdsmeta.h"
#include "gst-nvmessage.h"
#include "nvbufsurface.h"
#include "nvbufsurftransform.h"
#include "gstnvdsinfer.h"
#include "gst/rtsp-server/rtsp-server.h"
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/gstbuffer.h>


using namespace std;

#define MAX_DISPLAY_LEN 64
#define PGIE_CLASS_ID_VEHICLE 0
#define PGIE_CLASS_ID_PERSON 2

#define SKIP_FRAME 0

#define GST_CAPS_FEATURES_NVMM "memory:NVMM"
#define PGIE_CONFIG_FILE "/opt/nvidia/deepstream/deepstream-6.0/sources/apps/sample_apps/deepstream-test1/dstest1_pgie_config.txt"

/* The muxer output resolution must be set if the input streams will be of
 * different resolution. The muxer will scale all the input frames to this
 * resolution. */
#define MUXER_OUTPUT_WIDTH 1920
#define MUXER_OUTPUT_HEIGHT 1080

/* Muxer batch formation timeout, for e.g. 40 millisec. Should ideally be set
 * based on the fastest source's framerate. */
#define MUXER_BATCH_TIMEOUT_USEC 30000

#define CHECK_ERROR(error) \
    if (error) { \
        g_printerr ("Error while parsing config file: %s\n", error->message); \
        goto done; \
    }

#define CONFIG_GROUP_TRACKER "tracker"
#define CONFIG_GROUP_TRACKER_WIDTH "tracker-width"
#define CONFIG_GROUP_TRACKER_HEIGHT "tracker-height"
#define CONFIG_GROUP_TRACKER_LL_CONFIG_FILE "ll-config-file"
#define CONFIG_GROUP_TRACKER_LL_LIB_FILE "ll-lib-file"
#define CONFIG_GROUP_TRACKER_ENABLE_BATCH_PROCESS "enable-batch-process"
#define CONFIG_GPU_ID "gpu-id"

#define USER_ARRAY_SIZE 16
/** set the user metadata type */
#define NVDS_USER_FRAME_META_EXAMPLE (nvds_get_user_meta_type("NVIDIA.NVINFER.USER_META"))
#define NVDS_SOURCE_BIN_GST_META (nvds_get_user_meta_type("GST_USER_META"))

/*
管道在无法推送数据的情况下，超过10s后则无法再写入数据，管道会销毁重建
*/

class AnalysisStream : public QObject
{
    Q_OBJECT
public:
    explicit AnalysisStream(string analysisType, string outPut, string rtmpAddress, QObject *parent = 0);
    ~AnalysisStream();
    bool initPipeline();
    void startAnalysis();
    void pauseAnalysis();
    void resumeAnalysis();
    void stopAnalysis();

Q_SIGNALS:
    void sendResult(DetectionInfo);

private:
    static AnalysisStream *static_analysisStream;
    static gboolean bus_call (GstBus * bus, GstMessage * msg, gpointer data);
    static gboolean set_tracker_properties (GstElement *nvtracker);
    static gchar *get_absolute_file_path (gchar *cfg_file_path, gchar *file_path);
    static GstPadProbeReturn osd_sink_pad_buffer_probe (GstPad * pad, GstPadProbeInfo * info, gpointer u_data);
    static void cb_need_data(GstElement *appsrc);
    static void start_feed (GstElement * source, guint size, gpointer user_data);
    static void stop_feed (GstElement * source, gpointer user_data);
    void emitSignal(DetectionInfo detection);                        //to reliaze that static function send siganl
    //rtsp server
    static GstRTSPServer *server;
    static gboolean start_rtsp_streaming (guint rtsp_port_num, guint updsink_port_num, guint64 udp_buffer_size);
    static void destroy_sink_bin();
    static GstRTSPFilterResult client_filter (GstRTSPServer * server, GstRTSPClient * client, gpointer user_data);
    static gboolean timeout (GstRTSPServer * server);
    //管道
    GMainLoop *loop;
    GstElement *pipeline = NULL;
    guint bus_watch_id;
    //控制参数
    string analysisType;                    //使用的算法类型
    string outPut;                          //管道输出类型(可扩充)  display/rtmp/rtsp/none
    string rtmpAddress;                     //rtmp推流地址
    //异常检查线程
    bool activateRelease;                   //用于管道主动释放和异常释放的判断
    long startTime;                         //用于无数据推送时的超时判断，以便销毁管道
    long pipelineCheckTid;
    //共享内存
    void *shm = NULL;        //共享存储段连接的实际地址
    int shmid;               //共享内存标识符
    void *sem = NULL;        //共享存储段连接的实际地址
    int semid;               //共享内存标识符
};

#endif // ANALYSISSTREAM_H
