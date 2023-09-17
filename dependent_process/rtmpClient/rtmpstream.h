#ifndef RTMPSTREAM_H
#define RTMPSTREAM_H

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
#include "global.h"

#define MAX_DISPLAY_LEN 64

using namespace std;

/*
管道在无法推送数据的情况下，超过10s后则无法再写入数据，管道会销毁重建
*/

class RtmpStream : public QObject
{
    Q_OBJECT
public:
    explicit RtmpStream(string outPut, string rtmpAddress, QObject *parent = 0);
    ~RtmpStream();
    bool initPipeline();
    bool stop_rtmp();

Q_SIGNALS:
    void streamError();

private:
    static RtmpStream *static_rtmpStream;
    void emitStreamError();                  //to reliaze that static function send siganl
    static gboolean bus_call (GstBus * bus, GstMessage * msg, gpointer data);
    static GstPadProbeReturn osd_sink_pad_buffer_probe (GstPad * pad, GstPadProbeInfo * info, gpointer u_data);
    static void cb_need_data(GstElement *appsrc);
    static void start_feed (GstElement * source, guint size, gpointer user_data);
    static void stop_feed (GstElement * source, gpointer user_data);
    //管道
    GMainLoop *loop;
    GstElement *pipeline = NULL, *streammux = NULL, *decoder = NULL;
    GstPad *sinkpad, *srcpad;
    guint bus_watch_id;
    //控制参数
    string outPut;
    string rtmpAddress;
    bool activateRelease;                    //用于管道主动释放和异常释放的判断
    long startTime;                          //用于无数据推送时的超时判断，以便销毁管道
    long pipelineCheckTid;
    //共享内存
    void *shm = NULL;        //共享存储段连接的实际地址
    int shmid;               //共享内存标识符
    void *sem = NULL;        //共享存储段连接的实际地址
    int semid;        //共享内存标识符
};

#endif // ANALYSISSTREAM_H
