#ifndef RTMPSTREAM_H
#define RTMPSTREAM_H

#include <QThread>
#include <QObject>
#include "global.h"
#include <gst/gst.h>
#include <gst/controller/gstinterpolationcontrolsource.h>
#include <gst/controller/gstdirectcontrolbinding.h>
#include <glib.h>
#include <cuda_runtime_api.h>
#include "gstnvdsmeta.h"
#include "gst-nvmessage.h"
#include "nvbufsurface.h"
#include "nvbufsurftransform.h"

/*
管道在无法推送数据的情况下，超过10s后则无法再写入数据，管道会销毁重建
*/

class RtmpStream : public QObject
{
    Q_OBJECT
public:
    explicit RtmpStream(QObject *parent = 0);
    ~RtmpStream();
    bool initPipeline();
    bool stop_rtmp();

Q_SIGNALS:
    void streamError();

private:
    void emitStreamError();                  //to reliaze that static function send siganl
    static gboolean bus_call (GstBus * bus, GstMessage * msg, gpointer data);
    static RtmpStream *static_rtmpStream;
    static GstPadProbeReturn osd_sink_pad_buffer_probe (GstPad * pad, GstPadProbeInfo * info, gpointer u_data);
    GMainLoop *loop;
    GstElement *pipeline;
    guint bus_watch_id;
    bool activateRelease;                    //用于管道主动释放和异常释放的判断
    long startTime;                          //用于无数据推送时的超时判断，以便销毁管道
    long pipelineCheckTid;
};

#endif // ANALYSISSTREAM_H
