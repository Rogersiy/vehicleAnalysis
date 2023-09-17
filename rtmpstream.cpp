#include "rtmpstream.h"
RtmpStream* RtmpStream::static_rtmpStream = NULL;

RtmpStream::RtmpStream(QObject *parent):QObject(parent), pipeline(NULL), bus_watch_id(0), loop(NULL)
{
    static_rtmpStream = this;

    //初始化管道异常检查线程
    this->startTime = 0;
    std::thread pipelineCheckThread = std::thread([this]{
        while(true){
            if(time(NULL) - this->startTime >= 10 && this->startTime != 0){
                qCritical() << "管道无数据推送超时,异常退出";
                g_main_loop_quit(this->loop);
                this->startTime = 0;
            }
            g_usleep(500000);
        }
        this->pipelineCheckTid = -1;
    });
    this->pipelineCheckTid = pipelineCheckThread.native_handle();
    pipelineCheckThread.detach();
}

RtmpStream::~RtmpStream()
{
    if(this->pipelineCheckTid > 0){
        pthread_cancel(this->pipelineCheckTid);
        this->pipelineCheckTid = -1;
    }

    static_rtmpStream = NULL;
}

bool RtmpStream::initPipeline(){
    qDebug() << "rtmp推流管道初始化中...";

    this->activateRelease = false;
    this->startTime = 0;

    guint udp_port  = g_udp_port;
    string rtmpAddress = g_rtmp_address.toStdString();

    GstElement *udpsrc = NULL, *rtpdepay, *parse = NULL, *flvmux, *sink = NULL;
    GstBus *bus = NULL;
    GstElement  *queue1 = NULL, *queue2= NULL, *queue3= NULL, *queue4= NULL;
    /* Standard GStreamer initialization */
    gst_init(NULL, NULL);
    loop = g_main_loop_new(NULL, FALSE);

    /* Create gstreamer elements */
    /* Create Pipeline element that will form a connection of other elements */
    pipeline = gst_pipeline_new ("rtmp-pipeline");
    /* udpsrc element for reading from the file */
    udpsrc = gst_element_factory_make ("udpsrc", "source");

    rtpdepay = gst_element_factory_make ("rtph264depay", "rtpdepay");

    parse = gst_element_factory_make ("h264parse", "h264-parser2");

    flvmux = gst_element_factory_make ("flvmux", "flvmux1");

    sink = gst_element_factory_make("rtmpsink", "rtmpsink1");

    queue1 = gst_element_factory_make ("queue", "queue1");
    queue2 = gst_element_factory_make ("queue", "queue2");
    queue3 = gst_element_factory_make ("queue", "queue3");
    queue4 = gst_element_factory_make ("queue", "queue4");

    if (!pipeline || !udpsrc || !rtpdepay || !parse || !flvmux || !sink) {
        g_printerr ("one element could not be created. Exiting.\n");
        return -1;
    }

    g_object_set (udpsrc, "name", "pay0", "port", udp_port, "buffer-size", 524288, "caps", gst_caps_new_simple ("application/x-rtp",
                                                                                                                "media", G_TYPE_STRING, "video",
                                                                                                                "clock-rate", G_TYPE_INT, 90000,
                                                                                                                "encoding-name", G_TYPE_STRING, "H264",
                                                                                                                "payload", G_TYPE_INT, 96, NULL), NULL);

    g_object_set(G_OBJECT(flvmux), "streamable", true, NULL);
    //g_object_set(G_OBJECT(sink), "async", true, "sync", false, "qos", false, NULL);
    //g_object_set (G_OBJECT (sink), "max-lateness", 1000000000, NULL);
    //g_object_set(G_OBJECT(sink), "enable-last-sample", false, NULL);
    g_object_set (G_OBJECT (sink), "location", (char *)rtmpAddress.c_str(), NULL);


    gst_bin_add_many (GST_BIN (pipeline), udpsrc, queue1, rtpdepay, queue2, parse, queue3, flvmux, queue4, sink, NULL);


    if (!gst_element_link_many (udpsrc, queue1, rtpdepay, queue2, parse, queue3, flvmux, queue4, sink, NULL)) {
        g_printerr ("Elements could not be linked: 3. Exiting.\n");
        return -1;
    }

    /* Lets add probe to get informed of the meta data generated, we add probe to
     * the sink pad of the osd element, since by that time, the buffer would have
     * had got all the metadata. */
    GstPad *osd_sink_pad = gst_element_get_static_pad (sink, "sink");
    if (!osd_sink_pad)
        g_print ("Unable to get sink pad\n");
    else
        gst_pad_add_probe (osd_sink_pad, GST_PAD_PROBE_TYPE_BUFFER,
                           osd_sink_pad_buffer_probe, NULL, NULL);
    gst_object_unref (osd_sink_pad);

    /* we add a message handler */
    bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));
    bus_watch_id = gst_bus_add_watch (bus, bus_call, loop);
    gst_object_unref (bus);
    qDebug() << "rtmp推流管道初始化成功!";


    gst_element_set_state (pipeline, GST_STATE_PLAYING);
    qDebug() << "rtmp推流管道运行中...";
    g_main_loop_run(loop);
    qDebug() << "rtmp推流管道释放中...";

//    GstState state, pending_state;
//    gst_element_get_state(pipeline, &state, &pending_state, GST_CLOCK_TIME_NONE);
//    qWarning() << "pipeline state is" << gst_element_state_get_name (state);
//    qWarning() << "pipeline state is locked:" << gst_element_is_locked_state(pipeline);

//    gst_element_get_state(udpsrc, &state, &pending_state, GST_CLOCK_TIME_NONE);
//    qWarning() << "udpsrc state is" << gst_element_state_get_name (state);
//    qWarning() << "udpsrc state is locked:" << gst_element_is_locked_state(udpsrc);

//    gst_element_get_state(rtpdepay, &state, &pending_state, GST_CLOCK_TIME_NONE);
//    qWarning() << "rtpdepay state is" << gst_element_state_get_name (state);
//    qWarning() << "rtpdepay state is locked:" << gst_element_is_locked_state(rtpdepay);

//    gst_element_get_state(parse, &state, &pending_state, GST_CLOCK_TIME_NONE);
//    qWarning() << "parse state is" << gst_element_state_get_name (state);
//    qWarning() << "parse state is locked:" << gst_element_is_locked_state(parse);

//    gst_element_get_state(flvmux, &state, &pending_state, GST_CLOCK_TIME_NONE);
//    qWarning() << "flvmux state is" << gst_element_state_get_name (state);
//    qWarning() << "flvmux state is locked:" << gst_element_is_locked_state(flvmux);

//    gst_element_get_state(sink, &state, &pending_state, GST_CLOCK_TIME_NONE);
//    qWarning() << "sink state is" << gst_element_state_get_name (state);
//    qWarning() << "sink state is locked:" << gst_element_is_locked_state(sink);

//    gst_element_send_event(udpsrc, gst_event_new_eos());
//    qDebug() << "gst_element_send_event udpsrc...";

//    gst_element_send_event(rtpdepay, gst_event_new_eos());
//    qDebug() << "gst_element_send_event rtpdepay...";

//    gst_element_send_event(parse, gst_event_new_eos());
//    qDebug() << "gst_element_send_event parse...";

//    gst_element_send_event(flvmux, gst_event_new_eos());
//    qDebug() << "gst_element_send_event flvmux...";

//    gst_element_send_event(sink, gst_event_new_eos());
//    qDebug() << "gst_element_send_event sink...";

//    gst_element_send_event(pipeline, gst_event_new_eos());
//    qDebug() << "gst_element_send_event pipeline...";

//    GstControlSource* csource = gst_interpolation_control_source_new ();
//    g_object_set (csource, "mode", GST_INTERPOLATION_MODE_LINEAR, NULL);
//    g_object_set (GST_OBJECT(udpsrc), "num-buffers", 1024, NULL);
//    gst_object_add_control_binding (GST_OBJECT(udpsrc), gst_direct_control_binding_new (GST_OBJECT(udpsrc), "num-buffers", csource));
//    qDebug() << "num-buffers...";


//    gst_element_unlink(queue4, sink);
//    gst_bin_remove(GST_BIN(pipeline), sink);
//    sink = gst_element_factory_make("fakesink", "fakesink1");
//    gst_bin_add(GST_BIN(pipeline), sink);
//    gst_element_link(queue4, sink);
//    gst_element_set_state(GST_BIN(pipeline), GST_STATE_PLAYING);
//    qDebug() << "GST_STATE_PLAYING...";

//    g_usleep(5000000);


//    gst_element_set_state(pipeline, GST_STATE_PAUSED);
//    qDebug() << "GST_STATE_PAUSED...";
//    gst_element_set_state(pipeline, GST_STATE_READY);
//    qDebug() << "GST_STATE_READY...";
    gst_element_set_state(pipeline, GST_STATE_NULL);
    qDebug() << "GST_STATE_NULL...";
    g_usleep(5000000);                     //must wait for some time
    qDebug() << "g_usleep...";
    gst_object_unref(GST_OBJECT(pipeline));
    //qDebug() << "gst_object_unref...";
    g_source_remove(bus_watch_id);
    //qDebug() << "g_source_remove...";
    g_main_loop_unref(loop);
    qDebug() << "rtmp推流管道已退出";

    if(static_rtmpStream != NULL && !this->activateRelease){
        static_rtmpStream->emitStreamError();
    }

    return true;
}

void RtmpStream::emitStreamError(){
    Q_EMIT streamError();
}

gboolean RtmpStream::bus_call (GstBus * bus, GstMessage * msg, gpointer data){
    GMainLoop *loop = (GMainLoop *) data;
    switch (GST_MESSAGE_TYPE (msg)) {
        case GST_MESSAGE_EOS:
        {
          qWarning () << "rtmp推流结束!!!!!!";
          g_main_loop_quit (loop);
          break;
        }
        case GST_MESSAGE_WARNING:
        {
          gchar *debug;
          GError *error;
          gst_message_parse_warning (msg, &error, &debug);
          qWarning() << "WARNING from element %s:" << GST_OBJECT_NAME (msg->src) << ":" << error->message;
          g_print ("WARNING from element %s: %s\n",
              GST_OBJECT_NAME (msg->src), error->message);
          g_free (debug);
          g_print ("Warning: %s\n", error->message);
          g_error_free (error);
          break;
        }
        case GST_MESSAGE_ERROR:
        {
          gchar *debug;
          GError *error;
          gst_message_parse_error (msg, &error, &debug);
          qCritical() << "rtmp推流管道发生错误,ERROR from element:" << GST_OBJECT_NAME (msg->src) << "  " << error->message;
          g_print ("ERROR from element %s: %s\n", GST_OBJECT_NAME (msg->src), error->message);
          if (debug)
            qDebug() << "Error details:" << debug;
            g_print ("Error details: %s\n", debug);
          g_free (debug);
          g_error_free (error);
          break;
        }
        case GST_MESSAGE_STATE_CHANGED:
        {
          /* We are only interested in state-changed messages from the pipeline */
          //if (GST_MESSAGE_SRC (msg) == GST_OBJECT (data.pipeline)) {
            GstState old_state, new_state, pending_state;
            gst_message_parse_state_changed (msg, &old_state, &new_state, &pending_state);
            //qWarning() << GST_MESSAGE_SRC (msg) << " state changed from" << gst_element_state_get_name (old_state) << " to " << gst_element_state_get_name (new_state);
            g_print ("Pipeline state changed from %s to %s:\n",
                gst_element_state_get_name (old_state), gst_element_state_get_name (new_state));
          //}
          break;
        }
        #ifndef PLATFORM_TEGRA
        case GST_MESSAGE_ELEMENT:
        {
          if (gst_nvmessage_is_stream_eos (msg)) {
            guint stream_id;
            if (gst_nvmessage_parse_stream_eos (msg, &stream_id)) {
                qWarning() << "Got EOS from stream:" << stream_id;
                g_print ("Got EOS from stream %d\n", stream_id);
            }
          }
          break;
        }
        #endif
        default:
        {
          //qDebug() << "Unexpected message received";
          break;
        }
    }
    return TRUE;
}

GstPadProbeReturn RtmpStream::osd_sink_pad_buffer_probe (GstPad * pad, GstPadProbeInfo * info, gpointer u_data)
{
    static_rtmpStream->startTime = time(NULL);
//    string time = (QDateTime::currentDateTime().toString("yyyy年MM月dd日 hh:mm:ss")).toStdString();
//    cout << "rtmpThread:" << time << endl;
    return GST_PAD_PROBE_OK;
}

bool RtmpStream::stop_rtmp(){
    this->activateRelease = true;
    g_main_loop_quit (loop);
}
