#ifndef ANALYSISSTREAM_H
#define ANALYSISSTREAM_H

#include <QThread>
#include <QObject>
#include "global.h"
#include <gst/gst.h>
#include <glib.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

#define MAX_DISPLAY_LEN 64
#define PGIE_CLASS_ID_VEHICLE 0
#define PGIE_CLASS_ID_PERSON 2

#define SKIP_FRAME 0

#define GST_CAPS_FEATURES_NVMM "memory:NVMM"
#define PGIE_CONFIG_FILE "/opt/nvidia/deepstream/deepstream-6.0/sources/apps/sample_apps/deepstream-test1/dstest1_pgie_config.txt"
#define TRACKER_CONFIG_FILE "/home/bx/projects/vehicleAnalysis/config/tracker_config.txt"

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

class AnalysisStream : public QObject
{
    Q_OBJECT
public:
    explicit AnalysisStream(QObject *parent = 0);
    ~AnalysisStream();
    bool initPipeline();
    void start_analysis();
    void pause_analysis();
    void resume_analysis();
    void stop_analysis();

Q_SIGNALS:
    void startSuccess();
    void streamError();
    void sendResult(DetectionInfo);

private:
    void out_loop();                          //release pipeline
    void emitSignal(DetectionInfo detection);                        //to reliaze that static function send siganl
    void emitStreamError();                   //to reliaze that static function send siganl
    void emitStartSuccess();                  //to reliaze that static function send siganl

    static GstPadProbeReturn sink_pad_buffer_probe(GstPad *pad, GstPadProbeInfo *info, gpointer u_data);
    static GstPadProbeReturn source_bin_src_pad_buffer_probe (GstPad * pad, GstPadProbeInfo * info, gpointer u_data);
    static gpointer source_bin_gst_to_nvds_meta_transform_func(gpointer data, gpointer user_data);
    static void source_bin_gst_nvds_meta_release_func(gpointer data, gpointer user_data);
    static gpointer source_bin_meta_copy_func(gpointer data, gpointer user_data);
    static void source_bin_meta_release_func(gpointer data, gpointer user_data);
    static gboolean bus_call (GstBus * bus, GstMessage * msg, gpointer data);
    static void cb_newpad (GstElement * decodebin, GstPad * decoder_src_pad, gpointer data);
    static void decodebin_child_added (GstChildProxy * child_proxy, GObject * object, gchar * name, gpointer user_data);
    static GstElement* create_source_bin (guint index, gchar * uri);
    static gboolean set_tracker_properties (GstElement *nvtracker);
    static gchar *get_absolute_file_path (gchar *cfg_file_path, gchar *file_path);
    static char* _(char *c);
    static void destroy_sink_bin();
    static GstRTSPFilterResult client_filter (GstRTSPServer * server, GstRTSPClient * client, gpointer user_data);
    static gboolean start_rtsp_streaming (guint rtsp_port_num, guint updsink_port_num, guint64 udp_buffer_size);
    static gboolean timeout (GstRTSPServer * server);

    /*adding user data to the pipeline*/
    void * set_metadata_ptr();
    static gpointer copy_user_meta(gpointer data, gpointer user_data);
    static void release_user_meta(gpointer data, gpointer user_data);
    static GstPadProbeReturn nvinfer_src_pad_buffer_probe (GstPad * pad, GstPadProbeInfo * info, gpointer u_data);
    static GstPadProbeReturn osd_sink_pad_buffer_probe (GstPad * pad, GstPadProbeInfo * info, gpointer u_data);
    static void cb_need_data(GstElement *appsrc);
    static void start_feed (GstElement * source, guint size, gpointer user_data);
    static void stop_feed (GstElement * source, gpointer user_data);

    static AnalysisStream *static_analysisStream;
    static GstRTSPServer *server;
    GMainLoop *loop;
    GstElement *pipeline;
    guint bus_watch_id;
    guint sourceid = 0;                 //to determine the callback of the feed to the appsrc
    user_meta user_meta_data;
    bool activeRelease;
    bool pipelineDataFlow;
};

#endif // ANALYSISSTREAM_H
