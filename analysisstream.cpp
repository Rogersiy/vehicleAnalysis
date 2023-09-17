#include "analysisstream.h"
static gchar pgie_classes_str[4][32] = { "Vehicle", "TwoWheeler", "Person", "Roadsign"};
AnalysisStream* AnalysisStream::static_analysisStream = NULL;
GstRTSPServer* AnalysisStream::server = NULL;

AnalysisStream::AnalysisStream(QObject *parent):QObject(parent), pipeline(NULL), bus_watch_id(0), loop(NULL)
{
    static_analysisStream = this;
}

AnalysisStream::~AnalysisStream()
{
    static_analysisStream = NULL;
}

bool AnalysisStream::initPipeline(){
    qDebug() << "分析管道初始化中...";
    this->activeRelease = false;
    this->pipelineDataFlow = false;
    //input
    bool matFlag = false;                               //send the mat or h264 to the pipeline, now to support the mat format
    //analysis
    bool analysisFlag = false;                     //infer or not
    //output
    string outputForm = g_rtmp_client_output.toStdString();    //selectd: display/rtspServer/rtmp
    string hardwareCoding = "nvv4l2h264enc";          //hardware type encoding(nvv4l2h264enc/x264enc, outputForm != display
    guint udp_port  = 5400;
    guint rtsp_port = 8554;
    string rtmpAddress = g_rtmp_client_address.toStdString();

    GstElement *appsrc = NULL, *h264parser = NULL, *decoder = NULL, *capfilter = NULL, *nvvidconvsrc = NULL,
            *streammux = NULL, *nvvidconvosd = NULL, *nvosd = NULL, *transform = NULL, *sink = NULL;
    GstElement *pgie = NULL, *nvtracker = NULL;
    GstElement *nvvidconvrtsp = NULL, *filter4 = NULL, *h264enc = NULL;
    GstElement *rtppay = NULL;
    GstElement *parse = NULL, *flvmux;
    GstBus *bus = NULL;
    GstPad *osd_sink_pad = NULL;
    GstCaps *caps = NULL, *caps4 = NULL;
    GstCapsFeatures *feature = NULL;
    /* Add queue elements between every two elements */
    GstElement  *queue1 = NULL, *queue2= NULL, *queue3= NULL, *queue4= NULL, *queue5= NULL,
                *queue6= NULL, *queue7= NULL, *queue8= NULL, *queue9= NULL, *queue10= NULL;

    //get cuda device
    int current_device = -1;
    cudaGetDevice(&current_device);
    struct cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, current_device);

    /* Standard GStreamer initialization */
    gst_init(NULL, NULL);
    loop = g_main_loop_new(NULL, FALSE);

    /* Create gstreamer elements */
    /* Create Pipeline element that will form a connection of other elements */
    pipeline = gst_pipeline_new ("dstest1-pipeline");
    /* appsrc element for reading from the file */
    appsrc = gst_element_factory_make ("appsrc", "source");

    /*queue*/
    queue1 = gst_element_factory_make ("queue", "queue1");
    queue2 = gst_element_factory_make ("queue", "queue2");
    queue3 = gst_element_factory_make ("queue", "queue3");
    queue4 = gst_element_factory_make ("queue", "queue4");
    queue5 = gst_element_factory_make ("queue", "queue5");
    queue6 = gst_element_factory_make ("queue", "queue6");
    queue7 = gst_element_factory_make ("queue", "queue7");
    queue8 = gst_element_factory_make ("queue", "queue8");
    queue9 = gst_element_factory_make ("queue", "queue9");
    queue10 = gst_element_factory_make ("queue", "queue10");

    /* Create nvstreammux instance to form batches from one or more sources. */
    streammux = gst_element_factory_make ("nvstreammux", "stream-muxer");

    if (!pipeline || !appsrc || !streammux) {
        g_printerr ("pipeline, appsrc or streammux element could not be created. Exiting.\n");
        return -1;
    }

    // 给实时视频流打上时间戳
    //g_object_set (appsrc, "do-timestamp", TRUE,   NULL);
    g_signal_connect(appsrc, "need-data", G_CALLBACK(start_feed), NULL);
    //g_signal_connect (appsrc, "enough-data", G_CALLBACK (stop_feed), NULL);

    g_object_set (G_OBJECT (streammux), "batch-size", 1, NULL);
    g_object_set (G_OBJECT (streammux), "width", MUXER_OUTPUT_WIDTH, "height", MUXER_OUTPUT_HEIGHT, "batched-push-timeout", MUXER_BATCH_TIMEOUT_USEC, "live-source", TRUE, "attach-sys-ts", TRUE, NULL);

    //文字绘制
    /* Use convertor to convert from NV12 to RGBA as required by nvosd */
    nvvidconvosd = gst_element_factory_make ("nvvideoconvert", "nvvideo-converter");
    /* Create OSD to draw on the converted RGBA buffer */
    nvosd = gst_element_factory_make ("nvdsosd", "nv-onscreendisplay");
    if (!nvvidconvosd || !nvosd ) {
        g_printerr ("One element could not be created. Exiting.\n");
        return -1;
    }
    g_object_set (G_OBJECT (nvvidconvosd), "nvbuf-memory-type", 0, NULL);
    gst_bin_add_many (GST_BIN (pipeline), nvvidconvosd, nvosd, NULL);

    if(matFlag){
        //mat
        capfilter = gst_element_factory_make ("capsfilter", "capsfilters");

        nvvidconvsrc = gst_element_factory_make ("nvvideoconvert", "nvvideo-converter-src");

        if (!capfilter || !nvvidconvsrc) {
            g_printerr ("capfilter or nvvidconvsrc element could not be created. Exiting.\n");
            return -1;
        }

        g_object_set (appsrc, "caps", gst_caps_new_simple ("video/x-raw",
                        "format", G_TYPE_STRING, "RGBA",
                        "width", G_TYPE_INT, MUXER_OUTPUT_WIDTH,
                        "height", G_TYPE_INT, MUXER_OUTPUT_HEIGHT,
                        "framerate", GST_TYPE_FRACTION, 30, 1, NULL), NULL);

        caps = gst_caps_new_simple ("video/x-raw", "format", G_TYPE_STRING, "NV12", NULL);
        feature = gst_caps_features_new ("memory:NVMM", NULL);
        gst_caps_set_features (caps, 0, feature);
        g_object_set (G_OBJECT (capfilter), "caps", caps, NULL);

        g_object_set (G_OBJECT (nvvidconvsrc), "nvbuf-memory-type", 0, NULL);

        gst_bin_add_many (GST_BIN (pipeline), appsrc, nvvidconvsrc, capfilter, streammux, NULL);

        /*link the decoder to the streammux*/
        GstPad *sinkpad, *srcpad;
        gchar pad_name_sink[16] = "sink_0";
        gchar pad_name_src[16] = "src";

        sinkpad = gst_element_get_request_pad (streammux, pad_name_sink);
        if (!sinkpad) {
            g_printerr ("Streammux request sink pad failed. Exiting.\n");
            return -1;
        }

        srcpad = gst_element_get_static_pad (capfilter, pad_name_src);
        if (!srcpad) {
            g_printerr ("Decoder request src pad failed. Exiting.\n");
            return -1;
        }

        if (gst_pad_link (srcpad, sinkpad) != GST_PAD_LINK_OK) {
            g_printerr ("Failed to link decoder to stream muxer. Exiting.\n");
            return -1;
        }

        gst_object_unref (sinkpad);
        gst_object_unref (srcpad);

        if (!gst_element_link_many (appsrc, nvvidconvsrc, capfilter, NULL)) {
            g_printerr ("Elements could not be linked: 1. Exiting.\n");
            return -1;
        }
    }
    else{
        //h264
        /* Since the data format in the input file is elementary h264 stream,
         * we need a h264parser */
        h264parser = gst_element_factory_make ("h264parse", "h264-parser");

        /* Use nvdec_h264 for hardware accelerated decode on GPU */
        //decoder = gst_element_factory_make ("omxh264dec", "omxh264-decoder");
        decoder = gst_element_factory_make ("nvv4l2decoder", "nvv4l2-decoder");

        if (!h264parser || !decoder) {
            g_printerr ("h264parser or decoder element could not be created. Exiting.\n");
            return -1;
        }

        //appsrc config
        g_object_set(appsrc,
                    "do-timestamp", TRUE,
                    "is-live", TRUE,
                    "block", FALSE,
                    NULL);

        //decode disable dpb for low lantency
        g_object_set(decoder, "disable-dpb", TRUE, NULL);
        g_object_set (G_OBJECT (decoder), "enable-frame-type-reporting", true, NULL);
        g_object_set (G_OBJECT (decoder), "enable-max-performance", true, NULL);

        gst_bin_add_many (GST_BIN (pipeline), appsrc, queue1, h264parser, decoder, streammux, NULL);

        /*link the decoder to the streammux*/
        GstPad *sinkpad, *srcpad;
        gchar pad_name_sink[16] = "sink_0";
        gchar pad_name_src[16] = "src";

        sinkpad = gst_element_get_request_pad (streammux, pad_name_sink);
        if (!sinkpad) {
            g_printerr ("Streammux request sink pad failed. Exiting.\n");
            return -1;
        }

        srcpad = gst_element_get_static_pad (decoder, pad_name_src);
        if (!srcpad) {
            g_printerr ("Decoder request src pad failed. Exiting.\n");
            return -1;
        }

        if (gst_pad_link (srcpad, sinkpad) != GST_PAD_LINK_OK) {
            g_printerr ("Failed to link decoder to stream muxer. Exiting.\n");
            return -1;
        }

        gst_object_unref (sinkpad);
        gst_object_unref (srcpad);

        if (!gst_element_link_many (appsrc, queue1, h264parser, decoder, NULL)) {
            g_printerr ("Elements could not be linked: 1. Exiting.\n");
            return -1;
        }
    }

    if(analysisFlag){
        /* Use nvinfer to run inferencing on decoder's output,
         * behaviour of inferencing is set through config file */
        pgie = gst_element_factory_make ("nvinfer", "primary-nvinference-engine");
        if (!pgie) {
            g_printerr ("pgie element could not be created. Exiting.\n");
            return -1;
        }

        /* We need to have a tracker to track the identified objects */
        nvtracker = gst_element_factory_make ("nvtracker", "tracker");
        if (!nvtracker) {
            g_printerr ("nvtracker element could not be created. Exiting.\n");
            return -1;
        }

        /* Set all the necessary properties of the nvinfer element,
         * the necessary ones are : */
        g_object_set (G_OBJECT (pgie), "config-file-path", PGIE_CONFIG_FILE, NULL);
        /* skip frame */
        g_object_set (G_OBJECT (pgie), "interval", SKIP_FRAME);

        /* Set necessary properties of the tracker element. */
        if (!set_tracker_properties(nvtracker)) {
            g_printerr ("Failed to set tracker properties. Exiting.\n");
            return false;
        }

        gst_bin_add_many (GST_BIN (pipeline), pgie, nvtracker, NULL);
    }

    if(outputForm == "display"){
        if(prop.integrated){
            transform = gst_element_factory_make ("nvegltransform", "nvegl-transform");
            if(!transform) {
                g_printerr ("One tegra element could not be created. Exiting.\n");
                return -1;
            }
        }

        sink = gst_element_factory_make ("nveglglessink", "nvvideo-renderer"); //nveglglessink fakesink
        //g_object_set(G_OBJECT(sink), "sync", false, "qos", 0, NULL);

        if (!sink ) {
            g_printerr ("sink element could not be created. Exiting.\n");
            return -1;
        }

        gst_bin_add_many (GST_BIN (pipeline), queue2, queue3, queue4, queue5, queue6, queue7, transform, sink, NULL);

        if(analysisFlag){
            if (!gst_element_link_many (streammux, queue2, pgie, queue3, nvtracker, queue4, nvvidconvosd, queue5, nvosd, queue6, transform, queue7, sink, NULL)) {
                g_printerr ("Elements could not be linked: 2. Exiting.\n");
                return -1;
            }
        }
        else{
            if (!gst_element_link_many (streammux, queue2, nvvidconvosd, queue3, nvosd, queue4, transform, queue5, sink, NULL)) {
                g_printerr ("Elements could not be linked: 2. Exiting.\n");
                return -1;
            }
        }

        /* Lets add probe to get informed of the meta data generated, we add probe to
         * the sink pad of the osd element, since by that time, the buffer would have
         * had got all the metadata. */
        osd_sink_pad = gst_element_get_static_pad (nvosd, "sink");
        if (!osd_sink_pad)
            g_print ("Unable to get sink pad\n");
        else
            gst_pad_add_probe (osd_sink_pad, GST_PAD_PROBE_TYPE_BUFFER,
                               osd_sink_pad_buffer_probe, NULL, NULL);
        gst_object_unref (osd_sink_pad);
    }
    else{
        nvvidconvrtsp = gst_element_factory_make ("nvvideoconvert", "nvvideo-converter1");
        g_object_set (G_OBJECT (nvvidconvrtsp), "nvbuf-memory-type", 0, NULL);
        filter4 = gst_element_factory_make ("capsfilter", "filter4");
        if(hardwareCoding == "nvv4l2h264enc"){
            caps4 = gst_caps_from_string ("video/x-raw(memory:NVMM), format=I420");
        }
        else{
            caps4 = gst_caps_from_string ("video/x-raw, format=I420");
        }
        g_object_set (G_OBJECT (filter4), "caps", caps4, NULL);
        gst_caps_unref (caps4);
        if(hardwareCoding == "nvv4l2h264enc"){
            h264enc = gst_element_factory_make ("nvv4l2h264enc", "nvv4l2h264enc1");
            g_object_set (G_OBJECT (h264enc), "preset-level", 1, NULL);         //smaller to faster
            g_object_set (G_OBJECT (h264enc), "insert-sps-pps", 1, NULL);
            g_object_set (G_OBJECT (h264enc), "bufapi-version", 1, NULL);
            g_object_set (G_OBJECT (h264enc), "profile", 0, NULL);
            //g_object_set (G_OBJECT (h264enc), "bitrate", 1024, NULL);       //low bitrate->low quality  default 4000000
        }
        else{
            h264enc = gst_element_factory_make ("x264enc", "x264enc1");
            g_object_set (G_OBJECT (h264enc), "tune", 4, NULL);             //Zero latency
            g_object_set (G_OBJECT (h264enc), "bitrate", 1024, NULL);       //low bitrate->low quality
            //sg_object_set (G_OBJECT (h264enc), "bframes", 0, NULL);          //
            g_object_set (G_OBJECT (h264enc), "sliced-threads", TRUE, NULL);//Low latency but lower efficiency
            g_object_set (G_OBJECT (h264enc), "speed-preset", 2, NULL);     //smaller to faster
        }
        if (!nvvidconvrtsp || !h264enc || !filter4) {
            g_printerr ("One element could not be created.%p,%p,%p, Exiting.\n",nvvidconvrtsp, filter4, h264enc);
            return -1;
        }

        gst_bin_add_many (GST_BIN (pipeline), queue2, queue3, queue4, queue5, queue6, queue7, queue8, queue9, queue10, nvvidconvrtsp, filter4, h264enc, NULL);

        rtppay = gst_element_factory_make ("rtph264pay", "rtp-payer");
        sink = gst_element_factory_make("udpsink", "udp-sink");
        g_object_set(G_OBJECT(sink), "sync", false, "qos", 0, NULL);
        if (!rtppay || !sink) {
            g_printerr ("One element could not be created.%p,%p, Exiting.\n", rtppay, sink);
            return -1;
        }
        g_object_set (G_OBJECT (sink), "host", "127.0.0.1", "port", udp_port, NULL);

        gst_bin_add_many (GST_BIN (pipeline), rtppay, sink, NULL);

        if(analysisFlag){
            if (!gst_element_link_many (streammux, queue2, pgie, queue3, nvtracker, queue4, nvvidconvosd, queue5, nvosd, queue6, nvvidconvrtsp, queue7, filter4, queue8,
                                        h264enc, queue9, rtppay, queue10, sink, NULL)) {
                g_printerr ("Elements could not be linked: 3. Exiting.\n");
                return -1;
            }
        }
        else{
            if (!gst_element_link_many (streammux, queue3, nvvidconvosd, queue4, nvosd, queue5, nvvidconvrtsp, queue6, filter4, queue7,
                                        h264enc, queue8, rtppay, queue9, sink, NULL)) {
                g_printerr ("Elements could not be linked: 3. Exiting.\n");
                return -1;
            }
        }

        /* Lets add probe to get informed of the meta data generated, we add probe to
         * the sink pad of the osd element, since by that time, the buffer would have
         * had got all the metadata. */
        osd_sink_pad = gst_element_get_static_pad (nvosd, "sink");
        if (!osd_sink_pad)
            g_print ("Unable to get sink pad\n");
        else
            gst_pad_add_probe (osd_sink_pad, GST_PAD_PROBE_TYPE_BUFFER,
                               osd_sink_pad_buffer_probe, NULL, NULL);
        gst_object_unref (osd_sink_pad);

        if(outputForm == "rtspServer"){
            /*server init
             * */

            start_rtsp_streaming (rtsp_port/*rtsp_port*/, udp_port, 0);
        }
//        else if(outputForm == "rtmp"){
//            parse = gst_element_factory_make ("h264parse", "h264-parser2");
//            flvmux = gst_element_factory_make ("flvmux", "flvmux1");
//            sink = gst_element_factory_make("rtmpsink", "rtmpsink1");

//            if (!parse || !flvmux || !sink) {
//                g_printerr ("One element could not be created.%p,%p,%p, Exiting.\n", parse, flvmux, sink);
//                return -1;
//            }

//            gst_bin_add_many (GST_BIN (pipeline), parse, flvmux, sink, NULL);

//            g_object_set (G_OBJECT (sink), "location", (char *)rtmpAddress.c_str(), NULL);

//            if(analysisFlag){
//                if (!gst_element_link_many (streammux, pgie, nvvidconvosd, nvosd, nvvidconvrtsp, filter4, h264enc, parse, flvmux, sink, NULL)) {
//                    g_printerr ("Elements could not be linked: 3. Exiting.\n");
//                    return -1;
//                }
//            }
//            else{
//                if (!gst_element_link_many (streammux, nvvidconvosd, nvosd, nvvidconvrtsp, filter4, h264enc, parse, flvmux, sink, NULL)) {
//                    g_printerr ("Elements could not be linked: 3. Exiting.\n");
//                    return -1;
//                }
//            }

//            /* Lets add probe to get informed of the meta data generated, we add probe to
//             * the sink pad of the osd element, since by that time, the buffer would have
//             * had got all the metadata. */
//            osd_sink_pad = gst_element_get_static_pad (nvosd, "sink");
//            if (!osd_sink_pad)
//                g_print ("Unable to get sink pad\n");
//            else
//                gst_pad_add_probe (osd_sink_pad, GST_PAD_PROBE_TYPE_BUFFER,
//                                   osd_sink_pad_buffer_probe, NULL, NULL);
//            gst_object_unref (osd_sink_pad);
//        }
    }


    /* we add a message handler */
    bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));
    bus_watch_id = gst_bus_add_watch (bus, bus_call, loop);
    gst_object_unref (bus);

    /* dump graph on hup */
    //GST_DEBUG_BIN_TO_DOT_FILE_WITH_TS (GST_BIN (pipeline), GST_DEBUG_GRAPH_SHOW_ALL, "gst-launch.snapshot");

    qDebug() << "分析管道初始化成功!";
    return true;
}

void AnalysisStream::start_analysis(){
    qDebug() << "分析管道运行中...";
    gst_element_set_state (pipeline, GST_STATE_PLAYING);
    g_main_loop_run(loop);
    out_loop();
}

void AnalysisStream::pause_analysis(){
    gst_element_set_state(pipeline, GST_STATE_PAUSED);
}

void AnalysisStream::resume_analysis(){
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
}

void AnalysisStream::stop_analysis(){
    this->activeRelease = true;
    g_main_loop_quit(loop);
}

void AnalysisStream::out_loop(){
    qDebug() << "分析管道释放...";
    gst_element_set_state(pipeline, GST_STATE_NULL);
    if(g_rtmp_client_output == "rtspServer"){
        destroy_sink_bin();
    }
    g_usleep(5000000);                     //must wait for some time
    gst_object_unref(GST_OBJECT(pipeline));
    g_source_remove(bus_watch_id);
    g_main_loop_unref(loop);
    qDebug() << "分析管道已退出";
    if(static_analysisStream != NULL && !this->activeRelease){
        static_analysisStream->emitStreamError();
    }
}

void AnalysisStream::emitSignal(DetectionInfo detection){
    Q_EMIT sendResult(detection);
}

void AnalysisStream::emitStreamError(){
    Q_EMIT streamError();
}

void AnalysisStream::emitStartSuccess(){
    Q_EMIT startSuccess();
}

/*
 * static function!
 *
*/
GstPadProbeReturn AnalysisStream::source_bin_src_pad_buffer_probe (GstPad * pad, GstPadProbeInfo * info, gpointer u_data)
{
    GstBuffer *buf = (GstBuffer *) info->data;
    NvDsMeta *meta = NULL;

    user_meta *uri_meta = (user_meta *)g_malloc0(sizeof(user_meta));

    if(uri_meta == NULL)
    {
        cout << "GST_FLOW_ERROR" << endl;
        //return GST_FLOW_ERROR;
        return GST_PAD_PROBE_DROP;
    }
    /* Add dummy metadata */
    uri_meta->uri = ((user_meta *)u_data)->uri;

    /* Attach decoder metadata to gst buffer using gst_buffer_add_nvds_meta() */
    meta = gst_buffer_add_nvds_meta (buf, uri_meta, NULL,
      source_bin_meta_copy_func, source_bin_meta_release_func);

    /* Set metadata type */
    meta->meta_type = (GstNvDsMetaType)NVDS_SOURCE_BIN_GST_META;

    /* Set transform function to transform user metadata from Gst meta to
    * nvds meta */
    meta->gst_to_nvds_meta_transform_func = source_bin_gst_to_nvds_meta_transform_func;

    /* Set release function to release the transformed nvds metadata */
    meta->gst_to_nvds_meta_release_func = source_bin_gst_nvds_meta_release_func;

//    g_print("GST Dec Meta attached with gst decoder output buffer for Frame_Num = %d\n",
//      decoder_meta->frame_num);
//    g_print("Attached decoder Metadata: frame type = %d, frame_num = %d decode_error_status = %d\n\n",
//      decoder_meta->frame_type, decoder_meta->frame_num,
//      decoder_meta->dec_err);

    return GST_PAD_PROBE_OK;
}

/* gst to nvds transform function set by user. "data" holds a pointer to NvDsUserMeta */
gpointer AnalysisStream::source_bin_gst_to_nvds_meta_transform_func(gpointer data, gpointer user_data)
{
  NvDsUserMeta *meta = (NvDsUserMeta *)data;
  user_meta *src_meta = (user_meta *)meta->user_meta_data;
  user_meta *dst_meta = (user_meta *)source_bin_meta_copy_func(src_meta, NULL);
  return (gpointer)dst_meta;
}
/* release function set by user to release gst to nvds transformed metadata.
 * "data" holds a pointer to NvDsUserMeta */
void AnalysisStream::source_bin_gst_nvds_meta_release_func(gpointer data, gpointer user_data)
{
  NvDsUserMeta *meta = (NvDsUserMeta *) data;
  user_meta *u_meta = (user_meta *)meta->user_meta_data;
  source_bin_meta_release_func(u_meta, NULL);
}

/* gst meta copy function set by user */
gpointer AnalysisStream::source_bin_meta_copy_func(gpointer data, gpointer user_data)
{
    user_meta *src_meta = (user_meta *)data;
    user_meta *dst_meta = (user_meta*)g_malloc0(sizeof(user_meta));
    memcpy(dst_meta, src_meta, sizeof(user_meta));
    return (gpointer)dst_meta;
}

/* gst meta release function set by user */
void AnalysisStream::source_bin_meta_release_func(gpointer data, gpointer user_data)
{
    user_meta *meta = (user_meta *)data;
    if(meta) {
    g_free(meta);
    meta = NULL;
    }
}

gboolean AnalysisStream::bus_call (GstBus * bus, GstMessage * msg, gpointer data){
    GMainLoop *loop = (GMainLoop *) data;
    switch (GST_MESSAGE_TYPE (msg)) {
        case GST_MESSAGE_EOS:
        {
          qWarning () << "视频流结束!";
          g_main_loop_quit (loop);
          break;
        }
        case GST_MESSAGE_WARNING:
        {
          gchar *debug;
          GError *error;
          gst_message_parse_warning (msg, &error, &debug);
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
          qCritical() << "管道发生错误,ERROR from element:" << GST_OBJECT_NAME (msg->src) << "  " << error->message;
          g_print ("ERROR from element %s: %s\n", GST_OBJECT_NAME (msg->src), error->message);
          if (debug)
            qDebug() << "Error details:" << debug;
            g_print ("Error details: %s\n", debug);
          g_free (debug);
          g_error_free (error);
          break;
        }
        #ifndef PLATFORM_TEGRA
        case GST_MESSAGE_ELEMENT:
        {
          if (gst_nvmessage_is_stream_eos (msg)) {
            guint stream_id;
            if (gst_nvmessage_parse_stream_eos (msg, &stream_id)) {
              g_print ("Got EOS from stream %d\n", stream_id);
            }
          }
          break;
        }
        #endif
        default:
          break;
    }
    return TRUE;
}

void AnalysisStream::cb_newpad (GstElement * decodebin, GstPad * decoder_src_pad, gpointer data){
    GstCaps *caps = gst_pad_get_current_caps (decoder_src_pad);
    const GstStructure *str = gst_caps_get_structure (caps, 0);
    const gchar *name = gst_structure_get_name (str);
    GstElement *source_bin = (GstElement *) data;
    GstCapsFeatures *features = gst_caps_get_features (caps, 0);

    /* Need to check if the pad created by the decodebin is for video and not
    * audio. */
    if (!strncmp (name, "video", 5)) {
        /* Link the decodebin pad only if decodebin has picked nvidia
         * decoder plugin nvdec_*. We do this by checking if the pad caps contain
         * NVMM memory features. */
        if (gst_caps_features_contains (features, GST_CAPS_FEATURES_NVMM)) {
          /* Get the source bin ghost pad */
          GstPad *bin_ghost_pad = gst_element_get_static_pad (source_bin, "src");
          if (!gst_ghost_pad_set_target (GST_GHOST_PAD (bin_ghost_pad),
                  decoder_src_pad)) {
            g_printerr ("Failed to link decoder src pad to source bin ghost pad\n");
          }
          gst_object_unref (bin_ghost_pad);
        } else {
          g_printerr ("Error: Decodebin did not pick nvidia decoder plugin.\n");
        }
    }
}

void AnalysisStream::decodebin_child_added (GstChildProxy * child_proxy, GObject * object, gchar * name, gpointer user_data){
    g_print ("Decodebin child added: %s\n", name);
    if (g_strrstr (name, "decodebin") == name) {
    g_signal_connect (G_OBJECT (object), "child-added",
        G_CALLBACK (decodebin_child_added), user_data);
    }
}

GstElement* AnalysisStream::create_source_bin (guint index, gchar * uri){
    GstElement *bin = NULL, *uri_decode_bin = NULL;
    gchar bin_name[16] = { };

    g_snprintf (bin_name, 15, "source-bin-%02d", index);
    /* Create a source GstBin to abstract this bin's content from the rest of the
    * pipeline */
    bin = gst_bin_new (bin_name);

    /* Source element for reading from the uri.
    * We will use decodebin and let it figure out the container format of the
    * stream and the codec and plug the appropriate demux and decode plugins. */
    uri_decode_bin = gst_element_factory_make ("uridecodebin", "uri-decode-bin");

    if (!bin || !uri_decode_bin) {
    g_printerr ("One element in source bin could not be created.\n");
    return NULL;
    }

    /* We set the input uri to the source element */
    g_object_set (G_OBJECT (uri_decode_bin), "uri", uri, NULL);

    /* Connect to the "pad-added" signal of the decodebin which generates a
    * callback once a new pad for raw data has beed created by the decodebin */
    g_signal_connect (G_OBJECT (uri_decode_bin), "pad-added",
      G_CALLBACK (cb_newpad), bin);
    g_signal_connect (G_OBJECT (uri_decode_bin), "child-added",
      G_CALLBACK (decodebin_child_added), bin);

    gst_bin_add (GST_BIN (bin), uri_decode_bin);

    /* We need to create a ghost pad for the source bin which will act as a proxy
    * for the video decoder src pad. The ghost pad will not have a target right
    * now. Once the decode bin creates the video decoder and generates the
    * cb_newpad callback, we will set the ghost pad target to the video decoder
    * src pad. */
    if (!gst_element_add_pad (bin, gst_ghost_pad_new_no_target ("src",
              GST_PAD_SRC))) {
    g_printerr ("Failed to add ghost pad in source bin\n");
    return NULL;
    }

    return bin;
}

gboolean AnalysisStream::set_tracker_properties (GstElement *nvtracker){
    gboolean ret = FALSE;
    GError *error = NULL;
    gchar **keys = NULL;
    gchar **key = NULL;
    GKeyFile *key_file = g_key_file_new ();

    if (!g_key_file_load_from_file (key_file, TRACKER_CONFIG_FILE, G_KEY_FILE_NONE,
            &error)) {
      g_printerr ("Failed to load config file: %s\n", error->message);
      return FALSE;
    }

    keys = g_key_file_get_keys (key_file, CONFIG_GROUP_TRACKER, NULL, &error);
    CHECK_ERROR (error);

    for (key = keys; *key; key++) {
      if (!g_strcmp0 (*key, CONFIG_GROUP_TRACKER_WIDTH)) {
        gint width =
            g_key_file_get_integer (key_file, CONFIG_GROUP_TRACKER,
            CONFIG_GROUP_TRACKER_WIDTH, &error);
        CHECK_ERROR (error);
        g_object_set (G_OBJECT (nvtracker), "tracker-width", width, NULL);
      } else if (!g_strcmp0 (*key, CONFIG_GROUP_TRACKER_HEIGHT)) {
        gint height =
            g_key_file_get_integer (key_file, CONFIG_GROUP_TRACKER,
            CONFIG_GROUP_TRACKER_HEIGHT, &error);
        CHECK_ERROR (error);
        g_object_set (G_OBJECT (nvtracker), "tracker-height", height, NULL);
      } else if (!g_strcmp0 (*key, CONFIG_GPU_ID)) {
        guint gpu_id =
            g_key_file_get_integer (key_file, CONFIG_GROUP_TRACKER,
            CONFIG_GPU_ID, &error);
        CHECK_ERROR (error);
        g_object_set (G_OBJECT (nvtracker), "gpu_id", gpu_id, NULL);
      } else if (!g_strcmp0 (*key, CONFIG_GROUP_TRACKER_LL_CONFIG_FILE)) {
        char* ll_config_file = get_absolute_file_path (TRACKER_CONFIG_FILE,
                  g_key_file_get_string (key_file,
                      CONFIG_GROUP_TRACKER,
                      CONFIG_GROUP_TRACKER_LL_CONFIG_FILE, &error));
        CHECK_ERROR (error);
        g_object_set (G_OBJECT (nvtracker), "ll-config-file", ll_config_file, NULL);
      } else if (!g_strcmp0 (*key, CONFIG_GROUP_TRACKER_LL_LIB_FILE)) {
        char* ll_lib_file = get_absolute_file_path (TRACKER_CONFIG_FILE,
                  g_key_file_get_string (key_file,
                      CONFIG_GROUP_TRACKER,
                      CONFIG_GROUP_TRACKER_LL_LIB_FILE, &error));
        CHECK_ERROR (error);
        g_object_set (G_OBJECT (nvtracker), "ll-lib-file", ll_lib_file, NULL);
      } else if (!g_strcmp0 (*key, CONFIG_GROUP_TRACKER_ENABLE_BATCH_PROCESS)) {
        gboolean enable_batch_process =
            g_key_file_get_integer (key_file, CONFIG_GROUP_TRACKER,
            CONFIG_GROUP_TRACKER_ENABLE_BATCH_PROCESS, &error);
        CHECK_ERROR (error);
        g_object_set (G_OBJECT (nvtracker), "enable_batch_process",
                      enable_batch_process, NULL);
      } else {
        g_printerr ("Unknown key '%s' for group [%s]", *key,
            CONFIG_GROUP_TRACKER);
      }
    }

    ret = TRUE;
  done:
    if (error) {
      g_error_free (error);
    }
    if (keys) {
      g_strfreev (keys);
    }
    if (!ret) {
      g_printerr ("%s failed", __func__);
    }
    return ret;
}

gchar* AnalysisStream::get_absolute_file_path (gchar *cfg_file_path, gchar *file_path)
{
  gchar abs_cfg_path[PATH_MAX + 1];
  gchar *abs_file_path;
  gchar *delim;

  if (file_path && file_path[0] == '/') {
    return file_path;
  }

  if (!realpath (cfg_file_path, abs_cfg_path)) {
    g_free (file_path);
    return NULL;
  }

  // Return absolute path of config file if file_path is NULL.
  if (!file_path) {
    abs_file_path = g_strdup (abs_cfg_path);
    return abs_file_path;
  }

  delim = g_strrstr (abs_cfg_path, "/");
  *(delim + 1) = '\0';

  abs_file_path = g_strconcat (abs_cfg_path, file_path, NULL);
  g_free (file_path);

  return abs_file_path;
}

char* AnalysisStream::_(char *c)
{
    return g_locale_to_utf8(c,-1,NULL,NULL,NULL);
}
/*
 * rtsp server related
*/
void AnalysisStream::destroy_sink_bin(){
    GstRTSPMountPoints *mounts;
    GstRTSPSessionPool *pool;

    mounts = gst_rtsp_server_get_mount_points (server);
    gst_rtsp_mount_points_remove_factory (mounts, "/ds-test");
    g_object_unref (mounts);
    gst_rtsp_server_client_filter (server, client_filter, NULL);
    pool = gst_rtsp_server_get_session_pool (server);
    gst_rtsp_session_pool_cleanup (pool);
    g_object_unref (pool);
}

GstRTSPFilterResult AnalysisStream::client_filter (GstRTSPServer * server, GstRTSPClient * client, gpointer user_data){
    return GST_RTSP_FILTER_REMOVE;
}

gboolean AnalysisStream::start_rtsp_streaming (guint rtsp_port_num, guint updsink_port_num, guint64 udp_buffer_size){
    GstRTSPMountPoints *mounts;
    GstRTSPMediaFactory *factory;
    char udpsrc_pipeline[512];

    char port_num_Str[64] = { 0 };
    char *encoder_name;

    if (udp_buffer_size == 0)
        udp_buffer_size = 1024 * 1024;

    sprintf (udpsrc_pipeline,
             "( udpsrc name=pay0 port=%d buffer-size=%lu caps=\"application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96 \" )",
             updsink_port_num, udp_buffer_size);

    sprintf (port_num_Str, "%d", rtsp_port_num);

    server = gst_rtsp_server_new ();
    g_object_set (server, "service", port_num_Str, NULL);

    mounts = gst_rtsp_server_get_mount_points (server);

    factory = gst_rtsp_media_factory_new ();
    gst_rtsp_media_factory_set_launch (factory, udpsrc_pipeline);

    gst_rtsp_mount_points_add_factory (mounts, "/ds-test", factory);

    g_object_unref (mounts);

    gst_rtsp_server_attach (server, NULL);

    g_timeout_add_seconds (2, (GSourceFunc) timeout, server);

    g_print
            ("\n *** DeepStream: Launched RTSP Streaming at rtsp://localhost:%d/ds-test ***\n\n",
             rtsp_port_num);

    return TRUE;
}

gboolean AnalysisStream::timeout (GstRTSPServer * server)
{
  GstRTSPSessionPool *pool;

  pool = gst_rtsp_server_get_session_pool (server);
  gst_rtsp_session_pool_cleanup (pool);
  g_object_unref (pool);

  return TRUE;
}


/*
 * add data to the pipeline
*/
void* AnalysisStream::set_metadata_ptr()
{
  int i = 0;
  gchar *user_metadata = (gchar*)g_malloc0(USER_ARRAY_SIZE);

  g_print("\n**************** Setting user metadata array of 16 on nvinfer src pad\n");
  for(i = 0; i < USER_ARRAY_SIZE; i++) {
    user_metadata[i] = rand() % 255;
    g_print("user_meta_data [%d] = %d\n", i, user_metadata[i]);
  }
  return (void *)user_metadata;
}

/* copy function set by user. "data" holds a pointer to NvDsUserMeta*/
gpointer AnalysisStream::copy_user_meta(gpointer data, gpointer user_data)
{
  NvDsUserMeta *user_meta = (NvDsUserMeta *)data;
  gchar *src_user_metadata = (gchar*)user_meta->user_meta_data;
  gchar *dst_user_metadata = (gchar*)g_malloc0(USER_ARRAY_SIZE);
  memcpy(dst_user_metadata, src_user_metadata, USER_ARRAY_SIZE);
  return (gpointer)dst_user_metadata;
}

/* release function set by user. "data" holds a pointer to NvDsUserMeta*/
void AnalysisStream::release_user_meta(gpointer data, gpointer user_data)
{
  NvDsUserMeta *user_meta = (NvDsUserMeta *) data;
  if(user_meta->user_meta_data) {
    g_free(user_meta->user_meta_data);
    user_meta->user_meta_data = NULL;
  }
}

/* Set nvds user metadata at frame level. User need to set 4 parameters after
 * acquring user meta from pool using nvds_acquire_user_meta_from_pool().
 *
 * Below parameters are required to be set.
 * 1. user_meta_data : pointer to User specific meta data
 * 2. meta_type: Metadata type that user sets to identify its metadata
 * 3. copy_func: Metadata copy or transform function to be provided when there
 *               is buffer transformation
 * 4. release_func: Metadata release function to be provided when it is no
 *                  longer required.
 *
 * osd_sink_pad_buffer_probe  will extract metadata received on OSD sink pad
 * and update params for drawing rectangle, object information etc. */

GstPadProbeReturn AnalysisStream::nvinfer_src_pad_buffer_probe (GstPad * pad, GstPadProbeInfo * info, gpointer u_data)
{
  GstBuffer *buf = (GstBuffer *) info->data;
  NvDsMetaList * l_frame = NULL;
  NvDsUserMeta *user_meta = NULL;
  NvDsMetaType user_meta_type = NVDS_USER_FRAME_META_EXAMPLE;

  NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta (buf);

    for (l_frame = batch_meta->frame_meta_list; l_frame != NULL;
      l_frame = l_frame->next) {
        NvDsFrameMeta *frame_meta = (NvDsFrameMeta *) (l_frame->data);

        /* Acquire NvDsUserMeta user meta from pool */
        user_meta = nvds_acquire_user_meta_from_pool(batch_meta);

        /* Set NvDsUserMeta below */
        user_meta->user_meta_data = (void *)static_analysisStream->set_metadata_ptr();
        user_meta->base_meta.meta_type = user_meta_type;
        user_meta->base_meta.copy_func = (NvDsMetaCopyFunc)copy_user_meta;
        user_meta->base_meta.release_func = (NvDsMetaReleaseFunc)release_user_meta;

        /* We want to add NvDsUserMeta to frame level */
        nvds_add_user_meta_to_frame(frame_meta, user_meta);
    }
    return GST_PAD_PROBE_OK;
}

GstPadProbeReturn AnalysisStream::sink_pad_buffer_probe(GstPad *pad, GstPadProbeInfo *info, gpointer u_data){
    GstBuffer *buf = (GstBuffer *) info->data;
    if(buf == NULL)
    {
        return GST_PAD_PROBE_DROP;
    }

    NvDsMetaList * l_frame = NULL;
    NvDsMetaList * l_user_meta = NULL;
    NvDsMetaList * l_obj = NULL;
    NvDsObjectMeta *obj_meta = NULL;
    NvDsUserMeta *u_meta = NULL;
    user_meta *uri_meta = NULL;
    guint idx = 0;

    GstMapInfo in_map_info;
    NvBufSurface *surface = NULL;
    memset (&in_map_info, 0, sizeof (in_map_info));
    if (!gst_buffer_map (buf, &in_map_info, GST_MAP_READ)) {
        g_print ("Error: Failed to map gst buffer\n");
        gst_buffer_unmap (buf, &in_map_info);
        return GST_PAD_PROBE_DROP;
    }
    surface = (NvBufSurface *) in_map_info.data;
    int gpu = surface->gpuId;
    int batch_size= surface->batchSize;

    NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta (buf);
    if (batch_meta == nullptr)
    {
        g_print ("NvDsBatchMeta not found for input buffer.");
        gst_buffer_unmap (buf, &in_map_info);
        return GST_PAD_PROBE_DROP;
    }

    for (l_frame = batch_meta->frame_meta_list; l_frame != NULL; l_frame = l_frame->next) {
        NvDsFrameMeta *frame_meta = (NvDsFrameMeta *) (l_frame->data);

        gboolean is_infered = frame_meta->bInferDone;
        if(!is_infered)
        {
            idx++;
            continue;
        }
        idx++;
    }
    gst_buffer_unmap (buf, &in_map_info);
    return GST_PAD_PROBE_OK;
}

/* osd */
GstPadProbeReturn AnalysisStream::osd_sink_pad_buffer_probe (GstPad * pad, GstPadProbeInfo * info, gpointer u_data)
{
    GstBuffer *buf = (GstBuffer *) info->data;
    if(buf == NULL)
    {
        return GST_PAD_PROBE_DROP;
    }

    guint num_rects = 0;
    guint vehicle_count = 0;
    guint person_count = 0;
    NvDsMetaList * l_frame = NULL;
    NvDsMetaList * l_obj = NULL;
    NvDsObjectMeta *obj_meta = NULL;
    NvDsDisplayMeta *display_meta = NULL;
    guint idx = 0;

    GstMapInfo in_map_info;
    NvBufSurface *surface = NULL;
    memset (&in_map_info, 0, sizeof (in_map_info));
    if (!gst_buffer_map (buf, &in_map_info, GST_MAP_READ)) {
        g_print ("Error: Failed to map gst buffer\n");
        gst_buffer_unmap (buf, &in_map_info);
        return GST_PAD_PROBE_DROP;
    }
    surface = (NvBufSurface *) in_map_info.data;
    int gpu = surface->gpuId;
    int batch_size= surface->batchSize;

    NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta (buf);
    if (batch_meta == nullptr)
    {
        g_print ("NvDsBatchMeta not found for input buffer.");
        gst_buffer_unmap (buf, &in_map_info);
        return GST_PAD_PROBE_DROP;
    }

    if(!static_analysisStream->pipelineDataFlow){
        static_analysisStream->startSuccess();
        static_analysisStream->pipelineDataFlow = true;
    }

    for (l_frame = batch_meta->frame_meta_list; l_frame != NULL;
      l_frame = l_frame->next) {
        NvDsFrameMeta *frame_meta = (NvDsFrameMeta *) (l_frame->data);

        gboolean is_infered = frame_meta->bInferDone;
        if(is_infered)
        {
            DetectionInfo detection;
            std::vector<ObjectInfo> objectList;
            for (l_obj = frame_meta->obj_meta_list; l_obj != NULL;
                    l_obj = l_obj->next) {
                //目标检测及跟踪信息
                obj_meta = (NvDsObjectMeta *) (l_obj->data);
                //object info
                ObjectInfo object;
                //class id
                object.classId = obj_meta->class_id;
                //class name
                string name = obj_meta->obj_label;
                // name.pop_back();            // delete the last char
                object.className = name;
                //confidence
                object.confidence = obj_meta->confidence;
                //object id
                object.id = obj_meta->object_id;
                //deal with BBoxs
                cv::Rect bounding_rect;
                bounding_rect.x = obj_meta->rect_params.left;
                bounding_rect.y = obj_meta->rect_params.top;
                bounding_rect.width = obj_meta->rect_params.width;
                bounding_rect.height = obj_meta->rect_params.height;
                object.rect = bounding_rect;
                objectList.push_back(object);
            }
            detection.objectList = objectList;

            //获取图片数据
            cudaError_t cuda_err;
            if(cudaSetDevice (gpu) != cudaSuccess)
            {
                 g_print("ERROR: Failed to Set Cuda Device.\n");
                 //output_to_log("ERROR: Failed to Set Cuda Device.");
                 gst_buffer_unmap (buf, &in_map_info);
                 return GST_PAD_PROBE_DROP;
            }
            cudaStream_t cuda_stream;
            if(cudaStreamCreate(&cuda_stream) != cudaSuccess)
            {
                 g_print("ERROR: Failed to Create Cuda Stream.\n");
                 //output_to_log("ERROR: Failed to Create Cuda Stream.");
                 gst_buffer_unmap (buf, &in_map_info);
                 return GST_PAD_PROBE_DROP;
            }

            NvBufSurfTransform_Error err;
            NvBufSurfTransformConfigParams transform_config_params;
            NvBufSurfTransformParams transform_params;
            NvBufSurfTransformRect src_rect;
            NvBufSurfTransformRect dst_rect;
            NvBufSurface ip_surf;
            ip_surf = *surface;

            ip_surf.numFilled = ip_surf.batchSize = 1;
            ip_surf.surfaceList = &(surface->surfaceList[idx]);

            src_rect.top   = 0;
            src_rect.left  = 0;
            src_rect.width = (guint) ip_surf.surfaceList->width;
            src_rect.height= (guint) ip_surf.surfaceList->height;
            dst_rect.top   = 0;
            dst_rect.left  = 0;
            dst_rect.width = (guint) ip_surf.surfaceList->width;
            dst_rect.height= (guint) ip_surf.surfaceList->height;

            NvBufSurface *dst_surface = NULL;
            NvBufSurfaceCreateParams create_params;
            /* An intermediate buffer for NV12/RGBA to BGR conversion  will be required. Can be skipped if custom algorithm can work directly on NV12/RGBA. */
            create_params.gpuId  = surface->gpuId;
            create_params.width  = (gint) ip_surf.surfaceList->width;
            create_params.height = (gint) ip_surf.surfaceList->height;
            create_params.size = 0;
            create_params.colorFormat = NVBUF_COLOR_FORMAT_RGBA;
            create_params.layout = NVBUF_LAYOUT_PITCH;
            #ifdef __aarch64__
              create_params.memType = NVBUF_MEM_DEFAULT;
            #else
              create_params.memType = NVBUF_MEM_CUDA_UNIFIED;
            #endif

            if (NvBufSurfaceCreate (&dst_surface, 1, &create_params) != 0)
            {
              //output_to_log("ERROR: Failed to Create surface.");
              g_print("ERROR: Failed to Create surface.\n");
              gst_buffer_unmap (buf, &in_map_info);
              return GST_PAD_PROBE_DROP;
            }

            /* Configure transform session parameters for the transformation */
            transform_config_params.compute_mode = NvBufSurfTransformCompute_Default;
            transform_config_params.gpu_id = surface->gpuId;

            //        cudaError_t cuda_err;
            //        cuda_err = cudaSetDevice (surface->gpuId);
            //        cudaStream_t cuda_stream;
            //        cuda_err = cudaStreamCreate (&cuda_stream);
            transform_config_params.cuda_stream = cuda_stream;
            /* Set the transform session parameters for the conversions executed in this
             * thread. */
            err = NvBufSurfTransformSetSessionParams (&transform_config_params);
            if (err != NvBufSurfTransformError_Success)
            {
              //output_to_log("NvBufSurfTransformSetSessionParams failed with error " + std::to_string(int(err)));
              g_print ("NvBufSurfTransformSetSessionParams failed with error %d", err);
              gst_buffer_unmap (buf, &in_map_info);
              return GST_PAD_PROBE_DROP;
            }

            /* Set the transform parameters */
            transform_params.src_rect = &src_rect;
            transform_params.dst_rect = &dst_rect;
            transform_params.transform_flag = NVBUFSURF_TRANSFORM_FILTER | NVBUFSURF_TRANSFORM_CROP_SRC | NVBUFSURF_TRANSFORM_CROP_DST;
            transform_params.transform_filter = NvBufSurfTransformInter_Default;

            /* Memset the memory */
            NvBufSurfaceMemSet (dst_surface, 0, 0, 0);
            err = NvBufSurfTransform (&ip_surf, dst_surface, &transform_params);
            if (err != NvBufSurfTransformError_Success)
            {
                //output_to_log ("NvBufSurfTransform failed with error %d while converting buffer " + std::to_string(int(err)));
                g_print ("NvBufSurfTransform failed with error %d while converting buffer\n", err);
                gst_buffer_unmap (buf, &in_map_info);
                return GST_PAD_PROBE_DROP;
            }

            /* Map the buffer so that it can be accessed by CPU */
            if (NvBufSurfaceMap (dst_surface, 0, 0, NVBUF_MAP_READ) != 0)
            {
                //output_to_log("NvBufSurfaceMap failed with error.");
                g_print ("NvBufSurfaceMap failed with error.\n");
                gst_buffer_unmap (buf, &in_map_info);
                return GST_PAD_PROBE_DROP;
            }
            /* Cache the mapped data for CPU access */
            NvBufSurfaceSyncForCpu (dst_surface, 0, 0);

            // save image
            cv::Mat bgr_frame = cv::Mat (cv::Size(create_params.width, create_params.height), CV_8UC3);
            cv::Mat in_mat = cv::Mat (create_params.height, create_params.width, CV_8UC4, dst_surface->surfaceList[0].mappedAddr.addr[0], dst_surface->surfaceList[0].pitch);
            cv::cvtColor (in_mat, bgr_frame, cv::COLOR_RGBA2BGR);

            detection.width = create_params.width;
            detection.height = create_params.height;
            detection.pic = bgr_frame;

            static_analysisStream->emitSignal(detection);

            //release buffer
            if(NvBufSurfaceUnMap (dst_surface, 0, 0))
            {
                //output_to_log("NvBufSurfaceUnMap failed with error.");
                g_print ("NvBufSurfacUneMap failed with error.\n");
                gst_buffer_unmap (buf, &in_map_info);
                return GST_PAD_PROBE_DROP;
            }
            if(NvBufSurfaceDestroy (dst_surface))
            {
                //output_to_log("NvBufSurfaceDestroy failed with error.");
                g_print ("NvBufSurfaceDestroy failed with error.\n");
                gst_buffer_unmap (buf, &in_map_info);
                return GST_PAD_PROBE_DROP;
            }
            if(cudaStreamDestroy (cuda_stream) != cudaSuccess)
            {
                 g_print("ERROR: Failed to Destory Cuda Stream.\n");
                 //output_to_log("ERROR: Failed to Destory Cuda Stream.");
                 gst_buffer_unmap (buf, &in_map_info);
                 return GST_PAD_PROBE_DROP;
            }

        }

        //osd时间显示
        int offset = 0;
        display_meta = nvds_acquire_display_meta_from_pool(batch_meta);
        NvOSD_TextParams *txt_params  = &display_meta->text_params[0];
        display_meta->num_labels = 1;
        txt_params->display_text =  (char*)g_malloc0 (MAX_DISPLAY_LEN);
//        offset = snprintf(txt_params->display_text, MAX_DISPLAY_LEN, "Person = %d ", person_count);
//        offset = snprintf(txt_params->display_text + offset , MAX_DISPLAY_LEN, "Vehicle = %d ", vehicle_count);
        string time = (QDateTime::currentDateTime().toString("yyyy年MM月dd日 hh:mm:ss")).toStdString();
        offset = snprintf(txt_params->display_text, MAX_DISPLAY_LEN, ((char*)time.c_str()));

        /* Now set the offsets where the string should appear */
        txt_params->x_offset = 10;
        txt_params->y_offset = 8;

        /* Font , font-color and font-size */
        txt_params->font_params.font_name = "Serif";
        txt_params->font_params.font_size = 14;
        txt_params->font_params.font_color.red = 1.0;
        txt_params->font_params.font_color.green = 1.0;
        txt_params->font_params.font_color.blue = 1.0;
        txt_params->font_params.font_color.alpha = 1.0;

        /* Text background color */
        txt_params->set_bg_clr = 1;
        txt_params->text_bg_clr.red = 0.0;
        txt_params->text_bg_clr.green = 0.0;
        txt_params->text_bg_clr.blue = 0.0;
        txt_params->text_bg_clr.alpha = 1.0;

        nvds_add_display_meta_to_frame(frame_meta, display_meta);

        idx++;
    }

//    g_print ("Number of objects = %d Vehicle Count = %d Person Count = %d\n", num_rects, vehicle_count, person_count);
    gst_buffer_unmap (buf, &in_map_info);
    return GST_PAD_PROBE_OK;
}

/* This signal callback triggers when appsrc needs data. Here,
 * we add an idle handler to the mainloop to start pushing
 * data into the appsrc */
void AnalysisStream::start_feed (GstElement * source, guint size, gpointer user_data)
{
//  if (static_analysisStream->sourceid == 0) {
//    static_analysisStream->sourceid = g_idle_add ((GSourceFunc) cb_need_data, source);
//  }
    cb_need_data(source);
}

/* This callback triggers when appsrc has enough data and we can stop sending.
 * We remove the idle handler from the mainloop */
void AnalysisStream::stop_feed (GstElement * source, gpointer user_data)
{
//  if (static_analysisStream->sourceid != 0) {
//    g_source_remove (static_analysisStream->sourceid);
//    static_analysisStream->sourceid = 0;
//  }
}

/*add data to the source*/
void AnalysisStream::cb_need_data(GstElement *appsrc){
    /*
     * send h264 to the pipeline, not success, need discuss
    */
    static GstClockTime timestamp = 0;
    GstFlowReturn ret;
    GstMapInfo map;

    // 给互斥量加锁
    pthread_mutex_lock(&myMutex);

    // 判断条件变量是否满足 访问公共区条件
    while (0 == loopqueue.getSize())
    {
        pthread_cond_wait(&cond, &myMutex);
    }
    // 访问公共区, 取数据
    msg* picData = loopqueue.outLoopQueue();
    // 往appsrc输入数据
    GstBuffer *buf = gst_buffer_new_allocate(NULL, picData->len, NULL);
    gst_buffer_map(buf, &map, GST_MAP_WRITE);
    memcpy((guchar *)map.data, (guchar *)(picData->data), picData->len);
    map.size = picData->len;

    //printf("bufLen = %d, loopqueue size = %d\n", picData->len, loopqueue.getSize());
    // 给互斥量解锁
    pthread_mutex_unlock(&myMutex);

    GST_BUFFER_PTS(buf) = timestamp;
    GST_BUFFER_DURATION(buf) = gst_util_uint64_scale_int(1, GST_SECOND, 1);
    timestamp += GST_BUFFER_DURATION(buf);
    g_signal_emit_by_name(appsrc, "push-buffer", buf, &ret);
    //ret = gst_app_src_push_buffer(appsrc, buf);
    if (ret != GST_FLOW_OK) {
      g_print ("gst_app_src_push_buffer returned %d \n", ret);
      static_analysisStream->stop_analysis();
    }
    gst_buffer_unmap (buf, &map);
    gst_buffer_unref(buf);
    delete picData;

    /*
     * send mat to the pipeline
    */
//    static GstClockTime timestamp = 0;
//    GstFlowReturn ret;
//    GstMapInfo map;
//    // 给互斥量加锁
//    pthread_mutex_lock(&myMutex);
//    // 判断条件变量是否满足 访问公共区条件
//    while (0 == loopqueueImage.getSize())
//    {
//        pthread_cond_wait(&cond, &myMutex);
//    }
//    // 访问公共区, 取数据
//    imgmsg *picData = loopqueueImage.outLoopQueue();
//    if(picData->width != MUXER_OUTPUT_WIDTH || picData->height != MUXER_OUTPUT_HEIGHT){
//        cv::resize(picData->img, picData->img, cv::Size(MUXER_OUTPUT_WIDTH, MUXER_OUTPUT_HEIGHT));
//    }
//    // 往appsrc输入数据
//    gsize sizeInBytes = picData->height*picData->width*picData->channels;
//    GstBuffer *buf = gst_buffer_new_allocate (NULL, sizeInBytes, NULL);;
//    gst_buffer_map(buf, &map, GST_MAP_WRITE);
//    memcpy((guchar *)map.data, (guchar *)(picData->img.data), sizeInBytes);
//    map.size = sizeInBytes;
//    // 给互斥量解锁
//    pthread_mutex_unlock(&myMutex);
//    //send data
//    GST_BUFFER_PTS(buf) = timestamp;
//    GST_BUFFER_DURATION(buf) = gst_util_uint64_scale_int(1, GST_SECOND, 1);
//    timestamp += GST_BUFFER_DURATION(buf);
//    g_signal_emit_by_name(appsrc, "push-buffer", buf, &ret);
//    if (ret != GST_FLOW_OK) {
//      g_print ("gst_app_src_push_buffer returned %d \n", ret);
//      static_analysisStream->stop_analysis();
//    }
//    //release
//    gst_buffer_unmap (buf, &map);
//    gst_buffer_unref(buf);
//    delete picData;

    /*
     * reading local picture!!!
    */
    /*
    static GstClockTime timestamp = 0;
    guint size,depth,height,width,step,channels;
    GstFlowReturn ret ;
    IplImage* img;
    guchar *data1;
    GstMapInfo map;

    cv::Mat imgMat = imread("/opt/nvidia/deepstream/deepstream-6.0/samples/streams/sample_720p.jpg",cv::IMREAD_COLOR);
    cv::resize(imgMat, imgMat, cv::Size(MUXER_OUTPUT_WIDTH, MUXER_OUTPUT_HEIGHT));
    cvtColor(imgMat,imgMat,cv::COLOR_BGR2RGBA);
    IplImage imgIpl = imgMat;
    img = &imgIpl;

    height    = img->height;
    width     = img->width;
    step      = img->widthStep;
    channels  = img->nChannels;
    depth     = img->depth;
    data1      = (guchar *)img->imageData;
    size = height*width*channels;

    GstBuffer *buffer = NULL;
    buffer = gst_buffer_new_allocate (NULL, size, NULL);
    gst_buffer_map (buffer, &map, GST_MAP_WRITE);
    memcpy( (guchar *)map.data, data1,  gst_buffer_get_size( buffer ) );
//    GST_BUFFER_PTS (buffer) = timestamp;
//    GST_BUFFER_DURATION (buffer) = gst_util_uint64_scale_int (1, GST_SECOND, 1);
//    timestamp += GST_BUFFER_DURATION (buffer);
    g_signal_emit_by_name (appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unmap (buffer, &map);
    gst_buffer_unref(buffer);

    if (ret != GST_FLOW_OK) {
    g_print("quit");
        qDebug() << "pipeline error......";
        static_analysisStream->stop_analysis();
    }
    */
}

