#include "analysisstream.h"
AnalysisStream* AnalysisStream::static_analysisStream = NULL;
GstRTSPServer* AnalysisStream::server = NULL;

AnalysisStream::AnalysisStream(string analysisType, string outPut, string rtmpAddress, QObject *parent):QObject(parent), pipeline(NULL), bus_watch_id(0), loop(NULL)
{
    static_analysisStream = this;

    this->analysisType = analysisType;
    this->outPut = outPut;
    this->rtmpAddress = rtmpAddress;

    //初始化共享内存
    //获得或创建一个共享内存标识符
    this->shmid = shmget((key_t)1121, sizeof(StreamShmMsg), 0666|IPC_CREAT);
    if(this->shmid == -1)        //获取或创建一个共享内存标识符失败
    {
        qCritical() << "共享内存创建失败,程序退出";
        exit(EXIT_FAILURE);
    }
    this->shm = shmat(this->shmid, (void*)0, 0);        //返回共享存储段连接的实际地址
    if(this->shm == (void*)-1)
    {
        qCritical() << "共享存储段实际地址获取失败,程序退出";
        exit(EXIT_FAILURE);
    }
    g_shared_stream = (StreamShmMsg*)this->shm;                  //缓冲区为共享存储段连接地址

    //获得或创建一个共享内存标识符
    semid = shmget((key_t)1122, sizeof(SemMsg), 0666|IPC_CREAT);
    if(semid == -1)        //获取或创建一个共享内存标识符失败
    {
        qCritical() << "共享内存创建失败,程序退出";
        exit(EXIT_FAILURE);
    }
    sem = shmat(semid, (void*)0, 0);        //返回共享存储段连接的实际地址
    if(sem == (void*)-1)
    {
        qCritical() << "共享存储段实际地址获取失败,程序退出";
        exit(EXIT_FAILURE);
    }
    g_shared_sem = (SemMsg*)sem;                  //缓冲区为共享存储段连接地址

    //初始化管道异常检查线程
    this->startTime = 0;
    std::thread pipelineCheckThread = std::thread([this]{
        while(true){
            if(time(NULL) - this->startTime >= 8 && this->startTime != 0){
                qCritical() << "管道无数据推送超时,异常退出";
                exit(0);
            }
            g_usleep(1000000);
        }
        this->pipelineCheckTid = -1;
    });
    this->pipelineCheckTid = pipelineCheckThread.native_handle();
    pipelineCheckThread.detach();
}

AnalysisStream::~AnalysisStream()
{
    //关闭异常检查线程
    if(this->pipelineCheckTid > 0){
        pthread_cancel(this->pipelineCheckTid);
        this->pipelineCheckTid = -1;
    }

    //将共享内存从当前进程中分离
    if(shmdt(this->shm) == -1)        //失败
    {
        qCritical() << "共享内存分离失败";
        exit(EXIT_FAILURE);
    }
    if(shmctl(this->shmid, IPC_RMID, 0) == -1)        //失败
    {
        qCritical() << "删除该结构以及相连的共享存储段标识";
        exit(EXIT_FAILURE);
    }

    //将共享内存从当前进程中分离
    if(shmdt(sem) == -1)        //失败
    {
        qCritical() << "共享内存分离失败";
        exit(EXIT_FAILURE);
    }
    if(shmctl(semid, IPC_RMID, 0) == -1)        //失败
    {
        qCritical() << "删除该结构以及相连的共享存储段标识";
        exit(EXIT_FAILURE);
    }

    static_analysisStream = NULL;
}

bool AnalysisStream::initPipeline(){
    try{
        qDebug() << "分析管道初始化中...";
        this->activateRelease = false;
        this->startTime = 0;
        //变量定义
        GstElement *appsrc = NULL, *parse = NULL, *decoder = NULL, *streammux = NULL, *nvvidconvosd = NULL, *nvosd = NULL, *nvvidconvrtmp = NULL,
                *filter = NULL, *h264enc = NULL, *sink = NULL;
        GstElement *pgie = NULL, *nvtracker = NULL;                                                             //analysis
        GstElement *transform = NULL;                                                                           //display
        GstElement  *parse1 = NULL, *flvmux = NULL;                                                             //rtmp
        GstElement *rtppay = NULL;                                                                              //udp
        GstCaps *caps = NULL;
        GstBus *bus = NULL;
        GstElement  *queue1 = NULL, *queue2 = NULL, *queue3 = NULL, *queue4 = NULL, *queue5 = NULL, *queue6 = NULL, *queue7 = NULL, *queue8 = NULL,
                *queue9 = NULL, *queue10 = NULL, *queue11 = NULL, *queue12 = NULL;
        GstPad *sinkpad, *srcpad;
        /* Standard GStreamer initialization */
        gst_init(NULL, NULL);
        loop = g_main_loop_new(NULL, FALSE);
        //element初始化
        /* Create Pipeline element that will form a connection of other elements */
        pipeline = gst_pipeline_new ("rtmp-pipeline");
        /* appsrc element for reading from the shared memory */
        appsrc = gst_element_factory_make ("appsrc", "source");
        parse = gst_element_factory_make ("h264parse", "h264-parser");
        decoder = gst_element_factory_make ("nvv4l2decoder", "nvv4l2-decoder");
        streammux = gst_element_factory_make ("nvstreammux", "stream-muxer");
        pgie = gst_element_factory_make ("nvinfer", "primary-nvinference-engine");
        nvtracker = gst_element_factory_make ("nvtracker", "tracker");
        nvvidconvosd = gst_element_factory_make ("nvvideoconvert", "nvvideo-converter");
        nvosd = gst_element_factory_make ("nvdsosd", "nv-onscreendisplay");
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
        queue11 = gst_element_factory_make ("queue", "queue11");
        queue12 = gst_element_factory_make ("queue", "queue12");
        if (!pipeline || !appsrc || !parse || !decoder || !streammux || !pgie || !nvtracker || !nvvidconvosd || !nvosd) {
            g_printerr ("one element could not be created. Exiting.\n");
            return -1;
        }
        // 给实时视频流打上时间戳
        g_object_set (appsrc, "do-timestamp", TRUE,   NULL);
        g_signal_connect(appsrc, "need-data", G_CALLBACK(start_feed), NULL);
        g_object_set (appsrc, "is-live", TRUE,   NULL);
        //g_signal_connect (appsrc, "enough-data", G_CALLBACK (stop_feed), NULL);
        g_object_set(decoder, "disable-dpb", TRUE, NULL);
        g_object_set (G_OBJECT (decoder), "enable-frame-type-reporting", true, NULL);
        g_object_set (G_OBJECT (decoder), "enable-max-performance", true, NULL);
        g_object_set (G_OBJECT (streammux), "batch-size", 1, NULL);
        g_object_set (G_OBJECT (streammux), "width", 1920, "height", 1080, "batched-push-timeout", 30000, "live-source", TRUE, "attach-sys-ts", TRUE, NULL);
        //根据analysisType设置算法
        g_object_set (G_OBJECT (pgie), "config-file-path", (gchar*)(g_application_path + "/config/personCar_pgie_config.txt").c_str(), NULL);
        /* skip frame */
        g_object_set (G_OBJECT (pgie), "interval", SKIP_FRAME);
        /* Set necessary properties of the tracker element. */
        if (!set_tracker_properties(nvtracker)) {
            g_printerr ("Failed to set tracker properties. Exiting.\n");
            return false;
        }
        g_object_set (G_OBJECT (nvvidconvosd), "nvbuf-memory-type", 0, NULL);

        gst_bin_add_many (GST_BIN (pipeline), appsrc, queue1, parse, queue2, decoder, streammux, queue3, pgie, queue4, nvtracker, queue5, nvvidconvosd, queue6, nvosd, queue7, NULL);

        if (!gst_element_link_many (appsrc, queue1, parse, queue2, decoder, NULL)) {
            g_printerr ("Elements could not be linked: 3. Exiting.\n");
            return -1;
        }

        /*link the decoder to the streammux*/
        gchar pad_name_sink[16] = "sink_0";
        gchar pad_name_src[16] = "src";
        sinkpad = gst_element_get_request_pad(streammux, pad_name_sink);
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


        if(outPut == "none"){
            sink = gst_element_factory_make ("fakesink", "nvvideo-renderer"); //nveglglessink fakesink
            if(!sink){
                g_printerr ("fakesink element could not be created. Exiting.\n");
                return -1;
            }
            g_object_set(G_OBJECT(sink), "async", true, "sync", false, "qos", false, NULL);
            gst_bin_add_many (GST_BIN (pipeline), sink, NULL);
            if (!gst_element_link_many (streammux, queue3, pgie, queue4, nvtracker, queue5, nvvidconvosd, queue6, nvosd, queue7, sink, NULL)) {
                g_printerr ("Elements could not be linked: 3. Exiting.\n");
                return -1;
            }
        }
        else if(outPut == "display"){
            transform = gst_element_factory_make ("nvegltransform", "nvegl-transform");
            sink = gst_element_factory_make ("nveglglessink", "nvvideo-renderer"); //nveglglessink fakesink
            if(!transform || !sink){
                g_printerr ("transform or nveglglessink  element could not be created. Exiting.\n");
                return -1;
            }
            g_object_set(G_OBJECT(sink), "async", true, "sync", false, "qos", false, NULL);
            gst_bin_add_many (GST_BIN (pipeline), transform, queue8, sink, NULL);

            if (!gst_element_link_many (streammux, queue3, pgie, queue4, nvtracker, queue5, nvvidconvosd, queue6, nvosd, queue7, transform, queue8, sink, NULL)) {
                g_printerr ("Elements could not be linked: 3. Exiting.\n");
                return -1;
            }
        }
        else{
            nvvidconvrtmp = gst_element_factory_make ("nvvideoconvert", "nvvideo-converter1");
            filter = gst_element_factory_make ("capsfilter", "filter");
            h264enc = gst_element_factory_make ("nvv4l2h264enc", "nvv4l2h264enc1"); 
            if(!nvvidconvrtmp || !filter || !h264enc){
                g_printerr ("one element could not be created. Exiting.\n");
                return -1;
            }
            g_object_set (G_OBJECT (nvvidconvrtmp), "nvbuf-memory-type", 0, NULL);
            caps = gst_caps_from_string ("video/x-raw(memory:NVMM), format=I420");
            g_object_set (G_OBJECT (filter), "caps", caps, NULL);
            gst_caps_unref (caps);
            g_object_set (G_OBJECT (h264enc), "preset-level", 1, NULL);         //smaller to faster
            g_object_set (G_OBJECT (h264enc), "insert-sps-pps", 1, NULL);
            g_object_set (G_OBJECT (h264enc), "bufapi-version", 1, NULL);
            g_object_set (G_OBJECT (h264enc), "profile", 0, NULL);
            if(outPut == "rtmp"){
                parse1 = gst_element_factory_make ("h264parse", "h264-parser1");
                flvmux = gst_element_factory_make ("flvmux", "flvmux1");
                sink = gst_element_factory_make("rtmpsink", "rtmpsink1");
                if(!parse1 || !flvmux || !sink){
                    g_printerr ("one element could not be created. Exiting.\n");
                    return -1;
                }
                g_object_set(G_OBJECT(flvmux), "streamable", true, NULL);
                g_object_set (G_OBJECT (sink), "location", (char *)rtmpAddress.c_str(), NULL);
                g_object_set(G_OBJECT(sink), "async", true, "sync", false, "qos", false, NULL);
                gst_bin_add_many (GST_BIN (pipeline), nvvidconvrtmp, queue8, filter, queue9, h264enc, queue10, parse1, queue11, flvmux, queue12, sink, NULL);
                if (!gst_element_link_many (streammux, queue3, pgie, queue4, nvtracker, queue5, nvvidconvosd, queue6, nvosd, queue7, nvvidconvrtmp, queue8, filter,
                                            queue9, h264enc, queue10, parse1, queue11, flvmux, queue12, sink, NULL)) {
                    g_printerr ("Elements could not be linked: 3. Exiting.\n");
                    return -1;
                }
            }
            else if(outPut == "rtsp"){
                rtppay = gst_element_factory_make ("rtph264pay", "rtp-payer");
                sink = gst_element_factory_make("udpsink", "udp-sink");
                g_object_set(G_OBJECT(sink), "sync", false, "qos", 0, NULL);
                if (!rtppay || !sink) {
                    g_printerr ("One element could not be created.%p,%p, Exiting.\n", rtppay, sink);
                    return -1;
                }
                g_object_set (G_OBJECT (sink), "host", "127.0.0.1", "port", 5400, NULL);

                gst_bin_add_many (GST_BIN (pipeline), nvvidconvrtmp, queue8, filter, queue9, h264enc, queue10, rtppay, queue11, sink, NULL);

                if (!gst_element_link_many (streammux, queue3, pgie, queue4, nvtracker, queue5, nvvidconvosd, queue6, nvosd, queue7, nvvidconvrtmp, queue8, filter,
                                            queue9, h264enc, queue10, rtppay, queue11, sink, NULL)) {
                    g_printerr ("Elements could not be linked: 3. Exiting.\n");
                    return -1;
                }
                start_rtsp_streaming (8554/*rtsp_port*/, 5400, 0);
            }
            else{
                qWarning() << "输出配置不支持,分析管道初始化失败";
                return false;
            }
        }

        /* Lets add probe to get informed of the meta data generated, we add probe to
         * the sink pad of the osd element, since by that time, the buffer would have
         * had got all the metadata. */
        GstPad *osd_sink_pad = gst_element_get_static_pad (nvosd, "sink");
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
        qDebug() << "分析管道初始化成功!";

        return true;
    }
    catch(std::exception& e){
        qWarning() << "分析管道初始化失败:" << e.what();
        return false;
    }
    catch(...){
        qWarning() << "分析管道初始化失败!";
        return false;
    }
}

gboolean AnalysisStream::bus_call (GstBus * bus, GstMessage * msg, gpointer data){
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
          static_analysisStream->startTime = time(NULL);            //防止初始化时失败
          break;
        }
        case GST_MESSAGE_STATE_CHANGED:
        {
          /* We are only interested in state-changed messages from the pipeline */
          //if (GST_MESSAGE_SRC (msg) == GST_OBJECT (data.pipeline)) {
//            GstState old_state, new_state, pending_state;
//            gst_message_parse_state_changed (msg, &old_state, &new_state, &pending_state);
            //qWarning() << GST_MESSAGE_SRC (msg) << " state changed from" << gst_element_state_get_name (old_state) << " to " << gst_element_state_get_name (new_state);
//            g_print ("Pipeline state changed from %s to %s:\n",
//                gst_element_state_get_name (old_state), gst_element_state_get_name (new_state));
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

gboolean AnalysisStream::set_tracker_properties (GstElement *nvtracker){
    string TRACKER_CONFIG_FILE = g_application_path + "/config/tracker_config.txt";
    gboolean ret = FALSE;
    GError *error = NULL;
    gchar **keys = NULL;
    gchar **key = NULL;
    GKeyFile *key_file = g_key_file_new ();

    if (!g_key_file_load_from_file (key_file, (gchar*)TRACKER_CONFIG_FILE.c_str(), G_KEY_FILE_NONE,
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
        char* ll_config_file = get_absolute_file_path ((gchar*)TRACKER_CONFIG_FILE.c_str(),
                  g_key_file_get_string (key_file,
                      CONFIG_GROUP_TRACKER,
                      CONFIG_GROUP_TRACKER_LL_CONFIG_FILE, &error));
        CHECK_ERROR (error);
        g_object_set (G_OBJECT (nvtracker), "ll-config-file", ll_config_file, NULL);
      } else if (!g_strcmp0 (*key, CONFIG_GROUP_TRACKER_LL_LIB_FILE)) {
        char* ll_lib_file = get_absolute_file_path ((gchar*)TRACKER_CONFIG_FILE.c_str(),
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

GstPadProbeReturn AnalysisStream::osd_sink_pad_buffer_probe (GstPad * pad, GstPadProbeInfo * info, gpointer u_data)
{
    static_analysisStream->startTime = time(NULL);
    GstBuffer *buf = (GstBuffer *) info->data;
    if(buf == NULL)
    {
        return GST_PAD_PROBE_DROP;
    }

    NvDsMetaList * l_frame = NULL;
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

    for (l_frame = batch_meta->frame_meta_list; l_frame != NULL;
      l_frame = l_frame->next) {
        NvDsFrameMeta *frame_meta = (NvDsFrameMeta *) (l_frame->data);

        //osd时间显示
        int offset = 0;
        display_meta = nvds_acquire_display_meta_from_pool(batch_meta);
        NvOSD_TextParams *txt_params  = &display_meta->text_params[0];
        display_meta->num_labels = 1;
        txt_params->display_text =  (char*)g_malloc0 (MAX_DISPLAY_LEN);
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

    gst_buffer_unmap (buf, &in_map_info);
    return GST_PAD_PROBE_OK;
}

/* This signal callback triggers when appsrc needs data. Here,
 * we add an idle handler to the mainloop to start pushing
 * data into the appsrc */
void AnalysisStream::start_feed (GstElement * source, guint size, gpointer user_data)
{
    cb_need_data(source);
}

/* This callback triggers when appsrc has enough data and we can stop sending.
 * We remove the idle handler from the mainloop */
void AnalysisStream::stop_feed (GstElement * source, gpointer user_data)
{
}

/*add data to the source*/
void AnalysisStream::cb_need_data(GstElement *appsrc){
    /*
     * send h264 to the pipeline, not success, need discuss
    */
    static GstClockTime timestamp = 0;
    GstFlowReturn ret;
    GstMapInfo map;

    //共享内存读取数据
    if(sem_wait(&(g_shared_sem->streamSem)) == -1)        //sem_wait为P操作，减少信号量的值
    {
        cout << "信号量P操作 ERROR!" << endl;
        return;
    }
    while(g_shared_stream->analysisUse == 1){
//        cout << "无新数据，信号量V操作!" << endl;
        sem_post(&g_shared_sem->streamSem);                           //V 操作增加信号量
        g_usleep(5000);                                          //等待30ms
        if(sem_wait(&(g_shared_sem->streamSem)) == -1)                //sem_wait为P操作，减少信号量的值
        {
            cout << "信号量P操作 ERROR!" << endl;
            return;
        }
    }
    // 往appsrc输入数据
    cout << "len:" << g_shared_stream->len << "   analysisUse:" << g_shared_stream->analysisUse << "   time:"  << time(NULL) << endl;
    GstBuffer *buf = gst_buffer_new_allocate(NULL, g_shared_stream->len, NULL);
    gst_buffer_map(buf, &map, GST_MAP_WRITE);
    memcpy((guchar *)map.data, (guchar *)(g_shared_stream->data), g_shared_stream->len);
    map.size = g_shared_stream->len;
    g_shared_stream->analysisUse = 1;
    sem_post(&g_shared_sem->streamSem);                           //V 操作增加信号量

    //更新数据至管道
    GST_BUFFER_PTS(buf) = timestamp;
    GST_BUFFER_DURATION(buf) = gst_util_uint64_scale_int(1, GST_SECOND, 1);
    timestamp += GST_BUFFER_DURATION(buf);
    g_signal_emit_by_name(appsrc, "push-buffer", buf, &ret);
    //ret = gst_app_src_push_buffer(appsrc, buf);
    if (ret != GST_FLOW_OK) {
      g_print ("gst_app_src_push_buffer returned %d \n", ret);
      static_analysisStream->stopAnalysis();
    }
    gst_buffer_unmap (buf, &map);
    gst_buffer_unref(buf);
}

void AnalysisStream::startAnalysis(){
    gst_element_set_state (pipeline, GST_STATE_PLAYING);
    qDebug() << "分析管道运行中...";
    g_main_loop_run(loop);
    qDebug() << "分析管道释放中...";

    gst_element_set_state(pipeline, GST_STATE_NULL);
    if(outPut == "rtsp"){
        destroy_sink_bin();
    }
    qDebug() << "GST_STATE_NULL...";
    g_usleep(5000000);                     //must wait for some time
    gst_object_unref(GST_OBJECT(pipeline));
    g_source_remove(bus_watch_id);
    g_main_loop_unref(loop);
    qDebug() << "分析管道已退出,程序退出";
    exit(0);
}

void AnalysisStream::pauseAnalysis(){
    gst_element_set_state(pipeline, GST_STATE_PAUSED);
}

void AnalysisStream::resumeAnalysis(){
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
}

void AnalysisStream::stopAnalysis(){
    this->activateRelease = true;
    g_main_loop_quit (loop);
}

void AnalysisStream::emitSignal(DetectionInfo detection){
    Q_EMIT sendResult(detection);
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

gboolean AnalysisStream::timeout (GstRTSPServer * server)
{
  GstRTSPSessionPool *pool;

  pool = gst_rtsp_server_get_session_pool (server);
  gst_rtsp_session_pool_cleanup (pool);
  g_object_unref (pool);

  return TRUE;
}
