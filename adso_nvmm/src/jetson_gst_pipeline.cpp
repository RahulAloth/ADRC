#include <gst/gst.h>
#include <glib.h>
#include <cstring>
#include <iostream>

/* --------------------------------------------------------------------------
 *  Function Declarations
 * -------------------------------------------------------------------------- */

static gboolean bus_call(GstBus* bus, GstMessage* msg, gpointer data);
const char* select_pipeline(const char* mode);
int run_pipeline(const char* pipeline_desc);

/* --------------------------------------------------------------------------
 *  Bus Callback: Handles asynchronous messages from the pipeline
 * -------------------------------------------------------------------------- */
static gboolean bus_call(GstBus* /*bus*/, GstMessage* msg, gpointer data)
{
    auto* loop = static_cast<GMainLoop*>(data);

    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_EOS:
            g_print("End of stream\n");
            g_main_loop_quit(loop);
            break;

        case GST_MESSAGE_ERROR: {
            GError* err = nullptr;
            gchar* debug = nullptr;

            gst_message_parse_error(msg, &err, &debug);
            g_printerr("Error: %s\n", err->message);

            if (debug) {
                g_printerr("Debug details: %s\n", debug);
                g_free(debug);
            }

            g_error_free(err);
            g_main_loop_quit(loop);
            break;
        }

        default:
            break;
    }

    return TRUE;
}

/* --------------------------------------------------------------------------
 *  Pipeline Selector: Returns the correct pipeline string based on mode
 * -------------------------------------------------------------------------- */
const char* select_pipeline(const char* mode)
{
    if (strcmp(mode, "--nvmm") == 0) {
        g_print("Running NVMM pipeline...\n");
        return
            "v4l2src device=/dev/video0 ! "
            "image/jpeg, width=1280, height=720, framerate=30/1 ! "
            "jpegdec ! "
            "nvvidconv ! "
            "video/x-raw(memory:NVMM), format=NV12 ! "
            "nveglglessink";
    }

    if (strcmp(mode, "--cuda_acc") == 0) {
        g_print("Running CUDA-accelerated pipeline...\n");
        return
            "v4l2src device=/dev/video0 ! "
            "image/jpeg, width=1280, height=720, framerate=30/1 ! "
            "nvv4l2decoder mjpeg=1 ! "
            "nvvidconv ! "
            "video/x-raw(memory:NVMM), format=NV12 ! "
            "nveglglessink";
    }

    if (strcmp(mode, "--cuda_process") == 0) {
        g_print("Running NVMM + NVDEC + CUDA processing pipeline...\n");
        return
            "v4l2src device=/dev/video0 ! "
            "image/jpeg, width=1280, height=720, framerate=30/1 ! "
            "nvv4l2decoder mjpeg=1 ! "
            "nvvidconv ! "
            "video/x-raw(memory:NVMM), format=NV12 ! "
            "nvvideoconvert nvbuf-memory-type=0 cuda-process=true ! "
            "video/x-raw(memory:NVMM), format=NV12 ! "
            "nveglglessink";
    }

    if (strcmp(mode, "--infer") == 0) {
        g_print("Running DeepStream TensorRT inference pipeline...\n");
        return
            "v4l2src device=/dev/video0 ! "
            "image/jpeg, width=1280, height=720, framerate=30/1 ! "
            "nvv4l2decoder mjpeg=1 ! "
            "nvvidconv ! "
            "video/x-raw(memory:NVMM), format=NV12 ! "
            "mux.sink_0 "
            "nvstreammux name=mux batch-size=1 width=1280 height=720 live-source=1 ! "
            "nvvideoconvert ! "
            "video/x-raw(memory:NVMM), format=NV12 ! "
            "nvinfer config-file-path=/home/rahul/adso-nvmm/build/config_infer_primary.txt ! "
            "nvdsosd ! "
            "nveglglessink";
    }

    if (strcmp(mode, "--headless") == 0) {
        g_print("Running headless DeepStream inference pipeline...\n");
        return
            "v4l2src device=/dev/video0 ! "
            "image/jpeg, width=1280, height=720, framerate=30/1 ! "
            "nvv4l2decoder mjpeg=1 ! "
            "nvvidconv ! "
            "video/x-raw(memory:NVMM), format=NV12 ! "
            "mux.sink_0 "
            "nvstreammux name=mux batch-size=1 width=1280 height=720 live-source=1 ! "
            "nvvideoconvert ! "
            "video/x-raw(memory:NVMM), format=NV12 ! "
            "nvinfer config-file-path=peoplenet_config.txt ! "
            "fakesink sync=false";
    }

    return nullptr;
}

/* --------------------------------------------------------------------------
 *  Pipeline Runner: Creates, runs, and cleans up the GStreamer pipeline
 * -------------------------------------------------------------------------- */
int run_pipeline(const char* pipeline_desc)
{
    if (!pipeline_desc) {
        g_printerr("Invalid pipeline description\n");
        return -1;
    }

    GError* error = nullptr;
    GstElement* pipeline = gst_parse_launch(pipeline_desc, &error);

    if (!pipeline) {
        g_printerr("Failed to create pipeline: %s\n",
                   error ? error->message : "unknown error");
        if (error) g_error_free(error);
        return -1;
    }

    GMainLoop* loop = g_main_loop_new(nullptr, FALSE);

    GstBus* bus = gst_element_get_bus(pipeline);
    gst_bus_add_watch(bus, bus_call, loop);
    gst_object_unref(bus);

    if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        g_printerr("Failed to set pipeline to PLAYING\n");
        gst_object_unref(pipeline);
        g_main_loop_unref(loop);
        return -1;
    }

    g_main_loop_run(loop);

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    g_main_loop_unref(loop);

    return 0;
}

/* --------------------------------------------------------------------------
 *  Main Entry Point
 * -------------------------------------------------------------------------- */
int main(int argc, char* argv[])
{
    gst_init(&argc, &argv);

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0]
                  << " [--nvmm | --cuda_acc | --cuda_process | --infer | --headless]\n";
        return -1;
    }

    const char* pipeline_desc = select_pipeline(argv[1]);
    if (!pipeline_desc) {
        std::cerr << "Unknown mode: " << argv[1] << "\n";
        return -1;
    }

    return run_pipeline(pipeline_desc);
}