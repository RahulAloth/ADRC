#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/allocators/gstdmabuf.h>
#include <vpi/OpenCVInterop.hpp>
#include <vpi/algo/StereoDisparity.h>
#include <vpi/VPI.h>

#include <nvbufsurface.h>
#include <nvbufsurftransform.h>

#include <vpi/VPI.h>
#include <vpi/Image.h>

#include <opencv2/opencv.hpp>
#include <iostream>

#include <iostream>
#include <NvBuffer.h>
#include <vpi/OpenCVInterop.hpp>
#include <vpi/OpenCVInterop.hpp>
#include <vpi/algo/StereoDisparity.h>
#include <vpi/VPI.h>

#include <opencv2/opencv.hpp>
#include <iostream>

#include <vpi/VPI.h>
#include <vpi/algo/StereoDisparity.h>
#include <vpi/Image.h>

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <iostream>
#include <gst/gst.h>
#include <gst/video/video.h>
#include <gst/app/gstappsink.h>

#include <vpi/VPI.h>
#include <vpi/Image.h>
#include <vpi/algo/StereoDisparity.h>

#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <iostream>


static const int FRAME_WIDTH  = 1280;
static const int FRAME_HEIGHT = 720;
static int g_nvbuf_fd_right = -1;

static NvBufSurface *g_right_surface = nullptr;
static VPIStream stream;
// -------------------------
// Shared stereo context
// -------------------------
struct StereoContext {
    cv::Mat left;
    cv::Mat right;

    GstClockTime ts_left  = 0;
    GstClockTime ts_right = 0;
};

static StereoContext user_context;

static bool init_nvbuffer_right()
{
    // Create VPI stream
    
    if (vpiStreamCreate(0, &stream) != VPI_SUCCESS) {
        std::cerr << "Failed to create VPI stream\n";
        return false;
    }
    return true;
}

static VPIPayload stereo = nullptr;
static VPIStatus st;
static VPIStereoDisparityEstimatorParams runParams;
static VPIImage leftVPI, rightVPI, disparityVPI;
static VPIPayload payload;
static VPIStereoDisparityEstimatorParams params;
bool disparity_thread(StereoContext *ctx)
{
    static bool initialized = true;
    int width  = ctx->left.cols;
    int height = ctx->left.rows;
    int width_right = ctx->right.cols;
    if( 0x00 >= width || 0x00 >= height || 0x00 >= width_right )
    {
        std::cerr << "not ready yet\n";
        return false;
    }
    if (initialized) {
        if (vpiStreamCreate(0, &stream) != VPI_SUCCESS) {
            std::cerr << "Failed to create VPI stream\n";
            return false;
        }
        initialized = false;
    }

    // Wrap OpenCV images into VPIImage
    VPIImage leftVPI, rightVPI, disparityVPI;

    if (vpiImageCreateWrapperOpenCVMat(ctx->left,  VPI_IMAGE_FORMAT_U8, 0, &leftVPI) != VPI_SUCCESS ||
        vpiImageCreateWrapperOpenCVMat(ctx->right, VPI_IMAGE_FORMAT_U8, 0, &rightVPI) != VPI_SUCCESS) {
        std::cerr << "Failed to wrap OpenCV images into VPI\n";
        vpiStreamDestroy(stream);
        return false;
    }

    int32_t vpi_width, vpi_height;
    vpiImageGetSize(rightVPI, &vpi_width, &vpi_height);
    std::cout << "Image Size right VPI - Width: " << vpi_width << ", Height: " << vpi_height << "\n";

    vpiImageGetSize(leftVPI, &vpi_width, &vpi_height);
    std::cout << "Image Size left VPI - Width: " << vpi_width << ", Height: " << vpi_height << "\n";

    std::cout << "Images wrapped into VPI\n";


    static bool disparity_initialized = true;
    if (disparity_initialized) {
        // Create disparity output image (S16, Q10.5)
        if (vpiImageCreate(width, height, VPI_IMAGE_FORMAT_S16, 0, &disparityVPI) != VPI_SUCCESS) {
            std::cerr << "Failed to create disparity VPI image\n";
            vpiImageDestroy(leftVPI);
            vpiImageDestroy(rightVPI);
            vpiStreamDestroy(stream);
            return false;
        }

        // Disparity estimator payload creation
        VPIStereoDisparityEstimatorCreationParams createParams;
        vpiInitStereoDisparityEstimatorCreationParams(&createParams);

        createParams.maxDisparity     = 64;
        createParams.downscaleFactor  = 1;
        createParams.includeDiagonals = 1;


        if (vpiCreateStereoDisparityEstimator(
                VPI_BACKEND_CUDA,
                width, height,
                VPI_IMAGE_FORMAT_U8,
                &createParams,
                &payload) != VPI_SUCCESS) {
            std::cerr << "Failed to create stereo disparity estimator payload\n";
            vpiImageDestroy(leftVPI);
            vpiImageDestroy(rightVPI);
            vpiImageDestroy(disparityVPI);
            vpiStreamDestroy(stream);
            return false;
        }

        // Set runtime parameters
        vpiInitStereoDisparityEstimatorParams(&params);

        params.maxDisparity        = 0;   // use payload value
        params.confidenceThreshold = 0;
        params.minDisparity        = 0;
        params.p1                  = 3;
        params.p2                  = 48;
        params.uniqueness          = -1;  // disable uniqueness
        disparity_initialized = false;
    }
    if (vpiSubmitStereoDisparityEstimator(
            stream,
            VPI_BACKEND_CUDA,
            payload,
            leftVPI,
            rightVPI,
            disparityVPI,
            nullptr,     // no confidence map
            &params) != VPI_SUCCESS) {
        std::cerr << "Failed to submit stereo disparity estimator\n";
        vpiPayloadDestroy(payload);
        vpiImageDestroy(leftVPI);
        vpiImageDestroy(rightVPI);
        vpiImageDestroy(disparityVPI);
        vpiStreamDestroy(stream);
        return false;
    }

    vpiStreamSync(stream);

    // Read back disparity result
    VPIImageData dispData;
    st = vpiImageLockData(
        disparityVPI,
        VPI_LOCK_READ,
        VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR,
        &dispData
    );
    if (st != VPI_SUCCESS) {
        std::cerr << "Failed to lock disparity image\n";
        vpiPayloadDestroy(payload);
        vpiImageDestroy(leftVPI);
        vpiImageDestroy(rightVPI);
        vpiImageDestroy(disparityVPI);
        vpiStreamDestroy(stream);
        return false;
    }

    auto &plane = dispData.buffer.pitch.planes[0];

    cv::Mat disp16(
        plane.height,
        plane.width,
        CV_16SC1,
        plane.data,
        plane.pitchBytes
    );

    // Convert Q10.5 → float
    cv::Mat dispFloat;
    disp16.convertTo(dispFloat, CV_32F, 1.0 / 32.0);

    cv::imwrite("disparity.png", dispFloat);
    vpiImageUnlock(disparityVPI);

    // Cleanup
    vpiPayloadDestroy(payload);
    vpiImageDestroy(leftVPI);
    vpiImageDestroy(rightVPI);
    vpiImageDestroy(disparityVPI);
    vpiStreamDestroy(stream);

    std::cout << "Disparity saved to disparity.png\n";
    return true;
}

static bool sample_to_gray(GstSample *sample, cv::Mat &gray_out)
{
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (!buffer)
        return false;

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ))
        return false;

    GstCaps *caps = gst_sample_get_caps(sample);
    if (!caps) {
        gst_buffer_unmap(buffer, &map);
        return false;
    }

    GstStructure *s = gst_caps_get_structure(caps, 0);

    int width = 0, height = 0;
    gst_structure_get_int(s, "width", &width);
    gst_structure_get_int(s, "height", &height);

    const gchar *format = gst_structure_get_string(s, "format");

    GstVideoMeta *vmeta = gst_buffer_get_video_meta(buffer);
    int stride = 0;

    if (vmeta) {
        stride = vmeta->stride[0];
    } else {
        stride = width * 2; // assume YUY2 tightly packed
    }

    cv::Mat gray;

    if (g_strcmp0(format, "YUY2") == 0) {
        cv::Mat yuy2(height, width, CV_8UC2, map.data, stride);
        cv::cvtColor(yuy2, gray, cv::COLOR_YUV2GRAY_YUY2);
    } else if (g_strcmp0(format, "GRAY8") == 0) {
        gray = cv::Mat(height, width, CV_8UC1, map.data, stride).clone();
    } else {
        g_printerr("Unsupported format: %s\n", format);
        gst_buffer_unmap(buffer, &map);
        return false;
    }

    gst_buffer_unmap(buffer, &map);
    gray_out = gray;
    return true;
}


static GstFlowReturn on_new_sample_left(GstAppSink *appsink, gpointer user_data)
{
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample)
        return GST_FLOW_ERROR;

    cv::Mat gray;
    if (!sample_to_gray(sample, gray)) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstClockTime ts   = GST_BUFFER_PTS(buffer);
    user_context.left    = gray;
    user_context.ts_left = ts;
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

static GstFlowReturn on_new_sample_right(GstAppSink *appsink, gpointer user_data)
{
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample)
        return GST_FLOW_ERROR;
    cv::Mat gray;
    if (!sample_to_gray(sample, gray)) {
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
    }

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstClockTime ts   = GST_BUFFER_PTS(buffer);
    user_context.right    = gray;
    user_context.ts_right = ts;
    disparity_thread(&user_context);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
// New Code End
    return GST_FLOW_OK;
}

static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer data)
{
    GMainLoop *loop = (GMainLoop *)data;

    switch (GST_MESSAGE_TYPE(msg))
    {
        case GST_MESSAGE_EOS:
            g_print("End of stream\n");
            g_main_loop_quit(loop);
            break;

        case GST_MESSAGE_ERROR:
        {
            gchar *debug;
            GError *error;
            gst_message_parse_error(msg, &error, &debug);
            g_printerr("Error: %s\n", error->message);
            g_error_free(error);
            g_free(debug);
            g_main_loop_quit(loop);
            break;
        }

        default:
            break;
    }
    return TRUE;
}

int main(int argc, char *argv[])
{
    gst_init(&argc, &argv);
    GMainLoop *loop = g_main_loop_new(NULL, FALSE);
    // init_nvbuffer_right();
    // LEFT pipeline
    GstElement *pipelineL = gst_pipeline_new("pipeline-left");

    GstElement *srcL      = gst_element_factory_make("v4l2src", "srcL");
    GstElement *convL     = gst_element_factory_make("nvvidconv", "convL");
    GstElement *teeL      = gst_element_factory_make("tee", "teeL");

    GstElement *qDispL    = gst_element_factory_make("queue", "qDispL");
    GstElement *dispConvL = gst_element_factory_make("nvvidconv", "dispConvL");
    GstElement *dispSinkL = gst_element_factory_make("autovideosink", "dispSinkL");

    GstElement *qNvmmL    = gst_element_factory_make("queue", "qNvmmL");
    GstElement *appsinkL  = gst_element_factory_make("appsink", "appsinkL");

    if (!pipelineL || !srcL || !convL || !teeL ||
        !qDispL || !dispConvL || !dispSinkL ||
        !qNvmmL || !appsinkL)
    {
        g_printerr("Failed to create left pipeline elements\n");
        return -1;
    }

    g_object_set(srcL, "device", "/dev/video0", NULL);
    g_object_set(appsinkL, "sync", FALSE, "max-buffers", 1, "drop", TRUE, NULL);

    gst_bin_add_many(GST_BIN(pipelineL),
                     srcL, convL, teeL,
                     qDispL, dispConvL, dispSinkL,
                     qNvmmL, appsinkL,
                     NULL);

    if (!gst_element_link(srcL, convL) ||
        !gst_element_link(convL, teeL) ||
        !gst_element_link_many(teeL, qDispL, dispConvL, dispSinkL, NULL) ||
        !gst_element_link_many(teeL, qNvmmL, appsinkL, NULL))
    {
        g_printerr("Left: failed to link pipeline\n");
        return -1;
    }

    // RIGHT pipeline
    GstElement *pipelineR = gst_pipeline_new("pipeline-right");

    GstElement *srcR      = gst_element_factory_make("v4l2src", "srcR");
    GstElement *convR     = gst_element_factory_make("nvvidconv", "convR");
    GstElement *teeR      = gst_element_factory_make("tee", "teeR");

    GstElement *qDispR    = gst_element_factory_make("queue", "qDispR");
    GstElement *dispConvR = gst_element_factory_make("nvvidconv", "dispConvR");
    GstElement *dispSinkR = gst_element_factory_make("autovideosink", "dispSinkR");

    GstElement *qNvmmR    = gst_element_factory_make("queue", "qNvmmR");
    GstElement *appsinkR  = gst_element_factory_make("appsink", "appsinkR");

    if (!pipelineR || !srcR || !convR || !teeR ||
        !qDispR || !dispConvR || !dispSinkR ||
        !qNvmmR || !appsinkR)
    {
        g_printerr("Failed to create right pipeline elements\n");
        return -1;
    }

    g_object_set(srcR, "device", "/dev/video2", NULL);
    g_object_set(appsinkR, "sync", FALSE, "max-buffers", 1, "drop", TRUE, NULL);

    gst_bin_add_many(GST_BIN(pipelineR),
                     srcR, convR, teeR,
                     qDispR, dispConvR, dispSinkR,
                     qNvmmR, appsinkR,
                     NULL);

    if (!gst_element_link(srcR, convR) ||
        !gst_element_link(convR, teeR) ||
        !gst_element_link_many(teeR, qDispR, dispConvR, dispSinkR, NULL) ||
        !gst_element_link_many(teeR, qNvmmR, appsinkR, NULL))
    {
        g_printerr("Right: failed to link pipeline\n");
        return -1;
    }

    // Callbacks
    GstAppSinkCallbacks callbacksL = {};
    callbacksL.new_sample = on_new_sample_left;
    gst_app_sink_set_callbacks(GST_APP_SINK(appsinkL), &callbacksL, NULL, NULL);

    GstAppSinkCallbacks callbacksR = {};
    callbacksR.new_sample = on_new_sample_right;
    gst_app_sink_set_callbacks(GST_APP_SINK(appsinkR), &callbacksR, NULL, NULL);

    // Bus
    GstBus *busL = gst_pipeline_get_bus(GST_PIPELINE(pipelineL));
    gst_bus_add_watch(busL, bus_call, loop);
    gst_object_unref(busL);

    GstBus *busR = gst_pipeline_get_bus(GST_PIPELINE(pipelineR));
    gst_bus_add_watch(busR, bus_call, loop);
    gst_object_unref(busR);

    gst_element_set_state(pipelineL, GST_STATE_PLAYING);
    gst_element_set_state(pipelineR, GST_STATE_PLAYING);

    g_print("Running two-camera pipelines...\n");
    g_main_loop_run(loop);

    gst_element_set_state(pipelineL, GST_STATE_NULL);
    gst_element_set_state(pipelineR, GST_STATE_NULL);

    gst_object_unref(pipelineL);
    gst_object_unref(pipelineR);
    g_main_loop_unref(loop);

    return 0;
}
