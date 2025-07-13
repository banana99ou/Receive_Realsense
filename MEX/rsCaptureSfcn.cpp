#define S_FUNCTION_NAME  rsCaptureSfcn
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <librealsense2/rs.hpp>
#include <thread>
#include <mutex>
#include <atomic>

// —————— Shared data ——————
static rs2::pipeline            pipe;
static rs2::frameset            latestFs;
static std::mutex               fsMutex;
static std::atomic<bool>        haveFrame(false);
static std::atomic<bool>        keepRunning(false);
static std::thread              captureThread;

// —————— Background thread fn ——————
static void captureLoop()
{
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 60);
    cfg.enable_stream(RS2_STREAM_ACCEL);
    cfg.enable_stream(RS2_STREAM_GYRO);
    pipe.start(cfg);

    while (keepRunning.load()) {
        auto fs = pipe.wait_for_frames();       // blocking call
        {
            std::lock_guard<std::mutex> lk(fsMutex);
            latestFs = fs;
            haveFrame.store(true);
        }
    }

    pipe.stop();
}

// —————— S-Function methods ——————
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;

    ssSetNumContStates  (S, 0);
    ssSetNumDiscStates  (S, 0);
    ssSetNumInputPorts  (S, 0);

    // 1 output port: timestamp (1×1 double)
    ssSetNumOutputPorts (S, 1);
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);

    ssSetNumSampleTimes (S, 1);
    ssSetNumRWork       (S, 0);
    ssSetNumIWork       (S, 0);
    ssSetNumPWork       (S, 0);
    ssSetNumModes       (S, 0);
    ssSetNumNonsampledZCs(S, 0);
    ssSetOptions        (S, SS_OPTION_EXCEPTION_FREE_CODE);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    // fixed-step of 0.01 s
    ssSetSampleTime(S, 0, 0.01);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
    keepRunning.store(true);
    captureThread = std::thread(captureLoop);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T  *y0 = ssGetOutputPortRealSignal(S, 0);
    double   ts = 0.0;

    if (haveFrame.load()) {
        std::lock_guard<std::mutex> lk(fsMutex);
        auto cf = latestFs.get_color_frame();
        ts = cf.get_timestamp();      // ms
        haveFrame.store(false);
    }
    y0[0] = ts;
}

static void mdlTerminate(SimStruct *S)
{
    keepRunning.store(false);
    if (captureThread.joinable())
        captureThread.join();
}

#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif