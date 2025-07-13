#include <librealsense2/rs.hpp>
#include <iostream>

int main() {
    // Declare pipeline and configuration
    rs2::pipeline pipe;
    rs2::config cfg;

    // 1. Enable RGB stream at 640×480 @ 60 Hz
    //    (default format for color is BGR8)           :contentReference[oaicite:0]{index=0}
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);

    // 2. Enable IMU streams: accelerometer @ 250 Hz, gyroscope @ 200 Hz
    //    (common pairing for D435i)                   :contentReference[oaicite:1]{index=1}
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 250);
    cfg.enable_stream(RS2_STREAM_GYRO,  RS2_FORMAT_MOTION_XYZ32F, 200);

    // Start streaming with our configuration
    rs2::pipeline_profile profile = pipe.start(cfg);

    while (true) {
        // Block until a new set of frames arrives
        rs2::frameset frames = pipe.wait_for_frames();

        // 3a. Retrieve the color frame               :contentReference[oaicite:2]{index=2}
        rs2::video_frame color = frames.get_color_frame();

        // 3b. Retrieve IMU frames (first matching frame for each stream)
        //     and cast them to motion_frame            :contentReference[oaicite:3]{index=3}
        auto accel_frame = frames.first(RS2_STREAM_ACCEL)
                                .as<rs2::motion_frame>();
        auto gyro_frame  = frames.first(RS2_STREAM_GYRO)
                                .as<rs2::motion_frame>();

        // Access raw data pointers if needed:
        const uint8_t* color_data = 
            reinterpret_cast<const uint8_t*>(color.get_data());
        // color_data now points to BGR bytes (640×480×3)

        // 3c. Get IMU vectors
        rs2_vector accel = accel_frame.get_motion_data();
        rs2_vector gyro  = gyro_frame.get_motion_data();

        // Print a simple summary
        std::cout << "Color frame @ "
                  << color.get_timestamp() << " ms; "
                  << "Accel [m/s²]=(" 
                  << accel.x << ", " << accel.y << ", " << accel.z << ")  "
                  << "Gyro [°/s]=(" 
                  << gyro.x  << ", " << gyro.y  << ", " << gyro.z  << ")\n";
    }

    return 0;
}