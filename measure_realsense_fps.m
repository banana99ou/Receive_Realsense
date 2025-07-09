ctx  = realsense.context();
pipe = realsense.pipeline();
cfg  = realsense.config();
% match the same resolution and fps you’re using in Simulink
cfg.enable_stream(realsense.stream.color, 640, 480, realsense.format.rgb8, 60);
cfg.enable_stream(realsense.stream.accel);
cfg.enable_stream(realsense.stream.gyro);
pipe.start(cfg);

count   = 0;
t0      = tic;
DUR_SEC = 10;                    % measure over 10 seconds
while toc(t0) < DUR_SEC
    if pipe.poll_for_frames()
        count = count + 1;
    end
end
elapsed = toc(t0);
fps_meas = count/elapsed;
fprintf('Measured color-stream FPS: %.2f (requested 30)\n', fps_meas);

pipe.stop();
delete(pipe);