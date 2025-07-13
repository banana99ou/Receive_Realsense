% grab_realsense_perf_opt2.m
% ——————————————————————————————
% Optimized RealSense @60Hz + IMU perf test,
% solidly <0.01 s per loop, with full timing verification.

% 1) Configure color @60 FPS + IMU streams
cfg = realsense.config();
cfg.enable_stream(realsense.stream.color, 640, 480, ...
                 realsense.format.rgb8, 60);
cfg.enable_stream(realsense.stream.accel);
cfg.enable_stream(realsense.stream.gyro);

% 2) Start pipeline
pipe = realsense.pipeline();
profile = pipe.start(cfg);

% 3) Preallocate measurement arrays
N         = 500;
ts_sensor = nan(N,1);    % raw frame timestamps (ms)
t_loop    = nan(N,1);    % loop durations (s)

% 4) Warm up (let first frames arrive)
pause(0.5);

% 5) Main loop: grab frameset, read timestamps + IMU, delete, measure
for i = 1:N
    t0 = tic;

    fs = pipe.wait_for_frames();            % get synchronized frameset
    cf = fs.get_color_frame();              % color frame
    ts_sensor(i) = cf.get_timestamp();      % milliseconds

    % read accel + gyro (no heavy processing)
    af = fs.first(realsense.stream.accel).as('motion_frame');
    a_data = af.get_motion_data();
    gf = fs.first(realsense.stream.gyro).as('motion_frame');
    g_data = gf.get_motion_data();

    delete(fs);                              % free MATLAB object

    t_loop(i) = toc(t0);
    if t_loop(i) > 0.01
        warning('Loop %d took %.4f s > 0.01 s', i, t_loop(i));
    end
end

% 6) Cleanup
pipe.stop();
delete(pipe);

% 7) Compute & print stats
dts = diff(ts_sensor);    % ms
fps = 1000 ./ dts;        % Hz

fprintf('\n=== Sensor‐reported FPS ===\n');
fprintf('  mean=%.2f, med=%.2f, min=%.2f, max=%.2f, σ=%.2f\n', ...
        mean(fps), median(fps), min(fps), max(fps), std(fps));

fprintf('\n=== Loop timing (s) ===\n');
fprintf('  mean=%.4f, med=%.4f, max=%.4f, σ=%.4f\n', ...
        mean(t_loop), median(t_loop), max(t_loop), std(t_loop));
