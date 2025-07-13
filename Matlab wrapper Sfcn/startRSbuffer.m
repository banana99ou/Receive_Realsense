function startRSbuffer(imgW, imgH, reqFps)
% STARTRSBUFFER  Launch background RS capture + reshape into globals
%
%   startRSbuffer(width, height, reqFps)
%
% Globals created:
%   RS_BUFFER  (HxWx3 uint8)
%   RS_ACCEL   (3x1 single)
%   RS_GYRO    (3x1 single)
%   RS_TIMER   (MATLAB timer handle)
%   RS_PIPE    (realsense.pipeline)

    global RS_BUFFER RS_ACCEL RS_GYRO RS_TIMER RS_PIPE

    % 1) Pre-allocate
    RS_BUFFER = zeros(imgH, imgW, 3, 'uint8');
    RS_ACCEL  = single([0;0;0]);
    RS_GYRO   = single([0;0;0]);

    % 2) Open the camera once
    RS_PIPE = realsense.pipeline();
    cfg     = realsense.config();
    cfg.enable_stream(realsense.stream.color, imgW, imgH, realsense.format.rgb8, reqFps);
    cfg.enable_stream(realsense.stream.accel);
    cfg.enable_stream(realsense.stream.gyro);
    RS_PIPE.start(cfg);

    % 3) Start a fixed-rate timer at reqFps
    period = 1/reqFps;
    RS_TIMER = timer( ...
        'ExecutionMode','fixedRate', ...
        'Period',      period, ...
        'BusyMode',    'drop', ...        % don’t queue up if late
        'TimerFcn',    @bufferFetch ...
    );
    start(RS_TIMER);
end

%--------------------------------------------------------------%
function bufferFetch(~,~)
    % Called by RS_TIMER at ~30 Hz
    global RS_BUFFER RS_ACCEL RS_GYRO RS_PIPE

    fs = RS_PIPE.wait_for_frames();             % blocks ~1/30 s
    % 1) Colour frame → H×W×3 uint8
    cfrm = fs.get_color_frame();
    data = cfrm.get_data();                     % 1×(3WH)
    img  = permute(reshape(data',[3, size(RS_BUFFER,2), size(RS_BUFFER,1)]), [3 2 1]);
    RS_BUFFER = img;

    % 2) IMU
    afrm = fs.first(realsense.stream.accel).as('motion_frame');
    gfrm = fs.first(realsense.stream.gyro ).as('motion_frame');
    RS_ACCEL = single(afrm.get_motion_data());
    RS_GYRO  = single(gfrm.get_motion_data());

    delete(fs);
end
