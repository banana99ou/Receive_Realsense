function realsense_sfun(block)
% REALSENSE_SFUN  Live Intel RealSense source for Simulink (simulation only)
%
%   ▸ Out1 – Colour-RGB image, uint8 column-vector (imgW*imgH*3 × 1)
%   ▸ Out2 – Accel XYZ, single [3×1]   (zeros if IMU disabled)
%   ▸ Out3 – Gyro  XYZ, single [3×1]   (zeros if IMU disabled)
%
%   • One block per model
%   • Simulation only (Normal / Accelerator); no Coder support

% ── USER SETTINGS ─────────────────────────────────────────────────────
imgW      = 1280;   % pixels
imgH      = 720;    % pixels
fps       = 30;     % frames per second
enableIMU = true;
% ---------------------------------------------------------------------
vecLen = imgW * imgH * 3;   % elements in one RGB frame

% Shared objects (captured by nested functions)
pipe = [];                  %#ok<NASGU>  % realsense.pipeline

% Tell Simulink about ports & sample time
setup(block);

%======================================================================%
    function setup(block)
        block.NumInputPorts  = 0;
        block.NumOutputPorts = 3;                % colour / accel / gyro

        % Port-1 : colour image vector (uint8)
        setPort(block,1,vecLen,3);               % 3 = uint8
        % Port-2 : accel XYZ  (single)
        setPort(block,2,3,1);                    % 1 = single
        % Port-3 : gyro  XYZ  (single)
        setPort(block,3,3,1);

        block.SampleTimes        = [1/fps 0];
        block.SimStateCompliance = 'DefaultSimState';
        block.RegBlockMethod('Start',     @Start);
        block.RegBlockMethod('Outputs',   @Outputs);
        block.RegBlockMethod('Terminate', @Terminate);
    end

%------------------------------------------------------------------%
    function setPort(bl,idx,dim,dtid)
        bl.OutputPort(idx).Dimensions   = dim;
        bl.OutputPort(idx).DatatypeID   = dtid;
        bl.OutputPort(idx).Complexity   = 'Real';
        bl.OutputPort(idx).SamplingMode = 'Sample';
    end

%======================================================================%
    function Start(~)
        if ~isempty(pipe)
            error('Only one realsense_sfun block is allowed per model.');
        end

        cfg = realsense.config();
        cfg.enable_stream(realsense.stream.color, imgW, imgH, ...
                          realsense.format.rgb8, fps);
        cfg.enable_stream(realsense.stream.accel);
        cfg.enable_stream(realsense.stream.gyro);

        pipe = realsense.pipeline();
        pipe.start(cfg);
    end

%======================================================================%
    function Outputs(block)
        if isempty(pipe)
            error('RealSense pipeline not initialised (Start failed).');
        end

        fs   = pipe.wait_for_frames();      % blocking read

        % Colour frame → vector
        cfrm = fs.get_color_frame();
        block.OutputPort(1).Data = frameToVec(cfrm, vecLen);

        % IMU frames
        afrm = fs.first(realsense.stream.accel).as('motion_frame');
        gfrm = fs.first(realsense.stream.gyro ).as('motion_frame');
        block.OutputPort(2).Data = single(afrm.get_motion_data());
        block.OutputPort(3).Data = single(gfrm.get_motion_data());

        delete(fs);                         % free native handles
    end

%======================================================================%
    function Terminate(~)
        if ~isempty(pipe)
            pipe.stop();
            delete(pipe);
            pipe = [];
        end
    end

%======================================================================%
    function v = frameToVec(frame, N)
        % Convert RealSense frame to MATLAB-order column vector (RGB)
        w   = frame.get_width();
        h   = frame.get_height();
        buf = frame.get_data();                 % row vector uint8

        img = permute(reshape(buf',[3, w, h]), [3 2 1]); % H×W×3
        v   = reshape(img, N, 1);
    end
end
