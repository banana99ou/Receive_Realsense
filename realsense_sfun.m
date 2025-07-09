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
imgW      = 320;   % pixels
imgH      = 240;    % pixels
fps       = 30;     % frames per second
prev = 0;
% ---------------------------------------------------------------------
vecLen = imgW * imgH * 3;   % elements in one RGB frame

% Shared objects (captured by nested functions)
pipe = [];                  %#ok<NASGU>  % realsense.pipeline
% lastFs = [];

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

        block.SampleTimes        = [0.01 0];
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
        cfg.enable_stream(realsense.stream.color, imgW, imgH, realsense.format.rgb8, fps);
        cfg.enable_stream(realsense.stream.accel);
        cfg.enable_stream(realsense.stream.gyro);

        pipe = realsense.pipeline();
        pipe.start(cfg);

        % % — prime the cache so lastFs isn’t empty —
        % tempFs = pipe.wait_for_frames();
        % lastFs = tempFs;
    end

%======================================================================%
    function Outputs(block)
        persistent lastColorVec lastAccel lastGyro
        if isempty(lastColorVec)
            % Prime the caches on first call
            lastColorVec = zeros(vecLen,1,'uint8');
            lastAccel    = single([0;0;0]);
            lastGyro     = single([0;0;0]);
        end
        % ── TIMING INSTRUMENTATION ───────────────────────────
        persistent simZero realZero polled run
        if isempty(simZero)
            simZero  = block.CurrentTime;  % should be 0
            realZero = tic;                % start wall clock
            polled   = toc(realZero);
            run = toc(realZero);
        end
        % simElapsed  = block.CurrentTime  - simZero;
        realElapsed = toc(realZero);
        % % fprintf('SimTime=%.3f  RealElapsed=%.3f\n', simElapsed, realElapsed);
        % ── Poll for new frames ───────────────────────────────
        if pipe.poll_for_frames()
            temp = toc(realZero) - polled;
            fprintf('polled every=%.3f', temp);
            polled = toc(realZero);
            % Grab the fresh set
            fs = pipe.wait_for_frames();
            
            % Rebuild the color vector only now
            % cfrm = fs.get_color_frame();
            % lastColorVec = frameToVec(cfrm,vecLen);
            
            % Always update IMU too
            afrm = fs.first(realsense.stream.accel).as('motion_frame');
            gfrm = fs.first(realsense.stream.gyro ).as('motion_frame');
            lastAccel    = single(afrm.get_motion_data());
            lastGyro     = single(gfrm.get_motion_data());
            
            delete(fs);      % free old handles
        end

        temp = toc(realZero) - run;
        fprintf('running=%.3f\n', temp);
        run = toc(realZero);

        % ── Output the cached values ──────────────────────────
        % block.OutputPort(1).Data = lastColorVec;
        block.OutputPort(2).Data = lastAccel;
        block.OutputPort(3).Data = lastGyro;
    end

%======================================================================%
    function Terminate(~)
        if ~isempty(pipe)
            pipe.stop();
            delete(pipe);
            pipe = [];
        end
        % if ~isempty(lastFs)
        %     delete(lastFs);
        %     lastFs = [];
        % end
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
