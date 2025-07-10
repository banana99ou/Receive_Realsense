function realsense_sfun(block)
% Consumer S-Function: reads the globals filled by startRSbuffer
%   Out1: colour image vector (imgH*imgW*3 × 1 uint8)
%   Out2: accel [3×1 single]
%   Out3: gyro  [3×1 single]

    % USER tweak: must match startRSbuffer imgW, imgH
    imgW = 640;
    imgH = 480;

    vecLen = imgW*imgH*3;

    setup(block);

%---------------------------------------------
    function setup(block)
        block.NumInputPorts  = 0;
        block.NumOutputPorts = 3;

        setPort(block,1,vecLen,3);   % colour
        setPort(block,2,3,1);        % accel
        setPort(block,3,3,1);        % gyro

        block.SampleTimes = [0.01 0];        % 10 ms
        block.SimStateCompliance = 'DefaultSimState';
        block.RegBlockMethod('Outputs', @Outputs);
    end

    function setPort(bl,idx,dim,dtid)
        bl.OutputPort(idx).Dimensions   = dim;
        bl.OutputPort(idx).DatatypeID   = dtid;
        bl.OutputPort(idx).Complexity   = 'Real';
        bl.OutputPort(idx).SamplingMode = 'Sample';
    end

%---------------------------------------------
    function Outputs(block)
        global RS_BUFFER RS_ACCEL RS_GYRO

        % — initialize sim0/wall0 on first call —
        persistent sim0 wall0 prevTickWall
        if isempty(sim0)
            sim0  = block.CurrentTime;
            wall0 = tic;             % start wall clock
            prevTickWall = 0;
        end

        % — stamp times —
        simTime  = block.CurrentTime - sim0;
        wallTime = toc(wall0);
        fprintf('>>> Tick start: Sim=%.3f  Wall=%.3f\n', simTime, wallTime);

        % — measure our work —
        t0 = tic;
        imgVec = reshape(RS_BUFFER,[],1);  % very cheap
        % accel/gyro are just copies
        block.OutputPort(1).Data = imgVec;
        block.OutputPort(2).Data = RS_ACCEL;
        block.OutputPort(3).Data = RS_GYRO;
        execTime = toc(t0);

        % — drift vs perfect real time —
        drift = wallTime - simTime;
        fprintf('    Work=%.3f ms,  Drift=%.3f ms\n\n', execTime*1000, drift*1000);

        prevTickWall = wallTime;
    end
end