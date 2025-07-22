function sfun_udp_imu(block)
% SFUN_UDP_IMU  Level-2 M-file S-function
%   Receives one UDP packet per step (24 bytes = 6×float32) and outputs
%   [ax; ay; az; gx; gy; gz] as single-precision (1×6).
%
% Dialog parameters:
%   1) Local UDP port      (scalar)
%   2) Sample time (sec)   (scalar)

% -------------------------------------------------------------------------
% Setup
% -------------------------------------------------------------------------
setup(block);

% -------------------------------------------------------------------------
function setup(block)
    block.NumDialogPrms = 2;                % [PORT, Ts]
    block.NumInputPorts  = 0;
    block.NumOutputPorts = 1;

    % Output: 6-element single row-vector
    block.OutputPort(1).Dimensions  = [6 1];
    block.OutputPort(1).DatatypeID  = 1;    % single
    block.OutputPort(1).Complexity  = 'Real';
    block.OutputPort(1).SamplingMode = 'Sample';

    Ts = block.DialogPrm(2).Data;
    block.SampleTimes = [Ts 0];

    block.SimStateCompliance = 'DefaultSimState';

    block.RegBlockMethod('PostPropagationSetup', @PostPropSetup);
    block.RegBlockMethod('Start',               @Start);
    block.RegBlockMethod('Outputs',             @Outputs);
    block.RegBlockMethod('Terminate',           @Terminate);
end

% -------------------------------------------------------------------------
function PostPropSetup(block)
    % DWork(1): last valid 6-axis reading
    block.NumDworks = 1;
    block.Dwork(1).Name            = 'imu_last';
    block.Dwork(1).Dimensions      = 6;
    block.Dwork(1).DatatypeID      = 1;   % single
    block.Dwork(1).Complexity      = 'Real';
    block.Dwork(1).UsedAsDiscState = true;
end

% -------------------------------------------------------------------------
function Start(block)
    port = block.DialogPrm(1).Data;

    % Create the UDP receiver object; store in base for easy access
    u = dsp.UDPReceiver( ...
        'LocalIPPort',         port, ...
        'MessageDataType',     'uint8', ...
        'MaximumMessageLength', 24, ...
        'ReceiveBufferSize',   65536, ...
        'BlockingTime',        0.001);

    assignin('base', 'udpimu_handle', u);

    % Init last sample to zeros
    block.Dwork(1).Data = single(zeros(6,1));
end

% -------------------------------------------------------------------------
function Outputs(block)
    % Retrieve handle (created in Start)
    try
        u = evalin('base', 'udpimu_handle');
    catch
        block.OutputPort(1).Data = block.Dwork(1).Data;
        return
    end

    % Drain all waiting packets, keep the most recent
    while true
        pkt = u();
        if isempty(pkt)
            break
        end
        if numel(pkt) == 24
            % Convert 24 uint8 → 6 single (little endian)
            vals = typecast(uint8(pkt(:)).','single'); %#ok<DTCAST>
            block.Dwork(1).Data = vals(:);   % store column-vector
        end
    end

    % Output latest valid sample (or zeros until first packet arrives)
    block.OutputPort(1).Data = block.Dwork(1).Data;
end

% -------------------------------------------------------------------------
function Terminate(block)
    try
        u = evalin('base', 'udpimu_handle');
        release(u);
        evalin('base', 'clear udpimu_handle');
    catch
        % nothing to clean
    end
end
end
