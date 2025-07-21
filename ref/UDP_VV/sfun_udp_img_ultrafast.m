function sfun_udp_img_fast(block)
    % 초고속 UDP 이미지 조립 S-Function
    setup(block);
end

function setup(block)
    block.NumDialogPrms = 3; % [PORT, IMG_SIZE, CHUNK_SIZE]
    block.NumInputPorts  = 0;
    block.NumOutputPorts = 1;pilot_test_slc1

    img_size = block.DialogPrm(2).Data;
    block.OutputPort(1).Dimensions = img_size;
    block.OutputPort(1).DatatypeID = 3;  % uint8
    block.OutputPort(1).SamplingMode = 'Sample';
    block.SampleTimes = [0.01 0];
    block.SimStateCompliance = 'DefaultSimState';

    block.RegBlockMethod('PostPropagationSetup', @PostPropSetup);
    block.RegBlockMethod('Start', @Start);
    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('Terminate', @Terminate);
end

function PostPropSetup(block)
    img_size = block.DialogPrm(2).Data;
    block.NumDworks = 3;
    block.Dwork(1).Name = 'img_buf';      % 조립 중 버퍼
    block.Dwork(1).Dimensions = prod(img_size);
    block.Dwork(1).DatatypeID = 3; % uint8
    block.Dwork(1).Complexity = 'Real';
    block.Dwork(1).UsedAsDiscState = true;

    block.Dwork(2).Name = 'frame_id';     % 현재 프레임 id
    block.Dwork(2).Dimensions = 1;
    block.Dwork(2).DatatypeID = 0; % double
    block.Dwork(2).Complexity = 'Real';
    block.Dwork(2).UsedAsDiscState = true;

    block.Dwork(3).Name = 'last_output';  % 마지막 출력
    block.Dwork(3).Dimensions = prod(img_size);
    block.Dwork(3).DatatypeID = 3; % uint8
    block.Dwork(3).Complexity = 'Real';
    block.Dwork(3).UsedAsDiscState = true;
end

function Start(block)
    port      = block.DialogPrm(1).Data;
    img_size  = block.DialogPrm(2).Data;
    chunk_sz  = block.DialogPrm(3).Data;

    u = dsp.UDPReceiver('LocalIPPort', port, ...
        'MessageDataType', 'uint8', ...
        'MaximumMessageLength', chunk_sz+8, ...
        'ReceiveBufferSize', 131072, ...
        'BlockingTime', 0.001);
    assignin('base', 'udpimg_handle', u);

    block.Dwork(1).Data = zeros(prod(img_size), 1, 'uint8');
    block.Dwork(2).Data = 0;
    block.Dwork(3).Data = zeros(prod(img_size), 1, 'uint8');
end

function Outputs(block)
    img_size  = block.DialogPrm(2).Data;
    chunk_sz  = block.DialogPrm(3).Data;

    u = [];
    try
        u = evalin('base', 'udpimg_handle');
    catch
        block.OutputPort(1).Data = reshape(block.Dwork(3).Data, img_size);
        return;
    end

    max_iter = 512;
    while max_iter > 0
        pkt = u();
        if isempty(pkt), break; end
        if numel(pkt)<8, continue; end

        frame_id = double(typecast(pkt(1:4),'uint32'));
        chunk_id = double(typecast(pkt(5:6),'uint16')) + 1;

        if block.Dwork(2).Data ~= frame_id
            block.Dwork(1).Data(:) = 0;
            block.Dwork(2).Data = frame_id;
        end

        data_bytes = pkt(9:end);
        offset = (chunk_id-1)*chunk_sz + 1;
        nbytes = numel(data_bytes);
        if offset+nbytes-1 <= numel(block.Dwork(1).Data)
            block.Dwork(1).Data(offset:offset+nbytes-1) = data_bytes;
        end
        max_iter = max_iter-1;
    end
    % 조립 완성 여부 따지지 않고, 받은대로 바로 출력
    block.Dwork(3).Data = block.Dwork(1).Data;
    block.OutputPort(1).Data = reshape(block.Dwork(3).Data, img_size);
end

function Terminate(block)
    try
        u = evalin('base', 'udpimg_handle');
        release(u)
        evalin('base', 'clear udpimg_handle');
    catch
    end
end
