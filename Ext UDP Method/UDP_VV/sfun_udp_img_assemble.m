function sfun_udp_img_assemble(block)
    % UDP로 분할 수신된 카메라 이미지를 조립하여 [240,320] uint8 출력
    setup(block);
end

function setup(block)
    block.NumDialogPrms = 4; % [PORT, IMG_SIZE, CHUNK_SIZE, TIMEOUT]
    block.NumInputPorts  = 0;
    block.NumOutputPorts = 1;

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
    block.NumDworks = 4;
    block.Dwork(1).Name = 'img_buf';      % 현재 조립 중 버퍼
    block.Dwork(1).Dimensions = prod(img_size);
    block.Dwork(1).DatatypeID = 3; % uint8
    block.Dwork(1).Complexity = 'Real';
    block.Dwork(1).UsedAsDiscState = true;

    block.Dwork(2).Name = 'frame_id';     % 현재 조립 중 frame id (double)
    block.Dwork(2).Dimensions = 1;
    block.Dwork(2).DatatypeID = 0; % double
    block.Dwork(2).Complexity = 'Real';
    block.Dwork(2).UsedAsDiscState = true;

    block.Dwork(3).Name = 'chunk_flags';  % 청크 수신 마킹
    block.Dwork(3).Dimensions = 128; % (최대 청크 개수 여유)
    block.Dwork(3).DatatypeID = 0; % double
    block.Dwork(3).Complexity = 'Real';
    block.Dwork(3).UsedAsDiscState = true;

    block.Dwork(4).Name = 'last_complete'; % 출력용 최종 완성 프레임
    block.Dwork(4).Dimensions = prod(img_size);
    block.Dwork(4).DatatypeID = 3; % uint8
    block.Dwork(4).Complexity = 'Real';
    block.Dwork(4).UsedAsDiscState = true;
end

function Start(block)
    port      = block.DialogPrm(1).Data;
    img_size  = block.DialogPrm(2).Data;
    chunk_sz  = block.DialogPrm(3).Data;

    u = dsp.UDPReceiver('LocalIPPort', port, ...
        'MessageDataType', 'uint8', ...
        'MaximumMessageLength', chunk_sz+8, ...
        'ReceiveBufferSize', 131072, ...    % 넉넉하게
        'BlockingTime', 0.001);
    assignin('base', 'udpimg_handle', u);

    block.Dwork(1).Data = zeros(prod(img_size), 1, 'uint8');
    block.Dwork(2).Data = 0;      % double
    block.Dwork(3).Data = zeros(128,1); % double
    block.Dwork(4).Data = zeros(prod(img_size), 1, 'uint8');
end

function Outputs(block)
    img_size  = block.DialogPrm(2).Data;
    chunk_sz  = block.DialogPrm(3).Data;

    u = [];
    try
        u = evalin('base', 'udpimg_handle');
    catch
        block.OutputPort(1).Data = reshape(block.Dwork(4).Data, img_size);
        return;
    end

    % 여러 패킷 수신
    max_iter = 32;
    cnt = 0;
    while true
        pkt = u();
        if isempty(pkt), break; end
        if numel(pkt)<8, continue; end

        frame_id = double(typecast(pkt(1:4),'uint32'));
        chunk_id = double(typecast(pkt(5:6),'uint16')) + 1;
        total_chunks = double(typecast(pkt(7:8),'uint16'));

        data_bytes = pkt(9:end);

        % 새로운 프레임 시작: 버퍼 초기화
        if block.Dwork(2).Data ~= frame_id
            block.Dwork(1).Data(:) = 0;
            block.Dwork(2).Data = frame_id;
            block.Dwork(3).Data(:) = 0;
        end

        % 데이터 복사
        offset = (chunk_id-1)*chunk_sz + 1;
        nbytes = numel(data_bytes);
        if offset+nbytes-1 <= numel(block.Dwork(1).Data)
            block.Dwork(1).Data(offset:offset+nbytes-1) = data_bytes;
            block.Dwork(3).Data(chunk_id) = 1;
        end
        block.Dwork(3).Data(total_chunks+1:end) = 0;

        % cnt = cnt+1;
        % if cnt>=max_iter, break; end
    end

    % 완성여부 판정
    idx = find(block.Dwork(3).Data>0);
    if isempty(idx)
        total_chunks = 0;
    else
        total_chunks = max(idx);
    end

    frame_ready = false;
    if (total_chunks>0) && all(block.Dwork(3).Data(1:total_chunks)==1)
        frame_ready = true;
    end

    if frame_ready
        block.Dwork(4).Data = block.Dwork(1).Data;
    end

    % 항상 마지막 완성 프레임 출력
    block.OutputPort(1).Data = reshape(block.Dwork(4).Data, img_size);
end

function Terminate(block)
    try
        u = evalin('base', 'udpimg_handle');
        release(u)
        evalin('base', 'clear udpimg_handle');
    catch
    end
end
