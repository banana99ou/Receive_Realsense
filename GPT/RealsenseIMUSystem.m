classdef RealsenseIMUSystem < matlab.System & matlab.system.mixin.Propagates
    % Stream 3-axis accel+gyro (MOTION_XYZ32F) at 200 Hz
    properties (Nontunable)
        SampleRate = 200
    end
    properties (Access=private)
        pipeline
    end
    methods (Access=protected)
        function setupImpl(obj)
            obj.pipeline = realsense.pipeline();
            cfg = realsense.config();
            cfg.enable_stream( ...
               realsense.stream.Motion, ...
               realsense.format.MOTION_XYZ32F, ...
               obj.SampleRate);
            obj.pipeline.start(cfg);
        end
        function imu = stepImpl(obj)
            frames = obj.pipeline.wait_for_frames();
            m      = frames.get_motion_frame();
            d      = m.get_motion_data();
            imu    = [d.x, d.y, d.z];   % 1×3 double
        end
        function releaseImpl(obj)
            obj.pipeline.stop();
        end
        %% Signal attributes
        function [sz] = getOutputSizeImpl(~),    sz = [1,3];      end
        function    num = getNumOutputsImpl(~),  num = 1;        end
        function logical = isOutputComplexImpl(~),logical = false; end
        function    dt  = getOutputDataTypeImpl(~),dt = 'double'; end
        function    cp  = isOutputFixedSizeImpl(~),cp = true;    end
        function    icon = getIconImpl(~),        icon = 'IMU';    end
    end
end