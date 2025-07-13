classdef RealsenseRGBSystem < matlab.System & matlab.system.mixin.Propagates
    % Stream 640x480@60 RGB from D435i using librealsense
    properties (Nontunable)
        Width  = 640
        Height = 480
        FPS    = 60
    end
    properties (Access=private)
        pipeline
    end
    methods (Access=protected)
        function setupImpl(obj)
            % Initialize pipeline and enable color stream
            obj.pipeline = realsense.pipeline();
            cfg = realsense.config();
            cfg.enable_stream( ...
                realsense.stream.Color, ...
                realsense.format.RGB8, ...
                obj.FPS, obj.Width, obj.Height);
            obj.pipeline.start(cfg);
        end
        function img = stepImpl(obj)
            % Grab one frame, reshape and permute to H×W×3
            frames      = obj.pipeline.wait_for_frames();
            colorFrame  = frames.get_color_frame();
            raw         = colorFrame.get_data();            % 1×(W*H*3) uint8
            reshaped    = reshape(raw, 3, obj.Width, obj.Height);
            img         = permute(reshaped, [3,2,1]);      % H×W×3
        end
        function releaseImpl(obj)
            obj.pipeline.stop();
        end
        %% Signal attributes
        function [sz1] = getOutputSizeImpl(~),    sz1 = [480,640,3];    end
        function     num = getNumOutputsImpl(~),  num = 1;             end
        function logical = isOutputComplexImpl(~),logical = false;      end
        function    dt  = getOutputDataTypeImpl(~),dt = 'uint8';       end
        function    cp  = isOutputFixedSizeImpl(~),cp = true;          end
        function    icon = getIconImpl(~),        icon = 'RGB';         end
    end
end