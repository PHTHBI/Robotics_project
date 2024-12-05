classdef CalibrateCamera
    properties
        CameraMatrix      % Stores the calibrated camera matrix
        DistCoeffs        % Stores the distortion coefficients
        RMSReprojectionError % RMS error from calibration
        Directory = "C:\Users\nanna\OneDrive\Skrivebord\MSc E24\Robotics\DynamixelSDK-3.7.31\DynamixelSDK-3.7.31\matlab";
        ChessboardSize = [7, 10];
        IsCalibrated = false; % Flag to check if calibration is done
    end

    methods
        function captureChess(obj, numImages)
            % Captures chessboard images for calibration
            vid = videoinput('winvideo', 2); % Adjust based on your camera
            set(vid, 'ReturnedColorSpace', 'rgb');
            vid.FramesPerTrigger = 1;
            preview(vid);

            disp('Capturing chessboard images...');
            
            for i = 1:numImages
                img = getsnapshot(vid);
                filename = fullfile(obj.Directory, sprintf('CI%d.jpg', i));
                imwrite(img, filename);
                disp(['Saved: ', filename]);
                pause(1); % Delay between captures
            end

            stoppreview(vid);
            delete(vid);
            disp('Image capture completed.');
        end

        function obj = calibrate(obj, showCalib)
            % Performs calibration using captured chessboard images
            if obj.IsCalibrated
                disp('Camera is already calibrated.');
                return;
            end

            %criteria = [3, 30, 0.001]; % Calibration criteria
            objpoints = {};
            imgpoints = {};
            objp = zeros(prod(obj.ChessboardSize), 3);
            [x, y] = meshgrid(0:obj.ChessboardSize(2)-1, 0:obj.ChessboardSize(1)-1);
            objp(:, 1:2) = [x(:), y(:)];

            imageFiles = dir(fullfile(obj.Directory, '*.jpg'));
            for i = 1:length(imageFiles)
                img = imread(fullfile(obj.Directory, imageFiles(i).name));
                gray = rgb2gray(img);
                [ret, corners] = detectCheckerboardPoints(gray);

                if ret
                    objpoints{end+1} = objp; %#ok<AGROW>
                    imgpoints{end+1} = corners; %#ok<AGROW>

                    if showCalib
                        imshow(insertMarker(img, corners, 'o', 'Color', 'red', 'Size', 5));
                        title('Detected Checkerboard Corners');
                        pause(0.5);
                    end
                end
            end

            [cameraParams, ~, estimationErrors] = estimateCameraParameters(imgpoints, objpoints, ...
                'ImageSize', size(gray), 'WorldUnits', 'millimeters');

            obj.CameraMatrix = cameraParams.IntrinsicMatrix;
            obj.DistCoeffs = cameraParams.RadialDistortion;
            obj.RMSReprojectionError = mean(estimationErrors(:));
            obj.IsCalibrated = true;

            disp('Calibration complete.');
            disp('Camera Matrix:');
            disp(obj.CameraMatrix);
            disp('Distortion Coefficients:');
            disp(obj.DistCoeffs);

            if showCalib
                close all;
            end
        end
    end
end
