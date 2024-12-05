classdef BoardDetector
    properties
        ImagePath % Path where the captured image will be stored
        gridCorners = [];% Detected grid corners
        cellCenters = [];% Detected grid cell centers
    end

    methods
        function obj = BoardDetector(imagePath)
            % Constructor: Initialize the class with the image path
            obj.ImagePath = imagePath;
        end

        function captureBoardImage(obj)
            % Captures an image of the tic-tac-toe board
            %vid = videoinput('winvideo', '2'); % Adjust camera ID if necessary
            %set(vid, 'ReturnedColorSpace', 'rgb');
            %vid.FramesPerTrigger = 1;
            %preview(vid);

            cam = webcam('Logi C270 HD WebCam');
            %disp('Capturing tic-tac-toe board image...');
            %pause(2); % Allow the preview to stabilize
            img = snapshot(cam);
            imshow(img);
            imwrite(img, obj.ImagePath);
            %disp(['Image saved to ', obj.ImagePath]);

            %stoppreview(vid);
            %delete(vid);
        end

        function gridCorners = detectFourIntersections(obj)
            % Detects the four corners of the tic-tac-toe grid
            img = imread(obj.ImagePath);
            grayImg = rgb2gray(img);

            % Enhance the image and detect edges
            enhancedImg = imgaussfilt(grayImg, 2);
            edges = edge(enhancedImg, 'Canny', [0.2, 0.5]);

            % Skeletonize and detect lines
            thinnedEdges = bwmorph(edges, 'skel', Inf);
            [H, theta, rho] = hough(thinnedEdges);
            peaks = houghpeaks(H, 20, 'Threshold', ceil(0.3 * max(H(:))));
            lines = houghlines(thinnedEdges, theta, rho, peaks, 'FillGap', 20, 'MinLength', 50);

            % Separate horizontal and vertical lines
            angleTolerance = 10;
            horizontalLines = [];
            verticalLines = [];
            for k = 1:length(lines)
                angle = atan2d(lines(k).point2(2) - lines(k).point1(2), ...
                               lines(k).point2(1) - lines(k).point1(1));
                angle = mod(angle, 180);
                if abs(angle) <= angleTolerance || abs(angle - 180) <= angleTolerance
                    horizontalLines = [horizontalLines; lines(k)];
                elseif abs(angle - 90) <= angleTolerance
                    verticalLines = [verticalLines; lines(k)];
                end
            end

            % Filter to keep the longest lines
            horizontalLines = obj.filterLongestLines(horizontalLines, 2);
            verticalLines = obj.filterLongestLines(verticalLines, 2);

            % Find intersections
            intersections = [];
            for h = 1:length(horizontalLines)
                for v = 1:length(verticalLines)
                    [x_int, y_int] = obj.lineIntersection(...
                        [horizontalLines(h).point1; horizontalLines(h).point2], ...
                        [verticalLines(v).point1; verticalLines(v).point2]);
                    if ~isnan(x_int) && ~isnan(y_int)
                        intersections = [intersections; x_int, y_int];
                    end
                end
            end

            % Deduplicate intersections and filter to find closest four
            intersections = unique(round(intersections, 1), 'rows');
            gridCorners = obj.filterClosestFour(intersections);

            if size(gridCorners, 1) ~= 4
                error('Expected exactly 4 intersections. Detected %d.', size(gridCorners, 1));
            end
        end

        function cellCenters = findGridCellCenters(~, gridCorners)
            % Finds the centers of the tic-tac-toe grid cells
            if size(gridCorners, 1) ~= 4
                error('Expected 4 grid corners. Detected %d.', size(gridCorners, 1));
            end

            sortedCorners = sortrows(gridCorners, [1, 2]);
            topLeft = sortedCorners(1, :);
            bottomLeft = sortedCorners(2, :);
            topRight = sortedCorners(3, :);
            bottomRight = sortedCorners(4, :);

            xStep = (topRight(1) - topLeft(1)) / 2;
            yStep = (bottomLeft(2) - topLeft(2)) / 2;

            cellCenters = [];
            for row = -1:1
                for col = -1:1
                    centerX = topLeft(1) + (col + 0.5) * xStep * 2;
                    centerY = topLeft(2) + (row + 0.5) * yStep * 2;
                    cellCenters = [cellCenters; centerX, centerY];
                end
            end
        end
    end

    methods (Access = private)
        function [x_int, y_int] = lineIntersection(~, line1, line2)
            % Calculate the intersection of two lines
            x1 = line1(1, 1); y1 = line1(1, 2);
            x2 = line1(2, 1); y2 = line1(2, 2);
            x3 = line2(1, 1); y3 = line2(1, 2);
            x4 = line2(2, 1); y4 = line2(2, 2);

            denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
            if denom == 0
                x_int = NaN; y_int = NaN;
                return;
            end

            x_int = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom;
            y_int = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom;
        end

        function longestLines = filterLongestLines(~, lines, numToKeep)
            % Keep the longest lines
            if isempty(lines)
                longestLines = [];
                return;
            end

            lengths = arrayfun(@(line) norm(line.point1 - line.point2), lines);
            [~, sortedIdx] = sort(lengths, 'descend');
            longestLines = lines(sortedIdx(1:min(numToKeep, length(sortedIdx))));
        end

        function closestFour = filterClosestFour(~, intersections)
            % Select four closest points to the centroid
            if size(intersections, 1) <= 4
                closestFour = intersections;
                return;
            end

            centroid = mean(intersections, 1);
            distances = vecnorm(intersections - centroid, 2, 2);
            [~, sortedIdx] = sort(distances);
            closestFour = intersections(sortedIdx(1:4), :);
        end
    end
end
