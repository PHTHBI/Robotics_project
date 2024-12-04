function centers = find_circles(image)
    % FIND_CIRCLES detects the centers of circles in the given image.
    
    if size(image, 3) == 3
        image = rgb2gray(image);
    end

    % Smooth and enhance edges
    smoothedImage = imgaussfilt(image, 1.5);
    enhancedImage = adapthisteq(smoothedImage);
    edges = edge(enhancedImage, 'Canny', [0.1, 0.3]);

    % Detect circles using imfindcircles
    [centers, radii, metrics] = imfindcircles(edges, [15 200], ...
        'ObjectPolarity', 'bright', 'Sensitivity', 0.95, 'EdgeThreshold', 0.15);

    % Filter circles by detection metric
    minMetric = 0.4;
    validIndices = metrics > minMetric;
    centers = centers(validIndices, :);
    radii = radii(validIndices);
    metrics = metrics(validIndices);

    % Handle overlapping circles
    if ~isempty(centers)
        overlapThreshold = 0.7;
        validCircleMask = true(size(centers, 1), 1);
        for i = 1:size(centers, 1)
            for j = i+1:size(centers, 1)
                centerDistance = sqrt((centers(i, 1) - centers(j, 1))^2 + (centers(i, 2) - centers(j, 2))^2);
                if centerDistance < overlapThreshold * max(radii(i), radii(j))
                    if metrics(i) >= metrics(j)
                        validCircleMask(j) = false;
                    else
                        validCircleMask(i) = false;
                    end
                end
            end
        end
        centers = centers(validCircleMask, :);
    end
end

% --- Helper Functions ---
function [x_int, y_int] = line_intersection(line1, line2)
    x1 = line1(1, 1); y1 = line1(1, 2);
    x2 = line1(2, 1); y2 = line1(2, 2);
    x3 = line2(1, 1); y3 = line2(1, 2);
    x4 = line2(2, 1); y4 = line2(2, 2);
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if denom == 0, x_int = NaN; y_int = NaN; return; end
    x_int = ((x1*y2 - y1*x2) * (x3 - x4) - (x1 - x2) * (x3*y4 - y3*x4)) / denom;
    y_int = ((x1*y2 - y1*x2) * (y3 - y4) - (y1 - y2) * (x3*y4 - y3*x4)) / denom;
end