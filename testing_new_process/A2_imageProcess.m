function finalImg = A2_imageProcess(imagePath)
    % Read the image
    img = imread(imagePath);
    
    % Step 1: Convert to grayscale
    grayImg = rgb2gray(img);
    
    % Step 2: Apply thresholding to isolate dark lines
    threshold = 0.32; % Adjust this value as needed
    bwImg = imbinarize(grayImg, threshold);
    bwImg = ~bwImg; % Invert to make lines white on black
    
    % Step 3: Remove noise
    cleanedImg = bwareaopen(bwImg, 20); % Remove small objects (e.g., noise)
    
    % Step 4: Smooth the lines
    % Gaussian filtering to smooth pixelated edges
    smoothedImg = imgaussfilt(double(cleanedImg), 1.2); % Gaussian blur with sigma=1.5
    
    % Re-threshold to make edges crisp again
    smoothedImg = smoothedImg > 0.65; % Convert back to binary
    
    % Step 5: Make the lines skinnier
    se = strel('disk', 4); % Structuring element (disk-shaped)
    skinnyLines = imerode(smoothedImg, se); % Morphological erosion
    
    % Final cleaning for small artifacts
    finalImg = bwareaopen(skinnyLines, 20); % Remove small noise again
    
    % Convert the final binary image to uint8 format (compatible with imadjust)
    finalImg = uint8(finalImg) * 255; % Convert to uint8 (white = 255, black = 0)
end
