function exampleHelperTurtleBotPlotObject(img, binImg, c)
    %exampleHelperTurtleBotPlotObject Display color and binary images with center point
    %   exampleHelperTurtleBotPlotObject(IMG,BINIMG,C) - creates a figure displaying a color
    %   image and a binary image, placing the point C on both images. Used
    %   primarily for visualizing object isolation using image filtering
    %
    %   See also exampleHelperTurtleBotTrackingTimer, exampleHelperTurtleBotFindBlueBall
    
    %   Copyright 2014 The MathWorks, Inc.
    
    figure('Name','Image Calibration','Position',[50,50,400,600]);
    subplot(2,1,1)
    hold on
    imshow(img);
    if ~isempty(c)
        plot(c(1),c(2),'+','Color','r','MarkerSize',20);
    end
    hold off
    subplot(2,1,2)
    hold on
    imshow(binImg);
    if ~isempty(c)
        plot(c(1),c(2),'+','Color','r','MarkerSize',20);
    end
    hold off
end

