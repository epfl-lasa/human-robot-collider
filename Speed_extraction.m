
%%Read the video file
videoFileReader = vision.VideoFileReader('Test_12-3_H3-Legs_right_long_1000fps.avi');
S = info(videoFileReader);
%%parameters
frameRate = 1000; % frame/second
scale = 1/300; % m/pixel
%%define the region of interest(objectRegion)
videoPlayer = vision.VideoPlayer('Position',[100 100 600 400]);
videoFrame = step(videoFileReader);
objectRegion = [475,405,10,15];
objectImage = insertShape(videoFrame,'Rectangle',objectRegion,'Color','red');
%%Initializing the points of interest in the selected region + Tracker
points = detectMinEigenFeatures(im2gray(videoFrame),'ROI',objectRegion);
oldPoints = 0;
velList = [0;0;0;0];
pointImage = insertMarker(videoFrame,points.Location,'+','Color','white');
tracker = vision.PointTracker('NumPyramidLevels',4);
initialize(tracker,points.Location,videoFrame);
while ~isDone(videoFileReader)
      frame = step(videoFileReader);
      [points,validity] = tracker(frame);
      out = insertMarker(frame,points(validity, :),'+');
      if ~isempty(oldPoints)
        % Calculate velocity (pixels/frame)
        vel_pix = sqrt(sum((points-oldPoints).^2,2));
        vel = vel_pix * frameRate * scale; % pixels/frame * frame/seconds * meter/pixels
        if vel < 3 %disregard parasite values
            velList = [velList vel];
        end
      else
        vel_pix = 0;
        vel = 0;
      end
      % Visualize the velocity
      videoFrameOut = insertObjectAnnotation(videoFrame, 'circle', ...
          [points 10*ones(size(points,1),1)], ...
          cellstr(num2str(vel,'%2.2f')));
      step(videoPlayer, videoFrameOut);
      oldPoints = points;
end
%%Plotting the velocity for one of the points (smoothing done first to
%%disregard noise
velList = smooth(velList(1,:)');
velList = velList';
figure
plot(velList)
title('Collision Velocity')
xlabel('Frames')
ylabel('Velocity')
release(videoFileReader);
release(videoPlayer);