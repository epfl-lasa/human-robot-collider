
%%Read the video file (Uncomment the needed one)
%videoFileReader = vision.VideoFileReader('Test_12-3_H3-Legs_right_long_1000fps.avi');
%videoFileReader = vision.VideoFileReader('Test_06_Q3-Head_right_long_1000fps.avi');
%videoFileReader = vision.VideoFileReader('Test_09_Q3-Legs_right_long_1000fps.avi');
%videoFileReader = vision.VideoFileReader('Test_15-2_Q3-Ribcage_right_long_1000fps.avi');
S = info(videoFileReader);
%%parameters (Uncomment the needed one)
frameRate = 1000; % frame/second
scale = 1/300; % m/pixel
%%define the region of interest(objectRegion)
videoPlayer = vision.VideoPlayer('Position',[100 100 600 400]);
videoFrame = step(videoFileReader);
%Test12
%objectRegion = [475,405,10,15];   % For the region of interest
%objectRegion = [475,254,10,175];  % For the whole checkboard 
%Test 6
%objectRegion = [485,395,10,15];   % For the region of interest
%objectRegion = [485,244,10,185];  % For the whole checkboard 
%Test 9
%objectRegion = [505,405,10,15];   % For the region of interest
%objectRegion = [505,244,10,185];  % For the whole checkboard
%Test 15-2
%objectRegion = [510,405,10,15];  % For the region of interest
%objectRegion = [511,260,10,170]; % For the whole checkboard

objectImage = insertShape(videoFrame,'Rectangle',objectRegion,'Color','red');
figure;
imshow(objectImage);
%%Initializing the points of interest in the selected region + Tracker
points = detectMinEigenFeatures(im2gray(videoFrame),'ROI',objectRegion);
oldPoints = 0;
%Test 12
%velList = [0;0;0;0];     % For the region of interest
%velList = zeros([53,1]); % For the whole checkboard
%Test 6
%velList = zeros([3 1]);  % For the region of interest
%velList = zeros([41 1]); % For the whole checkboard
%Test 9
%velList = [0;0;0;0;0];   % For the region of interest
%velList = zeros([53 1]); % For the whole checkboard
%Test 15-2 
%velList = zeros([6 1]);  % For the region of interest
%velList = zeros([50 1]); % For the whole checkboard

pointImage = insertMarker(videoFrame,points.Location,'+','Color','white');
tracker = vision.PointTracker('NumPyramidLevels',4);
initialize(tracker,points.Location,videoFrame);
%%
while ~isDone(videoFileReader)
      frame = step(videoFileReader);
      [points,validity] = tracker(frame);
      out = insertMarker(frame,points(validity, :),'+');
      if ~isempty(oldPoints)
        % Calculate velocity (pixels/frame)
        vel_pix = sqrt(sum((points-oldPoints).^2,2));
        vel = vel_pix * frameRate * scale; % pixels/frame * frame/seconds * meter/pixels
        if vel < 5 %disregard parasite values
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

%% Save the file
vel_scaling = 3.1./(velList);
velList = vel_scaling .* velList;
save('Test15.mat', 'velList'); % Change .mat filename according to the test number
release(videoFileReader);
release(videoPlayer);
%% Plot
idx = 1;
%Smoothing of one of the curves to better visualize the result

%velList = smooth(velList(idx,:)');
%velList = velList';

figure
plot(velList(idx,1:1300))
title('Collision Velocity')
xlabel('Time [ms]')
ylabel('Velocity')


