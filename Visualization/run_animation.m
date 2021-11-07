
% path of the *.mat file containing the 3d model information
model_info_file = 'Rocket.mat';
euler = importfile('../attitude.txt', 1, 200000);


n=length(euler);
yout=zeros(n,10);

for i=1:n
    yout(i,7)= euler(i,1)*pi/180;
    yout(i,8)= euler(i,2)*pi/180;
    yout(i,9)= euler(i,3)*pi/180;

    
end
% define the reproduction speed factor
speedx = 1; 
% Do you want to save the animation in a mp4 file? (0.No, 1.Yes)
isave_movie = 0;
% Movie file name
movie_file_name = '';

sampletime = 0.001;
tf = (n-1)*sampletime;
tout = 0:sampletime:tf;
tout = tout';
% -------------------------------------------------------------------------
% The frame sample time shall be higher than 0.02 seconds to be able to 
% update the figure (CPU/GPU constraints)
frame_sample_time = max(0.1, tout(2)-tout(1));
% Resample the time vector to modify the reproduction speed
t_new   = tout(1):frame_sample_time*(speedx):tout(end);
% Resample the recorded data
y_new   = interp1(tout, yout, t_new','linear');
% We have to be careful with angles with ranges
y_new(:, 7)  = atan2(interp1(tout, sin(yout(:, 7)), t_new','linear'), interp1(tout, cos(yout(:, 7)), t_new','linear')) * 180 / pi;
y_new(:, 8)  = atan2(interp1(tout, sin(yout(:, 8)), t_new','linear'), interp1(tout, cos(yout(:, 8)), t_new','linear'))* 180 / pi;
y_new(:, 9)  = atan2(interp1(tout, sin(yout(:, 9)), t_new','linear'), interp1(tout, cos(yout(:, 9)), t_new','linear'))* 180 / pi;
% Assign the data
heading_deg           =  y_new(:, 9);
pitch_deg             =  y_new(:, 8);
bank_deg              =  y_new(:, 7);

%% Run aircraft_3d_animation function
% -------------------------------------------------------------------------
aircraft_3d_animation(model_info_file,...
    heading_deg, ...            Heading angle [deg]
    pitch_deg, ...              Pitch angle [deg]
    bank_deg, ...               Roll angle [deg]
    frame_sample_time, ...      Sample time [sec]
    speedx, ...                 Reproduction speed
    isave_movie, ...            Save the movie? 0-1
    movie_file_name);           % Movie file name