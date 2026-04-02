
%% function to setup figure window 
function [j,ball_image,alpha_channel, damage_img]=figure_setup_bike()

% adjust figure to desired size&position, then use f.Position to get #s
j=figure('position',[0 50 3840 2160]); %%width then height form bottom left. Size in pixels 
hold on % prevent axes from flipping y-axis when plotting image

%axis set up 
% axis are equal in size
% to use up full figure window resize and reposition axes of plot
% to its normalized limits (0 to 1)
axis('equal') % set aspect ratio so equal tick mark increments on each 
axis('manual') % freeze all axis limits for subsequent plots so they
% do not automatically adjust on the fly

ax=gca; % get current axes handle
ax.Position=[0 0 1 1]; % normalized position of axes [left bottom width height]
set(gcf,'Toolbar','none','Menu','none'); % remove toolbar and menu
set(gca,'visible','off'); % remove axis labels
set(gcf,'color','w'); % make figure background white
set(gca, 'Position', [0 0 1 1])

ylim([0 2160]) % set y axis limits
xlim([-100 3840]) %set x axis limits 

%% background picture load in 
bg=imread('newbkg_v3.png'); 
bg=flipud(bg); %flip so right side up. Because background image, dont need alpha.

%bk1= image(bg); %bk is now handle to modify later if needed (stores memory
%abt this background picture). 

%Want to set the x and y lim of picture to
%match figure window.
image(bg, ...
    'Xdata', [-90 4000],...
    'YData', [0 2160]); 


hold on %so figures can lay overtop
impixelinfo

%% static obstacle oil 
%obstacle 3 call; oil puddle 
scale_oil= 1.05; 
[oil_big,~, oil_alphabig]= imread('oil_puddle_v2.png'); 
oil_big=flipud(oil_big); 
oil_alphabig=flipud(oil_alphabig); 
oil= imresize(oil_big, scale_oil);
oil_alpha=imresize(oil_alphabig, scale_oil); 
[b,a,~]= size(oil); 
hoil=image(oil); %render oil image

set(hoil,...
    'Xdata', [2000-a/2, 2000+a/2], ...
    'Ydata', [1955-b/2, 1955+b/2], ...
    'AlphaData', oil_alpha); 

maskoil= oil_alpha> 0; %creating bool mask for oil 


scale_pothole= 0.6;

[pothole_big,~, pothole_alpha_big] = imread('pot_hole.png'); 
pothole_big=flipud(pothole_big);
pothole_alpha_big= flipud(pothole_alpha_big); 

pothole= imresize(pothole_big, scale_pothole); 
pothole_alpha= imresize(pothole_alpha_big, scale_pothole); 
[b1,a1,~]= size(pothole); 
hpothole = image(pothole); %render image 


set(hpothole, ...
    'Xdata', [3600-a1/2, 3600+a1/2], ...
    'Ydata', [400-b1/2, 400+b1/2], ...
    'AlphaData', pothole_alpha);  




%% Insert character player here **

% read image of black ball to track
% for fast performance, make image using as few pixels as possible
[ball_image,~,alpha_channel]=imread('circle_black_transparent.png');
ball_image=flipud(ball_image); % need to flip image so it is oriented correctly
% images are stored so that they face upwards when
% y axes are reversed (positive downwards),
% but our y axis is normal (postive upwards)
% so we need to flip the image so it faces upward
alpha_channel=flipud(alpha_channel);


[damage_img,~, alpha_damaged]= imread(); %sad face or something 
damaged_img=flipud(damaged_img); 
alpha_damaged=flupud(alpha_damaged); 
end
