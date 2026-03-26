close all
clear all
clc
% open figure window at desired location with desired dimensions
% of drawable area (drawable area excludes title, menu, and tool bars)
% obstacle_example.png is 800 wide x 400 high background image with 2 obstacles
%
% ENSURE FIGURE WINDOW DIMENSIONS EXACTLY AGREE WITH BACKGROUND
% IMAGE DIMENSIONS so (x,y) pixel locations in the figure correspond
% to (column,row) elements in the image matrix
% [left bottom width height]
f=figure('position',[2,50,800,400]); % move figure to where desired,
hold on % keep axes so increase y is upwards
axis('equal') % set aspect ratio so equal tick mark increments on each
% axis are equal in size
% to use full figure window, resize and reposition axes of plot to normalized
% limits (0 to 1)
ax=gca; % get current axes handle
ax.Position=[0 0 1 1]; % normalized position of axes [left bottom width height]
set(gcf,'Toolbar','none','Menu','none'); % remove toolbar and menu
set(gca,'visible','off'); % remove axis labels

bkgrnd=imread('obstacle_example.png');
bkgrnd=flipud(bkgrnd);
Bk2=image(bkgrnd);
axis('manual') % freeze all axis limits for subsequent plots so they
% do not automatically adjust on the fly
% display pixel info on lower LHS of figure window interactively based on mouse location
impixelinfo
return
[rocket,~,rocket_alpha]=imread('rocket_resized_transparent.png');
rocket=flipud(rocket);
rocket_alpha=flipud(rocket_alpha);
n=2000; % number of iterations
x=zeros(n+1); % preallocate position vector for speed
x(1)=400; % IC for x
y=100; % IC for y
scale=0.6; % scale factor for rocket ship: for best game performance
% ensure this is close to 1
[b,a,~]=size(rocket); % get b=height (y-axis), a=width (x-axis)
H3=image(rocket,'Xdata', [x(1)-scale*a/2,x(1)+scale*a/2], ...
'Ydata', [y-scale*b/2,y+scale*b/2], ...
'AlphaData', rocket_alpha); % spacecraft has transparent background

direction=5; % move from left to right
for i=1:n
    x(i+1)=x(i) + direction;
    if(bkgrnd(y,x(i+1),2) == 28) % look at G channel in RGB
                                 % rocket is hits obstacle:
    % reverse direction of rocket
    direction=-direction;
    % set position of rocket so it is at the previous
    % position (which did not hit the obstacle)
    x(i+1)=x(i);
end
    delete(H3);
    H3=image(rocket,'Xdata', [x(i)-scale*a/2,x(i)+scale*a/2], ...
'Ydata', [y-scale*b/2,y+scale*b/2], ...
'AlphaData', rocket_alpha); % spacecraft has transparent background
    pause(0.2);
end