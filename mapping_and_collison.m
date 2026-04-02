%   x_current: current step
%   x_predict: RK4 predicted step (next step)
%   mask: 1=allowed, 0=wall
%   substeps: number of small steps for safe movement


function collisionCode= mapping(mask1, maskbike, maskbluecar)
    % mapping and mask for obstacles using logical & so only detects if both are overlapped 
    overlap1= mask1 & maskbluecar; %player with bluecar 
    overlap2 =mask1 & maskbike; %player with bike 
    overlap4= mask1 & maskpothole; %player with pothole 

    % overlap3= mask1 & maskoil; %player with oil

    
    %change array to one column and check if any 1s for collision detection
    
    collision1= any(overlap1(:)); %with blueCar
    collision2= any(overlap2(:)); %with bike
    
    % collision3= any(overlap3(:)); %with oil 
    collision4= any(overlap4(:)); %with pothole
    
    %setting up for Switch. Assigning collision code.
    
    if collision1
        collisionCode =1; %player v bluecar 
    else collision2 
        collisionCode= 2; %player v bike 
    
    % elseif collision3
    %     collisionCode= 3;%player v oil 
    else collision4;
        collisionCode= 4; %player v pothole 
    end 
end 


function collisionCode2= staticobstacles(maskoil, maskpothole)

    overlap3= mask1 & maskoil; %player with oil
    % overlap4= mask1 & maskpothole; %player with pothole 
    
    collision3= any(overlap3(:)); %with oil 
    % collision4= any(overlap4(:)); %with pothole
    collision0= 0; % not overlap or 
    
    if collision3 
        collisionCode2= 1; %player v oil 
    else collision0
        collisionCode2=0; %nothing 
    end 

end 




       





   


%% mapping with masks. only works for one image. 
% hobstacle1= drawpolygon(); 
% mask1=createMask(hobstacle1);%create a logical array so that 1s outline the pond. CreateMask works by making everything else on outside 0s and inside 1s. 
% 
% testmask=imagesc(mask1); %image of the mask 
% colormap([0 0 1]); %blue color over the mask 
% alpha(testmask, 0.3); %cloudy transparency 
% axis image; 
% 
% save('obstacleMask.mat', 'mask1'); 
% disp('mask saved')

%second uncomment 
% load ('obstaclesMask.mat'); %bring in mask1 into code 
% testmask= imagesc(mask1); 
% colorap([1 0 0]); 
% alpha(testmask, 0.3); 
% axis image; 
% disp('Mask Loaded'); 
%% trying another apporahc 

 

function x_next = MovLimit(x_current, x_predict, mask, substeps)

    % Start from current state
    x_next = x_current;

    % Total displacement from RK4
    dx_total=x_predict(1)-x_current(1);
    dy_total=x_predict(2)-x_current(2);

    % Divide into small steps, makes the steps accurate so the moving obj won't skip the wall
    dx=dx_total/substeps;
    dy=dy_total/substeps;

    % Get mask size
    [rows, cols]=size(mask);

    % Step by step movement
    for k=1:substeps

        % Try small move
        x_trial=x_next;
        x_trial(1)=x_trial(1) + dx;
        x_trial(2)=x_trial(2) + dy;

        % Convert to pixel indices
        col=round(x_trial(1));
        row=round(x_trial(2));

        % Boundary check
        if row<1 || row>rows || col<1 || col>cols
            break
        end

        % Mask check
        if mask(row, col) == 1
            x_next = x_trial;
        else
            break % Collision happens and stop moving
        end

    end

    % Keep RK4 velocity only if movement succeeded
    if isequal(x_next(1:2), x_current(1:2))
        x_next(3)=0;
        x_next(4)=0;
    else
        x_next(3)=x_predict(3);
        x_next(4)=x_predict(4);
    end

end
