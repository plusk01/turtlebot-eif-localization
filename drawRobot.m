function handle = drawRobot( animate, xt, handle)
%DRAWROBOT Draws a simple Thrunbot
%   inputs:
%       xt: input state vector (x, y, theta)
%   outputs:
%       handle: the figure handle for the robot

    if nargin == 1 || isempty(handle)
        handle_circle = [];
        handle_line = [];
    else
        % Unpack the handles
        handle_circle = handle(1);
        handle_line = handle(2);
    end

    r = 0.5; % this comes from how we did the meshgrid when plotting with
             % `pcolor`. Since we start in the cetner of the square cell of
             % width 1, radius must be 0.5
    
    % Unpack the robot's states
    x = xt(1);
    y = xt(2);
    theta = xt(3);
            
    handle_circle = circle(x, y, r, handle_circle);
    handle_line = heading(x, y, r, theta, handle_line);
    
    if animate == 1
        drawnow; % draw the entire robot together
    end
    
    handle = [handle_circle handle_line];
end

function handle = circle(x,y,r, handle)
%x and y are the coordinates of the center of the circle
%r is the radius of the circle
%0.01 is the angle step, bigger values will draw the circle faster but
%you might notice imperfections (not very smooth)
    ang = 0:0.01:2*pi; 
    xp = r*cos(ang);
    yp = r*sin(ang);
    
    if isempty(handle)
        handle = plot(x+xp,y+yp, 'Color', [0 0.4470 0.7410]);
    else
        set(handle,'XData',x+xp,'YData',y+yp);
    end
end

function handle = heading(x,y,r,theta, handle)
    x2 = x + r*cos(theta);
    y2 = y + r*sin(theta);
    
    if isempty(handle)
        handle = plot([x x2],[y y2], 'Color', [0.8500 0.3250 0.0980]);
    else
        set(handle,'XData',[x x2],'YData',[y y2]);
    end
end