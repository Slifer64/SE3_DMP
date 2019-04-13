function t_frame = get_frame_handle(varargin)
    scale = 1;
    color_intensity = 1;
    
    if (nargin > 0)
        scale = varargin{1};
    end
    
    if (nargin > 1)
        color_intensity = varargin{2};
    end
    
    
    x_axis_color = color_intensity * [0 1 0];
    y_axis_color = color_intensity * [0 0 1];
    z_axis_color = color_intensity * [1 0 0];
    frame_origin_color = color_intensity * [1 1 0];
   
    r = 0.05;
    [X, Y, Z] = cylinder(r);
    Z(2,:) = 1;
    X=scale*X; Y=scale*Y; Z=scale*Z;
    Xx=Z; Xy=X; Xz=Y;
    Yx=Y; Yy=Z; Yz=X;
    Zx=X; Zy=Y; Zz=Z;

    [Cx, Cy, Cz] = sphere();
    sr = scale * 1.6 * r;
    Cx=sr*Cx; Cy=sr*Cy; Cz=sr*Cz;

    h(1) = surface(Xx,Xy,Xz,'FaceColor',x_axis_color);       % x-axis
    h(2) = surface(Yx,Yy,Yz,'FaceColor',y_axis_color);       % y-axis
    h(3) = surface(Zx,Zy,Zz,'FaceColor',z_axis_color);       % z-axis
    h(4) = surface(Cx,Cy,Cz,'FaceColor',frame_origin_color); % frame origin
    
    t_frame = hgtransform();
    set(h,'Parent',t_frame);
end