function points = gen_corner(center, r, start_ang, end_ang, n_points)
    theta = linspace(start_ang, end_ang, n_points);
    points = [center(1) + r*cos(theta)', center(2) + r*sin(theta)'];
end
