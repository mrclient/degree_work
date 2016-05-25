clear();
data = read("/home/evgeniy/for_ros/my_ws/src/degree_work/config/IP_rotat_data.txt", -1, 3);
x0 = data(1,1);
y0 = data(1,2);
a0 = 0.0;
for i = 2:max(size(data))
    x(i-1) = data(i,1);
    y(i-1) = data(i,2);
    a(i-1) = -data(i,3);
end
Y = -cos(a) .* (x + y) + sin(a) .* (y - x) + (x0 + y0) * ones(length(x), 1);
X = [cos(a) + sin(a) - ones(length(a)), cos(a) - sin(a) - ones(length(a))];
K = (X'*X)^-1 * X' * Y;
fd = mopen('/home/evgeniy/for_ros/my_ws/src/degree_work/config/master_pose.yaml', 'w');
mputl('master_pose:', fd);
dx = ' - ' + string(-K(1));
mputl(dx, fd);
dy = ' - ' + string(-K(2));
mputl(dy, fd);
mclose(fd);
exit();
