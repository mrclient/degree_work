clear();
data = read("/home/evgeniy/for_ros/my_ws/src/degree_work/config/IP_lin_data.txt", -1, 3);
x0 = data(1,1);
y0 = data(1,2);
for i = 2:max(size(data))
    x(i-1) = data(i,1);
    y(i-1) = data(i,2);
end
Y = y0 - y;
X = x0 - x;
K = (X'*X)^-1 * X' * Y;
angle = atan(K);

for i = 1:((max(size(data))-1)/2)
    first_half_x(i) = x(i);
    last_half_x(i) = x(i+(max(size(data))-1)/2);
end

if mean(first_half_x) > mean(last_half_x) then
    disp("PI was added");
    angle = angle + %pi;
end

fd = mopen('/home/evgeniy/for_ros/my_ws/src/degree_work/config/master_pose.yaml', 'a+');
angle_str = ' - ' + string(angle);
mputl(angle_str, fd);
mclose(fd);
exit();
