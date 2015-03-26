N = 500;
t = 0:0.01:0.01*(N-1);
l_leg = t*0;
dl_leg = l_leg;
ddl_leg = l_leg;

initial_position = 0.70;
final_position = 0.91;
max_v = 1;
max_a = 15;
t_trajectory = 0.1;

for i=1:N
    [l_leg(i), dl_leg(i), ddl_leg(i)] = LegLengthTrajectory(t(i), initial_position, final_position, t_trajectory, max_v, max_a);
end

figure(203);
subplot(311);
plot(t,l_leg)
subplot(312);
plot(t,dl_leg)
subplot(313);
plot(t,ddl_leg)