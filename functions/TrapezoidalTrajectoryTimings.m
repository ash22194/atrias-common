% Optimal trapezoidal velocity trajectory
function [tr, th] = TrapezoidalTrajectoryTimings(t_trajectory, max_acceleration, max_velocity, target_distance)

tt = t_trajectory;
a = max_acceleration;
v = max_velocity;
ls = target_distance;

% possible solutions
tr = [	tt/2;
	tt/2;
	(ls/a)^(1/2);
	v/a;
	0;
	tt/2 - (-(4*ls - a*tt^2)/a)^(1/2)/2;
	0;
	v/a;
	(-ls/a)^(1/2);
	v/a;
	-(ls/a)^(1/2);
	v/a;
	v/a;
	tt/2 + (-(4*ls - a*tt^2)/a)^(1/2)/2;
	v/a;
	-(-ls/a)^(1/2);
];

th = [	0;
	0;
	0;
	-(2*v - a*tt)/a;
	0;
	(-(4*ls - a*tt^2)/a)^(1/2);
	tt;
	-v/a;
	-2*(-ls/a)^(1/2);
	0;
	0;
	(a*ls - v^2)/(a*v);
	-(2*v - a*tt)/a;
	-(-(4*ls - a*tt^2)/a)^(1/2);
	0;
	2*(-ls/a)^(1/2);
];

tr = real(tr);
th = real(th);

% test constraints
c1 = -a.*tr.^2-a.*th.*tr+ls >= 0;
c2 = -2*tr-th+tt >= 0;
c3 = -a*tr+v >= 0;
c4 = tr >= 0;
c5 = th >= 0;
constraints = c1 & c2 & c3 & c4 & c5;

% objective function to maximize
f = -(a.*tr.^2 + a.*th.*tr - ls).^2;
satisfied = find(constraints);
total_times = 2*tr + th;
[M,~] = max(f(satisfied));
I = satisfied(f(satisfied) == M);
[~, I2] = min(total_times(I));

tr = tr(I(I2));
th = th(I(I2));

if isempty(tr) || isempty(th), tr = 0; th = 0; end


