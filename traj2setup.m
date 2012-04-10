function [ hh, dd ] = traj2setup(h,d,n) 
%%function [ hh, dd ] = traj2setup(h,d,n) 
%	Takes a headder and a data set then moves each joint from zero 
% 	to the first pose in the set in n steps
%
% Send:
%
%	h 	=	headder data
%	d	=	data d(:,motorNum)
%	n	=	number of steps 
%
% Return:
%	hh	=	headder data
%	dd 	=	traj for each joint from 0 to first pose in n steps

s = size(d);

d1 = zeros(1,s(2));
d2 = d(1,:);

dd = [];

for( i = 1:n)
	dd(i,:) = (d2-d1)/n*i;
end

hh = h;

end
