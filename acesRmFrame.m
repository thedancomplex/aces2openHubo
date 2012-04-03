function [h,d] = acesRmFrame(h, d)

addpath('openHubo/huboJointConstants');
huboJointConst;

i = find( h == 99 );

im = i-1;
iM = i+1;

L = length(h);
r1 = [];
r2 = [];
if (iM < length(h))
	r1 = 1:im;
	r2 = im:L;
else
	r1 = 1:im;
	r2 = [];
end




h = [h(r1),h(r2)];
d = [d(:,r1),d(:,r2)];
end
