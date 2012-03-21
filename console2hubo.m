function [ m, d, cmd] = console2hubo(str)

addpath('openHubo/huboJointConstants');
huboJointConst

m = 0;
d = 0;

try
	a = textscan(str, '%s%f');
	cmd = a{1};
	d = a{2};
	m = min(find(strcmp(jn,cmd) == 1));
catch exception
	m = -1;
	d = -1;
end

end 
