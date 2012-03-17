function [ jc, d ] = readAces(tName)

%% reads aces formatted file and retuns the motor numbers and the degree information for each time step
%
% function [ jc, d ] = readAces(tName)
%
% Send:
%	tName	=	string of the path and name to the can logged file
%
% Return:
%	jc	=	
%	t	=	time in seconds

addpath('openHubo/huboJointConstants');
huboJointConst;


disp(['Reading Joint: ', jn{m+1}])

%% read file - h = headder, d = data
[h,d] = hdrload('logs/throwTestR4_feet_still.txt.aces');

%% change headder into cell format
h = textscan(h,'%s');

%% search through all of the recorded joints
jc = [];	% joint columns
for(i = 1:length(h))
	%% get current joint column
	a = h{i};

	for( ii = 1:length(jn) )
		if(length(a) == length(a))
			t1 = a & jn{ii};
			if(sum(t1) == length(a))
				jc(i) = ii -1;	% motor number
			end
		end
	end 
end





end
