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


%disp(['Reading Joint: ', jn{m+1}])

%% read file - h = headder, d = data
[h,d] = hdrload(tName);

%% change headder into cell format
h = textscan(h,'%s');

%% search through all of the recorded joints
jc = [];	% joint columns
for(i = 1:length(h{1}))
	%% get current joint column
	a = h{1};
	a = a{i};
	tt 	=	strcmp(a,jn);
	ti 	=	find(tt == 1);
	jc(i) 	= 	min(ti) - 1;
end





end
