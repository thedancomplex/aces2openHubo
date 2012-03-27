clear all
close all

%% add to path
addpath('recordAces');
addpath('huboJointConstants');

%% Load constants
huboJointConst

%% Sampling Rate
T = 0.01;

%% initial hubo and world setup
% huboOpenRAVEsetup

%% enable the robot
%orBodyEnable(hubo,1)

%% load the recorded frame data
load record_ThrowR2;

s = size(deg);

tSec 	=  	1.0;
nstep 	= 	tSec/T;
n 	= 	floor(nstep);
da 	= 	[];
d = [];
for( i = 0:(s(1)-2))

	for(j = 1:n)
		ii = i*n+j;
		dv = (deg(i+2,:) - deg(i+1,:))/n;
		dp = deg(i+1,:); 
		da(ii,:) =  dp + dv*j;
		
		for iii = 1:length(mDes)
			da(ii,iii) = da(ii,iii)*orDir(mDes(iii)+1);
		end
	end
	d(i+1,:) = deg(i+1,:);
	for iii = 1:length(mDes)
		d(i+1,iii) = d(i+1,iii)*orDir(mDes(iii)+1);
%%		da(i+1,iii) = da(i+1,iii)*orDir(mDes(iii)+1);
	end
end

mo = {};
for(i = 1:length(mDes))
	ii = mDes(i) + 1;
	
	mo{i} = jn{ii};
end

tname = recordAces(mo,da,'huboThrowR2');
playAces(tname,T);
