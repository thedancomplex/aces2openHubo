clear all
close all

%% add to path
addpath('recordAces');
addpath('huboJointConstants');
%% dan test
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

deg(s(1)+1, : ) = deg(1,:);

s = size(deg);


rat 	= 	1.3;	% 7m/s ~ 5m@45deg
rat 	= 	1.8;	% 3.6m/s ~ 1.3m@45deg
rat	=	4.0;	
%% index	  1-2 2-3  3-4   4-5   5-6   6-7 7-1
tSec 	=  	[1.0, 0.5, 0.1, 0.06, 0.06, 0.3, 1.0]*rat;
nstep 	= 	tSec/T;
n 	= 	floor(nstep);
da 	= 	[];
d = [];
ih = 1;

for( i = 0:(s(1)-2))

	for(j = 1:n(i+1))
		ii = ih;
		dv = (deg(i+2,:) - deg(i+1,:))/n(i+1);
		dp = deg(i+1,:); 
		da(ii,:) =  dp + dv*j;
		
		for iii = 1:length(mDes)
			da(ii,iii) = da(ii,iii)*orDir(mDes(iii)+1);
		end
		ih = ih+1;
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

for ( i = 1:length(mo) )
	t = strcmp(jn,mo{i});
	t = min(find(t==1));
	deg(:,i) = deg(:,i)*orDir(t);
end

da = filterAces(da,3);



mo{length(mo)+1} = 'NKY';

i = strcmp(mo,'WST');
i = min(find(i==1));

da(:,s(2)+1) = da(:,i)+0.8;

shortName =	recordAces(mo,deg,'huboThrowSteps');

desName = 'huboThrow';
tname = recordAces(mo,da,desName);



d1 = zeros(1,s(2));
d2 = deg(1,:);
d3 = d2;
n = 200;
setupDa = [];
for (i = 1:n)
	setupDa(i,:) = (d2-d1)/n*i;
end



setupName = recordAces(mo,setupDa,[desName,'.setup']);


disp(['Ouput Files:'])
disp(['            Trajectory - ',tname]);
disp(['            Zero Setup - ',desName]);

velot = playAces2(tname,T,3,1);
v = sum((velot.^2)');
figure
plot((1:length(v))*T,v);
xlabel('Time (sec)');
ylabel('Speed (m/sec)');
title('speed graph of right hand in reference to right foot');

figure
plot(da)
xlabel('Time (sec)')
ylabel('Pos (rad)')
title('position of all joints')

figure
dda = diff(da)/T;
plot(dda);
xlabel('Time (sec)')
ylabel('Velocity (rad/sec)');
title(['Velocity of all joints at Multiplyer = ',num2str(rat)]);

figure
ddda = diff(dda)/T;
plot(ddda);
xlabel('Time (sec)')
ylabel('Accelleration (rad/sec^2)');
title(['Accelleration of all joints at Multiplyer = ',num2str(rat)]);
