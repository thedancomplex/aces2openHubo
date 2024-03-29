function [] = playAces(tname,T)

%% sampling rate
%T = 0.01;
%%i setup initial hubo and world in openRAVE
huboOpenRAVEsetup


%% time start   
t = 0;


%% enable the robot
orBodyEnable(hubo,1)

%% Load aces file
[jc, dd] = readAces(tname);
%[jc, dd] = readAces('jTest.aces');
sAces = size(dd);
d = dd(:,1:(sAces(2)));


%% the joints used
di 	= 	jc(1:(length(jc)));
ddi	=	di + 1;	

%% get joint lim
jLim 	= 	orRobotGetDOFLimits(hubo);		% Joint Limits
aDOF = orRobotGetDOFValues(hubo);

%% colision matrix
co = [];

%% the while loop
theGo = 1;

%while(theGo == 1)

for ( i = 1:sAces(1))		% go over whole trajectory
	%% set dof values
	deg = d(i,:);
	deg = deg.*orDir(ddi);

	orRobotSetDOFValues(hubo,deg, di);
	
	%% step simulation
	orEnvStepSimulation(T,1);
	envTimeOut = orEnvWait(hubo,5);
	orBodyEnable(hubo,1)
	
	%% check for collisions ( 0 = no colisiions, 1 = yes)
	c = orRobotCheckSelfCollision(hubo);
	
	%% get current values of hubo
	v = orRobotGetDOFValues(hubo);
	

	ci = length(co) + 1;
	if( c == 0 )
		%% do here if there is no colision
		co(i) = 0;
	else
		co(i) = 1;
	end
end


%%%orEnvSetOptions('simulation start')
disp('joints set')
