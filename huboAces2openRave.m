close all
clear all

%%i setup initial hubo and world in openRAVE
huboOpenRAVEsetup

%% sampling rate
T = 0.01;

%% time start   
t = 0


%% enable the robot
orBodyEnable(hubo,1)

%% the joints used
di 	= 	[ RSP, RSR, RSY, REB  ]';
ddi	=	di + 1;	

%% get joint lim
jLim 	= 	orRobotGetDOFLimits(hubo);		% Joint Limits
aDOF = orRobotGetDOFValues(hubo);

%% colision matrix
co = [];

%% the while loop
theGo = 1;
while(theGo == 1)

	%% set dof values
	orRobotSetDOFValues(hubo,d, di);
	
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
