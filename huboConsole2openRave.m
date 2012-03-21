close all
clear all

%% sampling rate
T = 0.01;
%%i setup initial hubo and world in openRAVE
huboOpenRAVEsetup


%% time start   
t = 0;


%% enable the robot
orBodyEnable(hubo,1)

%% get joint lim
jLim 	= 	orRobotGetDOFLimits(hubo);		% Joint Limits
aDOF = orRobotGetDOFValues(hubo);

%% the while loop
theGo = 1;

disp('Set OpenHubo Joint Value (rad)')
while(theGo == 1)

	str = input('aces cmd> ','s');
	[ m, d, cmd] = console2hubo(str);
	
	if(strcmp(cmd,'exit'))
		theGo = 0;
		disp('Exiting');
	elseif(strcmp(cmd,'record'))
		disp('Recording to record.aces');
	else
		%% set dof values
		deg = d;
		deg = deg.*orDir(m);

		orRobotSetDOFValues(hubo,deg, m-1);
	
		%% step simulation
		orEnvStepSimulation(T,1);
		envTimeOut = orEnvWait(hubo,5);
		orBodyEnable(hubo,1)
	
		%% check for collisions ( 0 = no colisiions, 1 = yes)
		c = orRobotCheckSelfCollision(hubo);
	
		%% get current values of hubo
		v = orRobotGetDOFValues(hubo);
	
		if( c == 0 )
			%% do here if there is no colision
			temp = 0;
		else
			disp('Self-Collision');
		end
	end
end


%%%orEnvSetOptions('simulation start')
disp('joints set')
