close all
clear all

%% Add paths
addpath('huboJointConstants');

%% sampling rate
T = 0.01;
%%i setup initial hubo and world in openRAVE
huboOpenRAVEsetup

%% Load paramaters
huboJointConst


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
try
	str = input('aces cmd> ','s');
	[ m, d, cmd] = console2hubo(str);
	
	if(strcmp(cmd,'exit'))
		theGo = 0;
		disp('Exiting');
	elseif(strcmp(cmd,'help'))
		disp('Help Menu:');
		disp('cmd	=	action');
		disp('------------------------');
		disp('exit 	=	exit program');
		disp('rmmat 	=	delete record.mat file');
		disp('record 	=	record current joint values to record.mat');
		disp('jNum jDeg	=	joint number and deg where joint number example RSP 1.43');
	elseif(strcmp(cmd,'rmmat'))
		disp('Delete record.mat???  yes or no');
		str = input('aces cmd> ','s');
		if(strcmp(str,'yes'))
			delete record.mat;
			disp('record.mat deleated');
		else
			disp('No Action Taken')
		end
	elseif(strcmp(cmd,'record'))
		disp('Recording to record.mat');
		mDesTR 	= [ RSP, RSR, RSY, REB, RWY, RWP ];
		mDesTL 	= [ LSP, LSR, LSY, LEB, LWY, LWP ];
		mDesBR 	= [ RHP, RHR, RHY, RKN, RAP, RAR ];
		mDesBL 	= [ LHP, LHR, LHY, LKN, LAP, LAR ];
		mDesM  	= [ WST ];
		mDes 	= [ mDesTR, mDesTL, mDesBR, mDesBL, mDesM ];

		degT = orRobotGetDOFValues(hubo,mDes);
		iRec = 1;
		deg = [];
		try
			load record.mat;
			iRec = iRec + 1;
		catch
			disp('No record.mat creating new file');
		end
		deg(iRec,:) = degT;
		save record.mat mDes iRec deg;
	else
			%% set dof values
			deg = d;
			deg = deg.*orDir(m);
			M = m-1;
			disp(['** Setting ',jn{m},' to ',num2str(d), ' rad with dir of ',num2str(orDir(m))]);
			orRobotSetDOFValues(hubo,deg, M);
	
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
catch
	disp('!!!input error try again!!!');
end
end


%%%orEnvSetOptions('simulation start')
disp('joints set')
