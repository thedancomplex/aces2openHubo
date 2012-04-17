function [theOut] = playAces2(tname,T,opt,vid)
%%function [theOut] = playAces2(tname,T,opt,vid)
% Plays aces file at time step and returns paramater from opt
% 
% Send:
%	tname 	=	Name of aces file
%	T	=	period (hubo = 0.01) in sec
%	opt 	=	Option
%			0:	None
%			1: 	return transformation matrix betwen Rt Foot
%				and the right hand
%			2:	returns collision velcot, 1 if there is a
%				at that time step 0 if there is none
%			3:	velos [Vx, Vy, Vz] of right hand in reference
%				to the right foot
%			4: 	Record New Files
%			5:	Enables openrave, required for all other
%				return functionality
%	vid 	=	if vid==1 pause to record video
%
% Return:
%	theOut	=	output dependent on opt
	

%% init vlaues
theOut 	=	[];

%% sampling rate
%T = 0.01;
%%i setup initial hubo and world in openRAVE
huboOpenRAVEsetup;

addpath('recordAces');
%% time start   
t = 0;


%% enable the robot
orBodyEnable(hubo,1)

%% Load aces file
[jc, dd] = readAces(tname);
[jc, dd] = acesRmFrame(jc,dd);
[jc, dd] = acesOffsets(jc,dd);
[jc, dd] = acesFilter(jc,dd,10);	%filter to smooth things out
%[jc, dd] = rightHandMotion(jc,dd,120);	% release hand at 100
%[jc, dd] = readAces('jTest.aces');
sAces = size(dd);
%d = dd(:,1:(sAces(2)));
d = dd;



%% record new files
for( i = 1:length(opt))
if( opt(i) == 4 )
	m = {};
	for( i = 1:length(jc) )
		m{i} = jn{jc(i)+1};
	end
	 
	recordName = recordAces(m, d, [tname,'.main']);
	disp(['Main File Recorded:  ',recordName]);

	[ m, dsetup] = traj2setup(m,d,200);	%% get to fist pose in 2 seconds

	setupName = recordAces(m, dsetup,[tname,'.setup']);
	disp(['Setup File Recorded: ',setupName]);
end
end

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

%% vars for velos calc
x0 = [];
x1 = [];
vel = [];
Tout = {};

if(vid == 1)
	input('Set Video Settings then press ENTER');
end

theGo = find(opt == 5);

if( theGo > 0 )
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
	
%% Get Colisions
	ci = length(co) + 1;
	if( c == 0 )
		%% do here if there is no colision
		co(i) = 0;
	else
		co(i) = 1;
	end
%% Get Velos
	L = orBodyGetLinks(hubo);
	RH = 21;	% right hand shell
	RF = 36;	% right foot
	Trh = L(:,RH);	% get only the right hand pos and rot
	Trf = L(:,RF);	% get only the right foot pos and rot

	Trh = [reshape(Trh,[3,4]); 0 0 0 1]; 	% convert to square matrix
	Trf = [reshape(Trf,[3,4]); 0 0 0 1];	% convert to square matrix
	Tf = Trh*Trf;

	for( j = 1:length(opt))
	switch opt(j)
		case 2,
			theOut = co;
		case 1,
			Tout{i} = Tf;
			theOut = Tout;
		case 3, 
			if( i == 1 )
				x0 = [ Tf(1,4), Tf(2,4), Tf(3,4)];
			end
			
			x1 = x0;
			x0 = [ Tf(1,4), Tf(2,4), Tf(3,4)];
			f = x0 - x1;
			vel(i,:) = f/T;
			theOut = vel;
		end
	end

end
end

%%%orEnvSetOptions('simulation start')
disp('joints set')
