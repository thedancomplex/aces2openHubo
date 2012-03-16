%% Set joint constants
huboJointConst

pth = what;

theStr = ['simulation stop ',num2str(T)];

%orEnvSetOptions('simulation stop 0.01');
orEnvSetOptions(theStr);
pause(1.0)
disp('stop simulation');
orEnvLoadScene([pth.path,'/environments/kitchen.env.xml'],1)

td = 0.5;

phy = 0;

if(phy == 1)
	orEnvSetOptions('physics ode');
	pause(td);
	orEnvSetOptions('gravity 0 0 -9.81');
	pause(td);
else
%	orEnvSetOptions('physics none');
	pause(td);
end
orEnvSetOptions('collision ode');
pause(td);
orEnvSetOptions('selfcollision on');
pause(td);

%orEnvLoadScene('/home/dasl/Documents/MATLAB/openHubo/jaemiHubo.robot.xml',1)
%robotid = orEnvCreateRobot('hubo','huboModel/openHubo/jaemiHubo.robot.xml')

bodies = orEnvGetBodies();

%hubo           = bodies{1}.id                          % this is hubo
%hubo            = orEnvGetBody('jaemiHubo')             % this is hubo
hubo            = orEnvGetBody('hubo');             % this is hubo
theFloor 	= orEnvGetBody('floor');
%huboRtHand      = orEnvCreateProblem('right_palm','jaemiHubo')  % manipulator 

%orRobotSetDOFValues(hubo,0.0,0) % set joint 0 to 0.5 rad on robot id hubo
orBodyEnable(hubo,1);
%for(i=1:31)
