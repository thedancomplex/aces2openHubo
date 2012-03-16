close all
clear all

%%i setup initial hubo and world in openRAVE
huboOpenRAVEsetup

%% sampling rate
T = 0.01;

%% time start   
t = 0



%d = 1;

figure
title('Throw Streight');
hold on;

%load huboLogWithJointConstraintsBIG;
%xxxd = x;
%yyyd = y
ii = 1;

n = 100000;
ni = 1;
xyFinal = zeros(n,2);
c = [];
theGo = 1;

orBodyEnable(hubo,1)

jLim 	= 	orRobotGetDOFLimits(hubo);		% Joint Limits
jLim 	=	0.40*jLim;
while(theGo == 1)

	di 	= 	[ RSP, RSR, RSY, REB  ]';
	ddi	=	di + 1;	

%	jLim 	= 	orRobotGetDOFLimits(hubo);		% Joint Limits
	a 	=	jLim;	
	sa	=	size(a);
	rrand	=	rand(sa(1),1);
	rdiff	=	abs(a(:,1)-a(:,2));
	rmin	=	min(jLim')';
	d	=	rrand(ddi).*rdiff(ddi)+rmin(ddi,1);

	aDOF = orRobotGetDOFValues(hubo);

	orRobotSetDOFValues(hubo,d, di);
	
	%save huboLog.mat
	
	orEnvStepSimulation(T,1);
	envTimeOut = orEnvWait(hubo,5);
	%c(ii) = orRobotCheckSelfCollision(hubo);
	orBodyEnable(hubo,1)

	c = orRobotCheckSelfCollision(hubo);
	%ii = ii+1;
	v = orRobotGetDOFValues(hubo);
	
	if( c == 0 )
		dddi 	= [di; 21];
		L 	= orBodyGetLinks(hubo);
		%TT 	= L(:,RWP+1);
		TT 	= L(:,dddi);
		st	= size(TT);
		T1 	= [reshape(TT(:,1),[3,4]); 0 0 0 1] ;		%% this forms a 3x4 matrix the first 3x3 is rotation the next 3x1 is translation
		T2 	= [reshape(TT(:,2),[3,4]); 0 0 0 1] ;		%% this forms a 3x4 matrix the first 3x3 is rotation the next 3x1 is translation
		T3 	= [reshape(TT(:,3),[3,4]); 0 0 0 1] ;		%% this forms a 3x4 matrix the first 3x3 is rotation the next 3x1 is translation
		T4 	= [reshape(TT(:,4),[3,4]); 0 0 0 1] ;		%% this forms a 3x4 matrix the first 3x3 is rotation the next 3x1 is translation
		T5 	= [reshape(TT(:,5),[3,4]); 0 0 0 1] ;		%% this forms a 3x4 matrix the first 3x3 is rotation the next 3x1 is translation

		%Tt	= T1*T2*T3*T4*T5;
		Tt	= T5;
		xyz	= Tt(:,4);
		x	= xyz(1);
		y	= xyz(2);
		z	= xyz(3);
		orEnvPlot([x,y,z],'size',5);
		
		goodXYZ(ii,:) = [x, y, z];
		ii = ii+1;
		save goodXYZmat040.mat goodXYZ

	end
	
	
	%tt1 = v(di(1));
	%tt2 = v(di(2));

	%xxx = a1*cos(tt1)+a2*cos(tt1+tt2);
	%yyy = a1*sin(tt1)+a2*sin(tt1+tt2);

	%plot(xxx,yyy,'k*');
end


%%%orEnvSetOptions('simulation start')
disp('joints set')
