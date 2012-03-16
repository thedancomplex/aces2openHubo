function []	=	can2aces(tName, m)
%% converts vector CANalyzer logs to ACES motion script.
% Note: current assumptions is that all joint's are being sampled at the same
% rate in the logging file.
%
% function []	=	can2aces(tName, m)
%
% Send:
%	tName	=	string of the path and name to the can logged file
%	m	=	array of the motors you are interested in
%
% Ouput:
%	tName.aces	=	will add file to logging director with the 
%				extention of *.aces added

addpath('huboJointConstants');
huboJointConst;

mot 	=	[];
tFlag 	=	0;
for ( i = 1:length(m) )
	[d, t] = readCan2(tName, m(i));
	

	ss = size(mot);
	dd = d;
	if (ss(1) > length(dd))
		for ( ii = length(dd):ss(1) )
			dd(ii) = dd(length(dd));
		end
	end


	if ( tFlag == 1 & ss(1) < length(dd) )
		for( ii = ss(1):length(dd) )
			mot(ii,:) = mot(ss(1),:);
		end
	end
%	disp(['size mot = ',num2str(size(mot))])
%	disp(['size dd  = ',num2str(size(dd))])
	mot(:,i) 	= 	dd;
	tFlag 		= 	1;
	if( m(i) == RWY)
		save mv2.mat dd;
	end
end

%% Convert to rad
mot 	=	degtorad(mot);		


fid 	=	fopen([tName,'.aces'],'w');

%% print headder
for( i = 1:length(m) )
	fprintf(fid,[jn{m(i)+1},' ']);
end	

%% print new line
fprintf(fid,'\n');

ss = size(mot);
for( i = 1:ss(1) )		%% Each row
	for ( ii = 1:ss(2) )	%% each element in the row
		a = mot(i,ii);
		%fprintf(fid, [num2str(mot(i,ii)), ' '] );
		fprintf(fid, [num2str(a,'%5.15f'), ' '] );
	end
	fprintf(fid,'\n');
end

fclose(fid);

save mv3.mat mot m
end

