function [dout] = filterAces(d,n)


dd = [];
ddd = [];
s = size(d);
for( i = 1:n )
	dd(i,:) = d(1,:);
	ddd(i,:) = d(s(1),:);
end



d = [dd;d;ddd];




dout = [];
s = size(d);
for( i = 1:(s(1)-n) )
	ss = zeros(1,s(2));
	for( j = 1:n )
		ss = ss + d(i+j-1,:);
	end 
	dout(i,:) = ss/n;

end


end
