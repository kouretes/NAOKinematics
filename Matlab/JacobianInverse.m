function [thetas]=JacobianInverse(Target,forward,initial)

initial
alpha=0.8;
CLAMP=10;
MAXITER=50;
Mask=ones(4,4);
Mask(1:3,4)=1;


[T s Derivatives]=forward(initial);
[k l m]=size(Derivatives);
%k chain size
%l,m = 4
J=zeros(k,(l)*m);
TOL=1e-2;
initial=initial(1:k)
pause

quality=sum(sum((abs(Target-T).*Mask)));
ITER=0;

while((quality>TOL && ITER<=MAXITER)|| ITER==0)
	ITER=ITER+1;
	tplot(ITER,:)=initial;

	[T s Derivatives]=forward(initial);
	quality=sum(sum((abs(Target-T).*Mask)));
	%alpha
	quality
	

	#alpha=0.2
	%Target
	%T1
	e=(Target-T).*Mask;
	e=e(:);
	%e=e(1:end-4);
	m=max(abs(e));
	if(m>CLAMP)
		e=(e./m).*CLAMP;
	end
	
	for i=1:k		
	 p=squeeze(Derivatives(i,:,:));%apla p = Derivatives(i,:,:) san return
	 p=p(:);
	 %p=p(1:end-4);
	 J(i,:)=p;
	end
	%s=sign(J'*e);
	%g=s*alpha*(abs(J'*e))^0.9;
	disp('--')
	g=alpha*pinv(J')*e
	%g=alpha*J*e;
	s=abs(g)<1e-7;
	if(s)
		g(s)=sign(g(s)).*(abs(g(s))).^0.6;
	end
	%g=(2./(1+exp(-g))-1)
	
	initial=initial+g;%.*[0 1]'	
	
	
	
	%pause
end

ITER
[T s Derivatives]=forward(initial);
Target-T
plot(tplot);
thetas=initial
end

