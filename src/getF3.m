function[F3]=getF3(a,e2,Bi,Li)
c=a.*sqrt(1+e2);
t=tan(Bi);
ng=sqrt(e2.*cos(Bi).^2);
V=sqrt(1+ng.^2);
N=c./V;
a3=N.*cos(Bi).^3.*(1-t.^2+ng.^2)./6;
a5=N.*cos(Bi).^5.*(5-18.*t.^2+t.^4+14.*ng.^2-58.*t.^2.*ng.^2)./120;
F3=a3.*Li.^3+a5.*Li.^5;
end