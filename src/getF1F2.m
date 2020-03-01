function[F1,F2]=getF1F2(a,e2,Bi,Li)
c=a.*sqrt(1+e2);
t=tan(Bi);
ng=sqrt(e2.*cos(Bi).^2);
V=sqrt(1+ng.^2);
N=c./V;
a1=N.*cos(Bi);
a2=0.5.*N.*cos(Bi).*sin(Bi);
a3=N.*cos(Bi).^3.*(1-t.^2+ng.^2)./6;
a4=N.*sin(Bi).*cos(Bi).^3.*(5-t.^2+9.*ng.^2+4.*ng.^4)./24;
a6=N.*sin(Bi).*cos(Bi).^5.*(61-58.*t.^2+t.^4)./720;
b0=1-0.75.*e2+(45.0/64).*e2.^2-(175.0/256).*e2^3+(11025.0/16384).*e2.^4.0;
b2=b0-1;
b4=15.*e2^2./32-175.*e2^3./384+3675.*e2^4./8192;
b6=-35.*e2^3./96+735.*e2^4./2048;
b8=315.*e2^4./1024;
F1=(c.*b2+(c.*b4+(c.*b6+c.*b8.*cos(Bi).^2).*cos(Bi).^2).*cos(Bi).^2.0).*sin(Bi).*cos(Bi);
F2=a2.*Li.*Li+a4.*Li.^4.0+a6.*Li.^6.0;
end