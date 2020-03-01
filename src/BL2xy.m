function[x,y]=BL2xy(B,L,L0,n)
%�������ת��Ϊ��˹����ĺ���
%B,LΪ�������,����������L0
%nΪ������,n=1:��������˹������;n=2,IUGG1975����;n=3WGS84����;n=4:CGCS2000����
%x,y���Ϊ��˹ƽ������
B=dms2rad(B);
L=dms2rad(L);
L0=dms2rad(L0);
ellipsoid=get_ellipsoid(n); %��ȡ�������
a=ellipsoid.a;
b=ellipsoid.b;
e=(sqrt(a^2-b^2))/a;
e1=(sqrt(a^2-b^2))/b;
X=get_X(B,a,b,e1);
V=sqrt(1+(e1.^2).*cos(B).^2);
c=(a.^2)/b;
M=c./(V.^3);
N=c./V;
t=tan(B);
n=sqrt((e1.^2).*(cos(B)).^2);
l=L-L0;
xp1=X;
xp2=(N.*sin(B).*cos(B).*l.^2)./2;
xp3=(N.*sin(B).*((cos(B)).^3).*(5-t.^2+9.*n.^2+4.*n.^4).*l.^4)./24;
xp4=(N.*sin(B).*((cos(B)).^5).*(61-58.*t.^2+t.^4).*l.^6)./720;
x=xp1+xp2+xp3+xp4;
yp1=N.*cos(B).*l;
yp2=N.*(cos(B)).^3.*(1-t.^2+n.^2).*l.^3./6;
yp3=N.*(cos(B)).^5.*(5-18.*t.^2+t.^4+14.*n.^2-58.*(n.^2).*(t.^2)).*l.^5./120;
y=yp1+yp2+yp3;
end
