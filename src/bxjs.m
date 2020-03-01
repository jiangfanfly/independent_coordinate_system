function [bxz,s]=bxjs(x,y,h)
%求解当前坐标系统变形情况
[n,m]=size(x);
R=6371000;
bx=[];
s=[];
for i=1:n
    for j=i+1:n
        s1=sqrt((x(i)-x(j))^2+(y(i)-y(j))^2);
        s=[s;s1];
        ym=(y(i)+y(j))/2;
        hm=(h(i)+h(j))/2;
        bx1=ym^2/(2*(R)^2)*s1-hm/R*s1;
        bx=[bx;bx1];
    end
end
bxz.bxmax=max(bx);
bxz.bxmin=min(bx);
end
