x=handles.x;
y=handles.y;
h=handles.h;
L0=str2num(get(handles.edit5,'string'));
n1=get(handles.popupmenu2,'value');
[B,L]=xy2BL(x',y',L0,n1);  %x,y 高斯平面坐标,L0中央经线坐标,n椭球号
ellipsoid=get_ellipsoid(n1); %获取椭球参数
a=ellipsoid.a;
b=ellipsoid.b;
e1=(sqrt(a^2-b^2))/a;
e2=(sqrt(a^2-b^2))/b;
c=a.*sqrt(1+e2^2);
B=dms2rad(B);
L=dms2rad(L);
Hm=mean(h);
ym=mean(y);
Bm=mean(B);          %求纬度的均值
Lm=mean(L);
N=a/(sqrt(1-e1^2*(sin(Bm))^2));   %平均纬度处的卯酉圈半径
Rm=(a*sqrt(1-e1^2))/(1-e1^2*(sin(Bm))^2); %计算测区平均半径
l=7362*sqrt(Hm)/(N*cos(Bm))*10^5;    %经度差(弧度)
L0=Lm-l;        %所需投影带中央经度
L0=rad2dms(L0);  %转换为dms形式
% L0=rad2dms(Lm);
set(handles.edit2,'string',num2str(L0));
%进行换带计算
B=rad2dms(B);
L=rad2dms(L);
[X2,Y2]=BL2xy(B,L,L0,n1);
%计算抵偿面至原椭球高程
X2m=mean(X2);
Y2m=mean(Y2);   %计算平均坐标,确定"大地原点"
set(handles.edit6,'string',num2str(Y2m));
H=Y2m^2/(2*Rm);
Hd=Hm-H;      %抵偿面至原椭球高程
hh=h-Hd;      %归化后高程
set(handles.edit3,'string',num2str(Hd));
xx=X2+(X2-X2m).*Hd./Rm;           %转换坐标
yy=Y2+(Y2-Y2m).*Hd./Rm;
%求解变形长度和投影带宽度
Ye=max(y);           %求最东位置的点的y坐标
i=find(abs(y-Ye)<=1e-4);
He=hh(i);
Yw=min(y);           %求最西位置的点的y坐标
j=find(abs(y-Yw)<=1e-4);
Hw=hh(j);
bxze=(Ye*1000)^2/(2*Rm^2)-He*1000/Rm;    %计算最东边变形/mm
bxzw=(Yw*1000)^2/(2*Rm^2)-Hw*1000/Rm;    %计算最西边变形/mm
set(handles.edit7,'string',num2str(bxze));
set(handles.edit8,'string',num2str(bxzw));
Yemax=(+sqrt((2.5*10^(-5)+He/Rm)*(2*Rm^2)))/1000;   %计算东边距中央子午线最大宽度/km
Ywmax=(-sqrt((2.5*10^(-5)+Hw/Rm)*(2*Rm^2)))/1000;   %计算西边距中央子午线最大宽度/km
dY=Yemax-Ywmax;                            %坐标带宽度/km
set(handles.edit9,'string',num2str(Yemax));
set(handles.edit10,'string',num2str(Ywmax));
set(handles.edit11,'string',num2str(dY));
handles.B=B;
handles.L=L;
handles.Hd=Hd;
handles.hh=hh;
handles.X2=X2;
handles.Y2=Y2;
handles.xx=xx;
handles.yy=yy;
handles.Yemax=Yemax;
handles.Ywmax=Ywmax;
handles.dY=dY;
guidata(hObject,handles);