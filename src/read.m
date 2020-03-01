[filename,pathname]=uigetfile({'*.xlsx;*.xls','data Files(*.xlsx;*.xls)'},'打开文件');
   if(isempty(pathname))
    returen;
end
filepath=strcat(pathname,filename);
[data,str]=xlsread(filepath,'sheet1','A:F');
n=data(:,1);
x=data(:,4);
y=data(:,5);
h=data(:,6);
str([1],:)=[];
dname=str(:,2);
dnd=str(:,3);
ym=mean(y);
Hm=mean(h);
mm=(0.00123*ym^2-15.7*Hm)*10^5;   %当前综合变形

%绘制变形关系图
plot(y./1000,h,'*r');
hold on;
y1=linspace(0,500,100);
y2=linspace(0,500,100);
H1=((0.00123/15.7)*y1.^2+10^5*m/15.7)*1000;
H2=((0.00123/15.7)*y2.^2-10^5*m/15.7)*1000;
plot(y1,H1,'-.b',y2,H2,'-.b');

%选择投影带
[B,L]=xy2BL(x,y,L0,n);  %x,y 高斯平面坐标,L0中央经线坐标,n椭球号
B=dms2rad(B);
L=dms2rad(L);
Bm=mean(B);          %求纬度的均值
Lm=mean(L);
N=a/(sqrt(1+e1^2*(cosBm)^2));   %平均纬度处的卯酉圈半径
Rm=(a*sqrt(1-e^2))/(1-e^2*(sinB)^2);
l=ym/N*cosBm;    %经度差(弧度)
L0=L-l;        %所需投影带中央经度
L0=rad2dms(L0);  %转换为dms形式

%进行换带计算
L0=dms2rad(L0);
[B,L]=xy2BL(x',y',l,n1);
[X2,Y2]=BL2xy(B,L,L0,n1);
X2m=mean(X2);
Y2m=mean(Y2);   %计算平均坐标,确定"大地原点"
H=0.785*(Y2/1000)^2;
Hd=Hm-H;
xx=x+(x-x0).*Hd./R;
yy=y+(y-y0).*Hd./R;





%判断使用方法
H1=((0.00123/15.7)*ym.^2+10^5*m/15.7)*1000;
H2=((0.00123/15.7)*ym.^2-10^5*m/15.7)*1000;
if H2<h<H1
    msgbox('提示!','不需要建立地方坐标系');
else if (ym/1000)<10
     msgbox('提示!','采用抵偿高程面作为投影面');
    else if (Hm<10)
            msgbox('提示!!','采用自选投影带');
        else 
            msgbox('提示','采用抵偿高程面以及自选投影带的方法');
        end
    end
end
