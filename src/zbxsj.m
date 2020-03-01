function varargout = zbxsj(varargin)
% ZBXSJ MATLAB code for zbxsj.fig
%      ZBXSJ, by itself, creates a new ZBXSJ or raises the existing
%      singleton*.
%
%      H = ZBXSJ returns the handle to a new ZBXSJ or the handle to
%      the existing singleton*.
%
%      ZBXSJ('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ZBXSJ.M with the given input arguments.
%
%      ZBXSJ('Property','Value',...) creates a new ZBXSJ or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before zbxsj_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to zbxsj_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help zbxsj

% Last Modified by GUIDE v2.5 05-Jan-2017 14:18:31

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @zbxsj_OpeningFcn, ...
                   'gui_OutputFcn',  @zbxsj_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before zbxsj is made visible.
function zbxsj_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to zbxsj (see VARARGIN)

% Choose default command line output for zbxsj
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes zbxsj wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = zbxsj_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in readkzd.
function readkzd_Callback(hObject, eventdata, handles)
% hObject    handle to readkzd (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[filename,pathname]=uigetfile({'*.xlsx;*.xls','data Files(*.xlsx;*.xls)'},'打开文件');
   if(isempty(pathname))
    returen;
end
filepath=strcat(pathname,filename);
[data,str]=xlsread(filepath,'sheet1','A:F');
n=data(:,1);
x=data(:,4);
y0=data(:,5);
h=data(:,6);
str([1],:)=[];
dname=str(:,2);
dnd=str(:,3);
kzd=strcat(num2str(n),',',num2str(x),',',num2str(y0),',',num2str(h));
set(handles.listbox1,'string',kzd);
y=y0-500000;
[bx,s]=bxjs(x,y,h);
set(handles.edit19,'string',num2str(bx.bxmin));
set(handles.edit20,'string',num2str(bx.bxmax));
ym=mean(y);
Hm=mean(h);
mm=abs((0.00123*(ym/1000)^2-15.7*Hm/1000)*10^(-5));   %当前综合变形
set(handles.edit1,'string',num2str(mm));
handles.bx=bx;
handles.y0=y0;
handles.n=n;
handles.x=x;
handles.y=y;
handles.h=h;
handles.dname=dname;
handles.dnd=dnd;
guidata(hObject,handles);


% --- Executes on selection change in listbox1.
function listbox1_Callback(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox1


% --- Executes during object creation, after setting all properties.
function listbox1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in plot.
function plot_Callback(hObject, eventdata, handles)
% hObject    handle to plot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%绘制变形关系图
n=handles.n;
x=handles.x;
y=handles.y;
h=handles.h;
val=get(handles.popupmenu1,'value'); 
m=get(handles.popupmenu1,'string');
mm=m{val};
m=str2num(mm);
plot(y./1000,h,'*r');
hold on;
y1=linspace(-100,100,100);
y2=linspace(-100,100,100);
H1=((0.00123/15.7)*y1.^2+10^5*m/15.7)*1000;
H2=((0.00123/15.7)*y2.^2-10^5*m/15.7)*1000;
plot(y1,H1,'-.b',y2,H2,'-.b');
hold on;
xlabel('y/km');
ylabel('H/m');
title('长度变形与高程H和横坐标y的关系');
legend('原坐标点','变形边界');


function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%判断使用方法
x=handles.x;
y=handles.y;
ym=mean(y)./1000;
xm=mean(x)./1000;
h=handles.h;
Hm=mean(h);
val=get(handles.popupmenu1,'value'); 
m=get(handles.popupmenu1,'string');
mm=m{val};
m=str2num(mm);
H1=((0.00123/15.7)*ym.^2+10^5*m/15.7)*1000;
H2=((0.00123/15.7)*ym.^2-10^5*m/15.7)*1000;
%判断使用什么方法
if (Hm>H2 && Hm<H1)
    set(handles.text21,'string','不需要建立地方坐标系!');
    msgbox('不需要建立地方坐标系!','提示!');
    ff=0;
    set(handles.pushbutton4,'Enable','off');
    set(handles.edit2,'Enable','off');
    set(handles.edit3,'Enable','off');
else if (abs(ym)<10)
     set(handles.text21,'string','只需采用抵偿高程面作为投影面!');
     msgbox('只需采用抵偿高程面作为投影面!','提示!');
     ff=1;
     set(handles.pushbutton4,'Enable','on');
     set(handles.edit2,'Enable','off');
     set(handles.edit3,'Enable','on');
    else if (Hm<100)
            set(handles.text21,'string','采用自选投影带!');
            msgbox('采用自选投影带!','提示!');
            ff=2;
            set(handles.edit3,'Enable','off');
            set(handles.pushbutton4,'Enable','on');
            set(handles.edit2,'Enable','on');
            set(handles.edit17,'Enable','off');
            set(handles.edit18,'Enable','off');
        else 
            set(handles.text21,'string','采用抵偿高程面以及自选投影带的方法!');
            msgbox('采用抵偿高程面以及自选投影带的方法!','提示!');
            ff=3;
            set(handles.edit3,'Enable','on');
            set(handles.pushbutton4,'Enable','on');
            set(handles.edit2,'Enable','on');
        end
    end
end
handles.ff=ff;
handles.m=m;
guidata(hObject,handles);

% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
xx=handles.xx;
yy=handles.yy;
hh=handles.hh;
plot(yy./1000,hh,'*g');
legend('原坐标点','变形边界(上)','变形边界(下)','新坐标点');
yy=yy+500000;
xy=[xx' yy' hh];
set(handles.listbox6,'string',num2str(xy));
% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%生成报表
dname=handles.dname;
B=rad2dms(handles.B);
L=rad2dms(handles.L);
x=handles.x;
y0=handles.y0;
h=handles.h;
xx=handles.xx;
yy0=handles.yy0;
hh=handles.hh;
bx=handles.bx;
dY=handles.dY;
dc=handles.dc;
ff=handles.ff;

fi=actxserver('Excel.Application');
set(fi,'Visible',1);
wkbs=fi.Workbooks;
Wkb=invoke(wkbs,'Add');
Actsh=fi.Activesheet;
Actsh.Range('A1:I1').ColumnWidth=[16,16,16,16,16,16,16,16,16,16];
%输出坐标结果
A=['局部坐标系设计成果'];
sx=get(handles.text21,'string');
A=strcat(A,'(',sx,')');
actshrng=get(Actsh,'Range','A1','I1');
set(actshrng,'MergeCells',9);
set(actshrng,'HorizontalAlignment',3);
set(actshrng,'Value',A);
bt1=['点名';'纬度';'经度';'原X';'原Y';'原H';'新X';'新Y';'新H'];
bt1=(cellstr(bt1))';
actshrng=get(Actsh,'Range','A2','I2');
set(actshrng,'Value',bt1);
[m,n]=size(x);
R1=strcat('A',num2str(3+m-1));
actshrng=get(Actsh,'Range','A3',R1);
set(actshrng,'Value',dname);
zb=[B',L',x,y0,h,xx',yy0',hh];
R2=strcat('I',num2str(3+m-1));
actshrng=get(Actsh,'Range','B3',R2);
set(actshrng,'Value',zb);
%新坐标系相关参数
A=['新坐标系参数'];
R1=strcat('A',num2str(3+m+1));
R2=strcat('G',num2str(3+m+1));
actshrng=get(Actsh,'Range',R1,R2);
set(actshrng,'MergeCells',7);
set(actshrng,'HorizontalAlignment',3);
set(actshrng,'Value',A);
 A=['投影带宽度/km'];
 R1=strcat('A',num2str(3+m+2));
 actshrng=get(Actsh,'Range',R1,R1);
 set(actshrng,'Value',A);
 R1=strcat('B',num2str(3+m+2));
 actshrng=get(Actsh,'Range',R1,R1);
 set(actshrng,'Value',dY);
  A=['综合平均变形mm/km'];
 R1=strcat('C',num2str(3+m+2));
 actshrng=get(Actsh,'Range',R1,R1);
 set(actshrng,'Value',A);
 R1=strcat('D',num2str(3+m+2));
 actshrng=get(Actsh,'Range',R1,R1);
 set(actshrng,'Value',bx);
switch ff
    case 1
        Hd=handles.Hd;
        A=['H抵:'];
        R1=strcat('E',num2str(3+m+2));
        actshrng=get(Actsh,'Range',R1,R1);
        set(actshrng,'Value',A);
        R2=strcat('F',num2str(3+m+2));
        actshrng=get(Actsh,'Range',R2,R2);
        set(actshrng,'Value',Hd);
    case 2
        L0=handles.L0;
        A=['中央子午线:'];
        R1=strcat('E',num2str(3+m+2));
        actshrng=get(Actsh,'Range',R1,R1);
        set(actshrng,'Value',A);
        R2=strcat('F',num2str(3+m+2));
        actshrng=get(Actsh,'Range',R2,R2);
        set(actshrng,'Value',L0);
    case 3
        L0=handles.L0;
        Hd=handles.Hd;
         A=['H抵'];
        R1=strcat('E',num2str(3+m+2));
        actshrng=get(Actsh,'Range',R1,R1);
        set(actshrng,'Value',A);
        R2=strcat('F',num2str(3+m+2));
        actshrng=get(Actsh,'Range',R2,R2);
        set(actshrng,'Value',Hd);
        A=['中央子午线:'];
        R1=strcat('G',num2str(3+m+2));
        actshrng=get(Actsh,'Range',R1,R1);
        set(actshrng,'Value',A);
        R2=strcat('H',num2str(3+m+2));
        actshrng=get(Actsh,'Range',R2,R2);
        set(actshrng,'Value',L0);
end
A=['源坐标系到设计局部坐标系四参数转换参数'];
R1=strcat('A',num2str(3+m+3));
R2=strcat('G',num2str(3+m+3));
actshrng=get(Actsh,'Range',R1,R2);
set(actshrng,'MergeCells',7);
set(actshrng,'HorizontalAlignment',3);
set(actshrng,'Value',A);
A=['坐标平移X'];
R1=strcat('A',num2str(3+m+4));
actshrng=get(Actsh,'Range',R1,R1);
set(actshrng,'Value',A);
R1=strcat('B',num2str(3+m+4));
actshrng=get(Actsh,'Range',R1,R1);
set(actshrng,'Value',dc(1));
A=['坐标平Y'];
R1=strcat('C',num2str(3+m+4));
actshrng=get(Actsh,'Range',R1,R1);
set(actshrng,'Value',A);
R1=strcat('D',num2str(3+m+4));
actshrng=get(Actsh,'Range',R1,R1);
set(actshrng,'Value',dc(2));
A=['选转参数a'];
R1=strcat('A',num2str(3+m+5));
actshrng=get(Actsh,'Range',R1,R1);
set(actshrng,'Value',A);
R1=strcat('B',num2str(3+m+5));
actshrng=get(Actsh,'Range',R1,R1);
set(actshrng,'Value',dc(3));
A=['尺度参数m'];
R1=strcat('C',num2str(3+m+5));
actshrng=get(Actsh,'Range',R1,R1);
set(actshrng,'Value',A);
R1=strcat('D',num2str(3+m+5));
actshrng=get(Actsh,'Range',R1,R1);
set(actshrng,'Value',dc(4));

dxx=handles.dxx;
dyy=handles.dyy;
sigma=handles.sigma;
A=['坐标转换精度'];
R1=strcat('F',num2str(3+m+4));
actshrng=get(Actsh,'Range',R1,R1);
set(actshrng,'Value',A);
R2=strcat('G',num2str(3+m+4));
actshrng=get(Actsh,'Range',R2,R2);
set(actshrng,'Value',sigma);

% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in listbox3.
function listbox3_Callback(hObject, eventdata, handles)
% hObject    handle to listbox3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox3 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox3


% --- Executes during object creation, after setting all properties.
function listbox3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in listbox4.
function listbox4_Callback(hObject, eventdata, handles)
% hObject    handle to listbox4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox4 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox4


% --- Executes during object creation, after setting all properties.
function listbox4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%四参数转换并进行检验
xx=handles.xx';
yy=handles.yy';
yy0=handles.yy0';
x=handles.x;
y0=handles.y0;

xA1=x([1:5],:);
yA1=y0([1:5],:);
xB1=xx([1:5],:);
yB1=yy0([1:5],:);
A=[xA1,yA1];
B=[xB1,yB1];
[dc]=similar4(A,B);        %求转换参数
set(handles.edit12,'string',num2str(dc(1)));
set(handles.edit13,'string',num2str(dc(2)));
set(handles.edit14,'string',num2str(dc(3)));
set(handles.edit15,'string',num2str(dc(4)));
handles.dc=dc;
guidata(hObject,handles);

function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in listbox2.
function listbox2_Callback(hObject, eventdata, handles)
% hObject    handle to listbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox2


% --- Executes during object creation, after setting all properties.
function listbox2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
xx=handles.xx';
yy0=handles.yy0';
x=handles.x;
y0=handles.y0;
dc=handles.dc;
x([1:5],:)=[];
y0([1:5],:)=[];
A=[x,y0];
xA2=A(:,1);
yA2=A(:,2);
[m,n]=size(A);
u=dc(4)*cos(dc(3));
w=dc(4)*sin(dc(3));
C=[ones(m,1),zeros(m,1),xA2,-yA2;
   zeros(m,1),ones(m,1),yA2,xA2];
dl=[dc(1);dc(2);u;w];
zb=C*dl;
[m,n]=size(A);
xB2=zb(1:m,:);
yB2=zb(m+1:2*m,:);
%求解点位中误差
[m,n]=size(yA2);
xx([1:5],:)=[];
yy0([1:5],:)=[];
dxx=xB2-xx;
dyy=yB2-yy0;
sigma=sqrt((sum(dxx.^2)+sum(dyy.^2))/(m-1));
set(handles.edit4,'string',num2str(sigma*1000));
handles.dxx=dxx;
handles.dyy=dyy;
handles.sigma=sigma;
guidata(hObject,handles);

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%选择投影带
ff=handles.ff;
x=handles.x;
y=handles.y;
h=handles.h;
m=handles.m;
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
%根据所选择的方法进行坐标系设计
switch ff
    case 1
        X2m=mean(x);
        Y2m=mean(y);   %计算平均坐标,确定"大地原点"
        set(handles.edit17,'string',num2str(X2m));
        set(handles.edit18,'string',num2str(Y2m));
        H=Y2m^2/(2*Rm);
        Hd=Hm-H;      %抵偿面至原椭球高程
        hh=h-Hd;      %归化后高程
        set(handles.edit3,'string',num2str(Hd));
         H=Y2m^2/(2*Rm);
        Hd=Hm-H;      %抵偿面至原椭球高程
        hh=h-Hd;      %归化后高程
        set(handles.edit3,'string',num2str(Hd));
        xx=(x+(x-X2m).*Hd./Rm)';           %转换坐标
        yy=(y+(y-Y2m).*Hd./Rm)';
        handles.Hd=Hd;
    case 2
        l=ym/(N*cos(Bm));
        L0=Lm-l;
        set(handles.edit2,'string',num2str(L0));
        %进行换带计算
        B=rad2dms(B);
        L=rad2dms(L);
        [xx,yy]=BL2xy(B,L,L0,n1);
        handles.L0=L0
    case 3
        L0=rad2dms(Lm);   %转换为dms形式
        set(handles.edit2,'string',num2str(L0));
        %进行换带计算
        B=rad2dms(B);
        L=rad2dms(L);
        [X2,Y2]=BL2xy(B,L,L0,n1);
        %计算抵偿面至原椭球高程
        X2m=mean(X2);
        Y2m=mean(Y2);   %计算平均坐标,确定"大地原点"
        set(handles.edit17,'string',num2str(X2m));
        set(handles.edit18,'string',num2str(Y2m));
        H=Y2m^2/(2*Rm);
        Hd=Hm-H;      %抵偿面至原椭球高程
        hh=h-Hd;      %归化后高程
        set(handles.edit3,'string',num2str(Hd));
        xx=X2+(X2-X2m).*Hd./Rm;           %转换坐标
        yy=Y2+(Y2-Y2m).*Hd./Rm;
        handles.X2=X2;
        handles.Y2=Y2;
        handles.Hd=Hd;
        handles.L0=L0;
end
%求解变形长度和投影带宽度
Ye=max(yy);           %求最东位置的点的y坐标
i=find(abs(yy-Ye)<=1e-4);
He=hh(i);
Yw=min(yy);           %求最西位置的点的y坐标
j=find(abs(yy-Yw)<=1e-4);
Hw=hh(j);
Hm=mean(hh);
Ym=mean(yy);
bxze=(Ye*1000)^2/(2*Rm^2)-He*1000/Rm;    %计算最东边变形/mm
bxzw=(Yw*1000)^2/(2*Rm^2)-Hw*1000/Rm;    %计算最西边变形/mm
bx=(Ym*1000)^2/(2*Rm^2)-Hw*1000/Rm;      %计算平均长度变形mm/km
set(handles.edit16,'string',num2str(bx));
set(handles.edit7,'string',num2str(bxze));
set(handles.edit8,'string',num2str(bxzw));
Yemax=(+sqrt((m+(Hm/1000)/Rm)*(2*Rm^2)))/1000;   %计算东边距中央子午线最大宽度/km
Ywmax=(-sqrt((m+(Hm/1000)/Rm)*(2*Rm^2)))/1000;   %计算西边距中央子午线最大宽度/km
dY=Yemax-Ywmax;                            %坐标带宽度/km
set(handles.edit9,'string',num2str(Yemax));
set(handles.edit10,'string',num2str(Ywmax));
set(handles.edit11,'string',num2str(dY));
yy0=yy+500000;
handles.B=B;
handles.L=L;
handles.hh=hh;
handles.xx=xx;
handles.yy=yy;
handles.yy0=yy0;
handles.Yemax=Yemax;
handles.Ywmax=Ywmax;
handles.dY=dY;
handles.bx=bx;
guidata(hObject,handles);

function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in popupmenu2.
function popupmenu2_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu2


% --- Executes during object creation, after setting all properties.
function popupmenu2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in listbox5.
function listbox5_Callback(hObject, eventdata, handles)
% hObject    handle to listbox5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox5 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox5


% --- Executes during object creation, after setting all properties.
function listbox5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in listbox6.
function listbox6_Callback(hObject, eventdata, handles)
% hObject    handle to listbox6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox6 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox6


% --- Executes during object creation, after setting all properties.
function listbox6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in listbox7.
function listbox7_Callback(hObject, eventdata, handles)
% hObject    handle to listbox7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox7 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox7


% --- Executes during object creation, after setting all properties.
function listbox7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit9_Callback(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit9 as text
%        str2double(get(hObject,'String')) returns contents of edit9 as a double


% --- Executes during object creation, after setting all properties.
function edit9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit10_Callback(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit10 as text
%        str2double(get(hObject,'String')) returns contents of edit10 as a double


% --- Executes during object creation, after setting all properties.
function edit10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit11_Callback(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit11 as text
%        str2double(get(hObject,'String')) returns contents of edit11 as a double


% --- Executes during object creation, after setting all properties.
function edit11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit12_Callback(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit12 as text
%        str2double(get(hObject,'String')) returns contents of edit12 as a double


% --- Executes during object creation, after setting all properties.
function edit12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit13_Callback(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit13 as text
%        str2double(get(hObject,'String')) returns contents of edit13 as a double


% --- Executes during object creation, after setting all properties.
function edit13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit14_Callback(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit14 as text
%        str2double(get(hObject,'String')) returns contents of edit14 as a double


% --- Executes during object creation, after setting all properties.
function edit14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit15_Callback(hObject, eventdata, handles)
% hObject    handle to edit15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit15 as text
%        str2double(get(hObject,'String')) returns contents of edit15 as a double


% --- Executes during object creation, after setting all properties.
function edit15_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit16_Callback(hObject, eventdata, handles)
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit16 as text
%        str2double(get(hObject,'String')) returns contents of edit16 as a double


% --- Executes during object creation, after setting all properties.
function edit16_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit17_Callback(hObject, eventdata, handles)
% hObject    handle to edit17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit17 as text
%        str2double(get(hObject,'String')) returns contents of edit17 as a double


% --- Executes during object creation, after setting all properties.
function edit17_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit18_Callback(hObject, eventdata, handles)
% hObject    handle to edit18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit18 as text
%        str2double(get(hObject,'String')) returns contents of edit18 as a double


% --- Executes during object creation, after setting all properties.
function edit18_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit19_Callback(hObject, eventdata, handles)
% hObject    handle to edit19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit19 as text
%        str2double(get(hObject,'String')) returns contents of edit19 as a double


% --- Executes during object creation, after setting all properties.
function edit19_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit20_Callback(hObject, eventdata, handles)
% hObject    handle to edit20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit20 as text
%        str2double(get(hObject,'String')) returns contents of edit20 as a double


% --- Executes during object creation, after setting all properties.
function edit20_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
