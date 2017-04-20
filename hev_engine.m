%% Engine Dynamics Approximation First Order
syms y(t)

T = 0.2;
Pe_star = 330000;

eqn = diff(y,t) == (-y + Pe_star)/T;
cond = y(0) == 0;
ysol(t) = dsolve(eqn,cond);

fplot(ysol/1000,[0 10])
xlabel('Time [s]');
ylabel('P_{eng} [kW]')
grid on;
%% Engine Dynamics Approximation Second Order
syms y(t)

wn = 2*pi*1.4;
wd = 2*pi*1;
xi = sqrt(1-(wd/wn)^2);
Pe_star = 330000;

eqn = diff(y,t,2) == -2*xi*wn*diff(y,t) - wn^2*y + wn^2*Pe_star;
Dy = diff(y,t);
cond = [y(0) == 0, Dy(0) == 0]; 
ysol(t) = dsolve(eqn,cond);

fplot(ysol/1000,[0 10])
xlabel('Time [s]');
ylabel('P_{eng}^* [kW]')
grid on;

%% Engine Power vs Engine RPM, Engine Fuel vs Engine RPM

% From engine data sheet
% Pferdestarkenstunde" (PSh), 1 PSh = 0.73549875 kWh
eng_rpm = [12 13 14 15  16 17 18 19 20]*100;
eng_kw = [200 235 270 280 290 300 310 320 330];
eng_gkwh = [162 151 149 148 149 150 152 155 159]/0.73549875;

x1 = linspace(1200,2000);
y1l = interp1(eng_rpm,eng_kw,linspace(1200,2000));
[p,S,mu] = polyfit(eng_rpm,eng_gkwh,6);
y1r = polyval(p,linspace(1200,2000),[],mu);

figure;
plot(x1,y1l,[eng_rpm(1) eng_rpm(4) eng_rpm(9)],[eng_kw(1) eng_kw(4) eng_kw(9)],'o','LineWidth',1.5);
set(gca,'fontsize',13)
xlim([1000 2200]);
ylim([0 350]);
ylabel('P_{eng} [kW]');
xlabel('[rpm]');
grid on;

figure;
plot(x1,y1r,[eng_rpm(1) eng_rpm(4) eng_rpm(9)],[eng_gkwh(1) polyval(p,eng_rpm(4),[],mu) eng_gkwh(9)],'o','LineWidth',1.5);
set(gca,'fontsize',13,'ytick',190:10:240)
xlim([1000 2200]);
ylim([190 240]);
ylabel('[g/kWh]');
xlabel('[rpm]');
grid on;

%%
figure;
yyaxis left;
plot(x1,y1l,'LineWidth',1.5);
xlim([1000 2200]);
ylim([0 350]);
ylabel('P_{eng} [kW]');
yyaxis right
plot(x1,y1r,eng_rpm,eng_gkwh,'o','LineWidth',1.5);
ylim([190 240]);
ylabel('[g/kWh]');
xlabel('[rpm]');
grid on;

%% Engine Fuel Consumption vs Engine Output Power

eng_kw = [200 280 330];
eng_gkwh = [162 148 159]/0.73549875;

p = polyfit(eng_kw,eng_gkwh,2);
x1 = linspace(200,330);
y1 = polyval(p,x1);
figure;
yyaxis left;
plot(eng_kw,eng_gkwh,'o',x1,y1,'LineWidth',1.5);
ylabel('[g/kWh]');
yyaxis right;
plot(x1,(x1.*y1)/1000,'LineWidth',1.5);
ylabel('[kg/h]');
xlabel('P_{eng} [kW]');
grid on;
%% Ft & Ft linearisation
mv = 39600;
mp = 117*60;
iv = 1.1;
m = (mv+mp)*iv;
acc = 2.3/3.6;
dec = -2.0/3.6; 
Fstart = m*3*9.8/1000; % 3 kgf/ton

v = 0:100;
Ftmax = zeros(1,100);
Ftmin = zeros(1,100);

% Ft curves
for i=1:101
    % Ftmax
    if v(i) <= 20
        Ftmax(i) = m*acc + Fstart;
    elseif v(i) <= 70
        Ftmax(i) = Ftmax(21)*(20/v(i));
    else
        Ftmax(i) = Ftmax(21)*(20/v(i))*(70/v(i));
    end
    %Ftmin
    if v(i) <= 42
        Ftmin(i) = m*dec;
    else
        Ftmin(i) = m*dec*(42/v(i));
    end
end

figure;
plot(v,Ftmax/1000,'b',v,Ftmin/1000,'r',[0 100],[Ftmin(1) Ftmin(1)]/1000,'k--','LineWidth',1.1);
grid on;
legend('F_{t,max}','F_{t,min}','F_{mb}');
xlabel('v [km/h]');
ylabel('[kN]');

% Linearised Ft
vp = 0:20:100;
Ftmaxp = Ftmax(1:20:101);
Ftminp = Ftmin(1:20:101);
Ftmaxl = interp1(vp,Ftmaxp,v);
Ftminl = interp1(vp,Ftminp,v);

figure;
plot(v,Ftmaxl/1000,'b',v,Ftminl/1000,'r',[0 100],[Ftminl(1) Ftminl(1)]/1000,'k--','LineWidth',1.1);
grid on;
legend('F_{t,max}','F_{t,min}','F_{mb}');
xlabel('v [km/h]');
ylabel('[kN]');

%%
figure;
plot(v,(v/3.6).*Ftv_max,v,(v/3.6).*Ftv_min);
grid on;
title('Maximum output power at wheels');
legend('P_{out,max}','P_{out,min}');
xlabel('v [m/s]');
ylabel('P_{out} [kW]');

%% Pout
x1 = 0:10:100;
x2 = -28:4:32;
[xx,yy] = meshgrid(x1,x2);
z = (xx/3.6).*yy;

figure;
surf(xx,yy,z)
colormap hsv
colorbar
hold on;
%zheight = max(z(:)); %finds the highest point in the surface plot
%plot3(v,Ftv_max,zheight*ones(1,length(v)),'LineWidth',2,'Color','red');
%hold on;
%plot3(v,Ftv_min,zheight*ones(1,length(v)),'Linewidth',2);
%title('Vehicle output power');
xlabel('v [km/h]');
ylabel('F_t [kN]');
zlabel('P_{out} [kW]');
hold on;

%% Pout linearised
Ft0 = [0 0 0 0 0];
v0 = [20 40 60 80 100]/3.6;
vlb = [0 20 40 60 80];
vub = [20 40 60 80 100];
Ftlb = [-28 -28 -28 -28 -28];
Ftub = [32 32 32 32 32];

for i = 1:5
    [x,y] = meshgrid((vlb(i):10:vub(i))/3.6,Ftlb(i):4:Ftub(i));
    z = Ft0(i)*v0(i) + Ft0(i)*(x-v0(i)) + v0(i)*(y-Ft0(i));
    surf(x*3.6,y,z);
    hold on;
end
%title('Vehicle output power');
xlabel('v [km/h]');
ylabel('F_t [kN]');
zlabel('P_{out} [kW]');
colormap hsv
colorbar
grid on;

%% Pout Levels
Ftlb = [-28 -21 -14 -7 0 8 16 24];
Ftub = [-21 -14 -7 0 8 16 24 32];

for i = 1:10
    vlb = 10*(i-1);
    vub = 10*i;
    for j = 1:8
        [x,y] = meshgrid(vlb:vub,Ftlb(j):Ftub(j));
        if j <= 4
            zv = ((vub-10)/3.6)*(Ftlb(j)+3.5);
        else
            zv = ((vub-10)/3.6)*(Ftlb(j)+4);
        end
        z = zv*ones(size(x));
        surf(x,y,z);
        hold on;
    end
end
xlabel('v [km/h]');
ylabel('F_t [kN]');
zlabel('P_{out} [kW]');
colormap hsv
%% Heuristic EMS
plot([0 100],[80 80],'k',[0 100],[20 20],'k',[25 25],[0 100],'k',[0 25],[50 50],'k--',[25 100],[75 75],'k--',[25 100],[25 25],'k--','LineWidth',1.2);
ylim([0 100]);
xlabel('v [km/h]');
ylabel('SoC [%]');
dimA = [20 57 10 10]/100;
dimB = [55 45 10 10]/100;
dimC = [55 22.5 10 10]/100;
dimD = [55 67.3 10 10]/100;
dimE = [20 33 10 10]/100;
strA = 'A';
strB = 'B';
strC = 'C';
strD = 'D';
strE = 'E';
annotation('textbox',dimA,'String',strA,'FontSize',16,'EdgeColor','none');
annotation('textbox',dimB,'String',strB,'FontSize',16,'EdgeColor','none');
annotation('textbox',dimC,'String',strC,'FontSize',16,'EdgeColor','none');
annotation('textbox',dimD,'String',strD,'FontSize',16,'EdgeColor','none');
annotation('textbox',dimE,'String',strE,'FontSize',16,'EdgeColor','none');

