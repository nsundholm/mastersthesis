%% Engine Dynamics Approximation
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
ylabel('P_{eng}^* [kW]'])
grid on;

%% Engine Power vs Engine RPM, Engine Fuel vs Engine RPM

% From engine data sheet
% Pferdestarkenstunde" (PSh), 1 PSh = 0.73549875 kWh
eng_rpm = [12 13 14 15 16 17 18 19 20]*100;
eng_kw = [200 235 270 280 290 300 310 320 330];
eng_gkwh = [162 151 149 148 148 149 152 155 159]/0.73549875;

x1 = linspace(1200,2000);
y1l = interp1(eng_rpm,eng_kw,linspace(1200,2000));
[p,S,mu] = polyfit(eng_rpm,eng_gkwh,7);
y1r = polyval(p,linspace(1200,2000),[],mu);

figure;
yyaxis left;
plot(x1,y1l,'LineWidth',1.5);
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

%% Tractive Effort, Maximum Engine Output Power, Engine Output Power
v = 0:100;
v_p = 0:20:100;
Ft_maxp = [32 32 19 13 10 7];
Ft_minp = [28 28 28 14 8 5]*-1;
Ftv_max = interp1(v_p,Ft_maxp,v);
Ftv_min = interp1(v_p,Ft_minp,v);

figure;
plot(v,Ftv_max,v,Ftv_min);
grid on;
title('Traction and regenerative brake curves');
legend('F_{t,max}','F_{t,min}');
xlabel('v [km/h]');
ylabel('F_t [kN]');

figure;
plot(v,(v/3.6).*Ftv_max,v,(v/3.6).*Ftv_min);
grid on;
title('Maximum output power at wheels');
legend('P_{out,max}','P_{out,min}');
xlabel('v [m/s]');
ylabel('P_{out} [kW]');


x1 = 0:5:100;
x2 = -28:2:32;
[xx,yy] = meshgrid(x1,x2);
z = (xx/3.6).*yy;

figure;
surf(xx,yy,z)
colormap hsv
colorbar
hold on;
zheight = max(z(:)); %finds the highest point in the surface plot
plot3(v,Ftv_max,zheight*ones(1,length(v)),'LineWidth',2,'Color','red');
hold on;
plot3(v,Ftv_min,zheight*ones(1,length(v)),'Linewidth',2);
title('Engine output power');
xlabel('v [km/h]');
ylabel('F_t [kN]');
zlabel('P_{out} [kW]');
hold on;

%% Linearization of Engine Output (Ft*v)
Ft0 = [0 0 0 0 0];
v0 = [10 30 50 70 90]/3.6;
vlb = [0 20 40 60 80];
vub = [20 40 60 80 100];
Ftlb = [-28 -28 -28 -28 -28];
Ftub = [32 32 32 32 32];

for i = 1:5
    [x,y] = meshgrid((vlb(i):vub(i))/3.6,Ftlb(i):Ftub(i));
    z = Ft0(i)*v0(i) + Ft0(i)*(x-v0(i)) + v0(i)*(y-Ft0(i));
    surf(x*3.6,y,z);
    hold on;
end
xlabel('v [km/h]');
ylabel('F_t [kN]');
zlabel('P_{out} [kW]');
colormap hsv
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

