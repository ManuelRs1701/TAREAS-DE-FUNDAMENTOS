% Universidad Catolica Boliviana San Pablo Regional Santa Cruz de la Sierra
% Proyecto de controlador PID por localizacion de polos
% Prof. Rodrigo Ivan Goytia Mejia
% Estudiante: Emanuel Román Sánchez
%fecha de presentacion 15/11/2022

close all
clear
clc
% Modelo del proceso de 2da orden en lazo abierto
% -------------------------------------------------------
% Ganancia del proceso
yini = 0;
yfin = 1.5;
uini = 0;
ufin = 4;
deltay = yfin-yini;
deltau = ufin-uini;
K = deltay/deltau;

% Atraso de transporte
theta = 0;

% Modelo del proceso de 2da orden
T = 30;
NC = 4;
psim = 0.6/NC;
wnm = 2*pi/T;

num = K*wnm^2;
den = [1 2*psim*wnm wnm^2];
Gs = tf(num,den);
Gs.iodelay = theta;

% Punto de operación
y0 = 10;
u0 = 4;

% Especificaciones del comportamiento en lazo cerrado
% -------------------------------------------------------
syms psi wn

% Calculo del coeficiente de amortiguamiento
Mp = 1/100;
eqn = exp(-(psi*pi)/sqrt(1-psi^2))==Mp;
psi = eval(solve(eqn,psi));
psi = psi(1);

% Frecuencia natural usando el tiempo pico, tp
tp = 40;
eqn = pi/(wn*sqrt(1-psi^2))==tp;
wn = eval(solve(eqn,wn));

% % Frecuencia natural usando el tiempo estabilizacion, ts al 2% de error
% ts2p = 40;
% eqn = 4/(psi*wn)==ts2p;
% wn = eval(solve(eqn,wn));

% % Frecuencia natural usando de subida, tr
% tr = 10;
% eqn = ((2.3*psi^2)-0.08*psi+1.1)/wn==tr;
% wn = eval(solve(eqn,wn));

% Determinando los polos de lazo cerrado de un sistema de 2da orden
polos(1) = -psi*wn+wn*sqrt(1-psi^2)*1i;
polos(2) = -psi*wn-wn*sqrt(1-psi^2)*1i;

% Parametro de ajuste del controlador p
p = 20;

% Ecuacion caracteristica de lazo cerrado deseada
ecarac = conv(conv([1 -polos(1)],[1 -polos(2)]),[1 p]);

% Proyecto del controlador PID
% Gc(s)=Kc(1+1/(Ti*s)+Td*s/(Tfs+1))  
% Gc(s)=Kp+Ki/s+Kd*s/(Tfs+1))
% -------------------------------------------------------
% Parametros del controlador PID

% Sintonia por localización de polos
%Kc = (wn^2+2*psi*wn*p-wn^2)/K ;       %Kc= 12.2805
%Ti = (Kc*K)/(wn^2*p);                 %Ti= 11.8548
%Td = (2*psi*wn+p-2*psi*wn)/(K*Kc);    %Td= 4.3429

Kc= 13.2805
%Ti= 12.8548
Td= 4.3429

%------------------------------------------------------------------------------------------
% PRUEBA 1
 %Kc = 5
 %Ti = 0
 %Td = 0

% PRUEBA 2
 %Kc = 5
 Ti = 100
 %Td = 0

% PRUEBA 3
 %Kc = 1
 %Ti = 1
 %Td = 1

% % Sintonia IMC
 %taulf = 2;
 %Kc = (2*psim/wnm)/(K*taulf)
 %Ti = (2*psim)/wnm
 %Td = 1/(2*psim*wnm^2)


%------------------------------------------------------------------------------------------
% control pid paralelo
Kp = Kc;
Ki = Kc/Ti;
Kd = Kc*Td;


Tf = 0.01;   % Filtro derivativo

% Modelo del controlador PID 
Gc = Kp+tf(Ki,[1 0])+tf([Kd 0],[Tf 1]);
% Gc = pid(Kp,Ki,Kd,Tf);


% Sistema realimentado
% -------------------------------------------------------
N = 40;

Gslf = feedback(Gc*Gs,1);

% Señal de referencia
yr = [ones(1,40) 4*ones(1,40) 2*ones(1,40) 2*ones(1,40) 2*ones(1,40)]';
% yr = ones(N,1);


% Modelo del ruido
medn = 0;
stdn = 0.1;
n = medn + stdn*randn(1,N*5);

% Modelo de disturbio
d = zeros(1,N*5);
pw = 0.2;           % porcentaje de la perturbacion en funcion de la referencia
pini = 120;
pfin = 160;
w = linspace(pw*yr(pini),-pw*yr(pfin),40);
d(120:160-1)=w;


t = 0:1:length(yr)-1;

% Señal de salida del sistema realimentado
y = lsim(Gslf,yr,t)+n'+d';

% Señal de error
e = yr-y;

% Señal de control
u = lsim(Gc,e,t);
u(1) = u(2);

% Señal de salida del sistema en lazo abierto
yla = lsim(Gs,yr,t);

% Calculo de indicadores
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Para comportamiento de seguimiento de referencia (servo)
IAEs = sum(abs(e(1:100)));
ISEs = sum(e(1:100).^2);
IAEopts = 100;
IAE_Exs = ((IAEs-IAEopts)/IAEs)*100;
% Para comportamiento de rechazo de perturbaciones (regulatorio)
IAEr = sum(abs(e(100:end)));
ISEr = sum(e(100:end).^2);
IAEoptr = 2;
IAE_Exr = ((IAEr-IAEoptr)/IAEr)*100;
disp('--------------------------------------------------')
disp('Indicadores de desempeño')
disp('--------------------------------------------------')
disp('Comportamiento servo')
fprintf('IAE = %6.2f\n',IAEs)
fprintf('ISE = %6.2f\n',ISEs)
fprintf('IAE Expertune = %6.2f\n',IAE_Exs)
disp('Comportamiento regulatorio')
fprintf('IAE = %6.2f\n',IAEr)
fprintf('ISE = %6.2f\n',ISEr)
% fprintf('IAE Expertune = %6.2f\n',IAE_Exr)
disp('--------------------------------------------------')
% Impresion de los resultados
% -------------------------------------------------------
figure
subplot(3,1,1)
plot(yr,'r--','linewidth',2), hold on
plot(y,'k','linewidth',2),
plot(yla,'b','linewidth',2),
ylabel('salida, y'), grid
legend('yr','ylf','yla')
title('Respuesta del Controlador PID por localización de polos')
subplot(3,1,2)
plot(u,'k','linewidth',2), ylabel('control, u'), grid
subplot(3,1,3)
plot(e,'k','linewidth',2), ylabel('error, e'), xlabel('Tiempo, s'),grid
