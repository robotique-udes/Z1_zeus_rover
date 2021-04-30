clear all
clc

%% Définition des masses 
%Masse Propulsion (kg)
mp=13.4;
%Masse Suspension (kg) 
ms=9.0;
%Masse Châssis (kg) 
mc=7.0;
%Masse Bras (kg) 
mb=7.0;
mbbe=2.0;
%Masse Puissance (kg) 
mpw=5.0;
%Masse Communication (kg) 
mantenne=2.4;
mcom=2.0;

m=mp+ms+mc+mb+mpw+mantenne+mcom+mbbe
%% Définition géométrie initiale du rover 
%Hauteur du bogie imposée par le prédimensionnement (m)
hbogie=0.212;
%Longueur du rover (m)
%L=1.169;
%Diamètre des roues (m)
d=0.3;
%hauteur suspension (m)
hs=.435;
L=1.169; %m
%hauteur & longueur châssis (m)
hc=0.16;
Lc=0.81; %m
df= 0.364 ; %m (du devant du châssis au devant de la plaque)
%Distance de la batterie (m)
Lba=0.25;
hba=0.20;
%hauteur & longueur de CoG bras (m)
Lb=.0;
hb=0.30;
Lbbe=.15;
%Hauteur antenne com
ha=.40;
%Distance du sac à dos de communication(m)
La=.10;

%% Définition des CoG géométriques initiaux 
%CoG Propulsion [x z]
CoGp =[(L/2) d]; 
%CoG Suspension [x z]
CoGs =[(L/3) ((4/5)*(hs-(d/2))+(d/2))]; 
%CoG Châssis [x z]
CoGc =[(L/2)+.08 (hs)];
%CoG Bras
CoGb=[CoGc(1)-(Lc/2)+Lb CoGc(2)+(hc/2)+hb];
CoGbbe=[CoGc(1)-(Lc/2)+Lbbe CoGc(2)-(hc/2)+Lbbe/3];
%CoG batterie [x z]
CoGpw =[CoGc(1)+(Lba)/2 (hs-((hc/2)-hc/3))]; 
%CoG Controle[x z]
CoGantenne =[CoGc(1)+(Lc/2) CoGc(2)+(hc/2)+ha];
CoGcom =[CoGc(1)+(Lc/2)+La CoGc(2)]; 

%% Calcul du Centre de masse initial 
%Centre de masse en X (m)
X=(mp*CoGp(1)+ms*CoGs(1)+mc*CoGc(1)+mpw*CoGpw(1)+mantenne*CoGantenne(1)+mcom*CoGcom(1)+mbbe*CoGbbe(1)+mb*CoGb(1))/m;
%Centre de masse en Z (m)
Z=(mp*CoGp(2)+ms*CoGs(2)+mc*CoGc(2)+mpw*CoGpw(2)+mantenne*CoGantenne(2)+mcom*CoGcom(2)+mbbe*CoGbbe(2)+mb*CoGb(2))/m;

% Centre de masse du rover
CoM=[X Z]

%% Calculs de tracé du rover

%Roues
M1=[d/2 ; d/2];
M3=[L-d/2 ; d/2];
%Suspension 
xc=M3(1)-CoM(1);
xb=3*xc/2;
x2=2*xb-(L-d);
M2x=M3(1)-x2;
B=[M3(1)-xb ; (d/2)+hbogie];
Bx=B(1);
R=[CoGc(1)-(Lc/2)+df+.06 ; CoGc(2)];
Rx=R(1); Ry=R(2);
CoMx=CoM(1);
n=0:25;
for i=1:25
%Suspension 
xc(i+1)=M3(1)-CoMx(i);
xb(i+1)=3*xc(i+1)/2;
x2(i+1)=2*xb(i+1)-(L-d);
%s=M2(1)-M1(1);
%h=0.5; %m
%r=d/2;
%zt=s/(((((r*(s^2-h^2)^.5+(h-r)*h)/(h-r)*((s^2-h^2)^.5)-h*r)^2)+1)^.5)
Bx(i+1)=M3(1)-xb(i+1);

B=[Bx(i+1) ; B(2)];
%Roues 2
M2x(i+1)=M3(1)-x2(i+1);

% Définition des CoG géométriques
%CoG Propulsion [x z]
CoGp =[(M3(1)+M2x(i+1)+M1(1))/3 d]; 
%CoG Suspension [x z]
CoGs =[Bx(i+1)+(M2x(i+1)-Bx(i+1))/5 (3/5)*(Ry(i)-(d/2))+(d/2)]; 
%CoG Châssis [x z]
CoGc =[CoGc(1) CoGc(2)];
%CoG batterie [x z]
CoGpw =[Rx(i)+(Lba/2) CoGc(2)-hc/2+hc/3]; 
%CoG bras [x z]
CoGb=[CoGc(1)-(Lc/2)+Lb CoGc(2)+(hc/2)+hb];
CoGbbe=[CoGc(1)-(Lc/2)+Lbbe CoGc(2)-(hc/2)+Lbbe/3];
%CoG Contrôle [x z]
CoGantenne =[CoGc(1)+(Lc/2) CoGc(2)+(hc/2)+ha];
CoGcom =[CoGc(1)+(Lc/2)+La CoGc(2)];
% Calcul du Centre de masse du Rover 
%Centre de masse en X (m)
X=(mp*CoGp(1)+ms*CoGs(1)+mc*CoGc(1)+mpw*CoGpw(1)+mantenne*CoGantenne(1)+mcom*CoGcom(1)+mbbe*CoGbbe(1)+mb*CoGb(1))/m;
%Centre de masse en Z (m)
Z=(mp*CoGp(2)+ms*CoGs(2)+mc*CoGc(2)+mpw*CoGpw(2)+mantenne*CoGantenne(2)+mcom*CoGcom(2)+mbbe*CoGbbe(2)+mb*CoGb(2))/m;
% Centre de masse du rover
CoM=[X Z];
CoMx(i+1)=CoM(1);
%Nouveau Rocker
Rx(i+1)=Rx(i) ;
Ry(i+1)=Ry(i) ;
R=[Rx ; Ry];
M2=[M2x(i+1) ; d/2];

end



figure(1)
plot(n,x2,'blue',n,xc,'red',n,CoMx,'green',n,xb,'black')
legend({'x2 : Distance M3-M2 (m)','xc : Distance M3-CoM (m)','Position du centre de masse (m)','xb : Distance M3-B (m)'},'Location','east')
title('Convergence des variables de dimensionnement')
xlabel('Nombre d''itérations') 
ylabel('Distance sur l''abscisse (m)') 

xb=xb(end)
x2=x2(end)
%% Calcul des angles et des longueurs des membrures 
CosThetaBogie =(dot(M1-B,M2-B))/(norm(M1-B)*norm(M2-B));
AngleBogie = acosd(CosThetaBogie)
CosThetaRocker = dot((R(:,end)-B),(R(:,end)-M3))/(norm(R(:,end)-B)*norm(R(:,end)-M3));
AngleRocker = acosd(CosThetaRocker)
DB=norm(R(:,end)-B)
DE=norm(R(:,end)-M3)
AC=norm(B-M2)
AB=norm(B-M1)

%% Équations de tracé
M=[M1 M2 M3];
labels1 = {'M1','M2','M3'};
CoMF=[X Z] %Centre de masse finale
%Tracé roues
th = 0:pi/50:2*pi;
Mx1 = d/2 * cos(th) + M(1,1);
My1 = d/2 * sin(th) + M(2,1);
Mx2 = d/2 * cos(th) + M(1,2);
My2 = d/2 * sin(th) + M(2,2);
Mx3 = d/2 * cos(th) + M(1,3);
My3 = d/2 * sin(th) + M(2,3);

%Tracé Membrures
Bx=[M1(1) B(1) M2(1)];
By=[M1(2) B(2) M2(2)];
RX=[B(1) R(1) M3(1)];
RY=[B(2) R(2) M3(2)];


% centres de masses
x= [CoGp(1) CoGs(1) CoGc(1) CoGpw(1) CoGantenne(1) CoGcom(1) CoGbbe(1) CoGb(1) CoM(1)];
y= [CoGp(2) CoGs(2) CoGc(2) CoGpw(2) CoGantenne(2) CoGcom(2) CoGbbe(2) CoGb(2) CoM(2)];
labels = {'CoGp','CoGs','CoGc','CoGpw','CoGantenne','CoGcom','CoGbbe','CoGb','Centre de Masse'};

% Tracé Châssis 
P1c=[CoGc(1)-Lc/2 CoGc(1)+Lc/2 CoGc(1)+Lc/2 CoGc(1)-Lc/2 CoGc(1)-Lc/2];
P2c=[CoGc(2)-hc/2 CoGc(2)-hc/2 CoGc(2)+hc/2 CoGc(2)+hc/2 CoGc(2)-hc/2];
%% Tracé graphique
%Tracé
figure(2)
plot(x,y,'o',P1c,P2c,'blue',M(1,1),M(2,1)'.',M(1,2),M(2,2)'.',M(1,3),M(2,3)'.',Mx1,My1,'blue',Mx2,My2,'blue',Mx3,My3,'blue',Bx,By,'green',RX,RY,'green')
text(x,y,labels,'VerticalAlignment','bottom','HorizontalAlignment','right')
text(M(1,:),M(2,:),labels1,'VerticalAlignment','bottom','HorizontalAlignment','right')
title('Projet Zeus - Rocker Bogie en fonction du centre de masse')
xlabel('Longueur (m)') 
ylabel('Hauteur (m)') 