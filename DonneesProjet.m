clear all; close all; clc;

% Paramètres 

m = 1; k = 1000;   
F1 = 100; F2 = 0; F3 = 0;

% Pot vibrant

MasseMobile = 0.02;
MasseCoffre = 1.08;
ka = 2000; 

% Actionneur entre S2 et S3 : confirmé
K_S2_S3 = [4*k -3*k  -k
     -3*k 8*k+ka -(5*k+ka)
     -k -(5*k+ka) 6*k+ka];
M_S2_S3 = [ 2*m 0 0 ; 0 m+MasseCoffre 0; 0 0 3*m+MasseMobile];

% Actionneur entre S1 et S2 : confirmé
K_S1_S2 = [4*k -(3*k+ka) -k
     -(3*k+ka) 8*k+ka -5*k
     -k -5*k 6*k]; 
M_S1_S2 = [ 2*m+MasseMobile 0 0 ; 0 m+MasseCoffre 0; 0 0 3*m];
 
% Actionneur entre S1 et S3 : confirmé
K_S1_S3 = [4*k+ka -3*k -(k+ka)
     -3*k 8*k -5*k
     -(k+ka) -5*k 6*k+ka]; 
M_S1_S3 = [ 2*m+MasseCoffre 0 0 ; 0 m 0; 0 0 3*m+MasseMobile];

% State Space 

A_S2_S3 = [ zeros(3,3) eye(3)
      -inv(M_S2_S3)*K_S2_S3 zeros(3,3)]; 
B_S2_S3 = [zeros(3,3); inv(M_S2_S3)]; 

A_S1_S2 = [ zeros(3,3) eye(3)
      -inv(M_S1_S2)*K_S1_S2 zeros(3,3)]; 
B_S1_S2 = [zeros(3,3); inv(M_S1_S2)]; 



% Capteurs sur les 3 déplacements 
C1 = [eye(3) zeros(3,3)]; 

% Question 2.6 : choix de la position optimale de l'actionneur

% % Cas 1 : actionneur entre S1 et S2 : solution optimale
[VecteursPropres,ValeursPropres] = eig(K_S1_S2,M_S1_S2);  
B = [1 0 0; -1 0 0; 0 0 0]; 
Btild = VecteursPropres'*B;
c1 = det(Btild(1,:)*Btild(1,:)')^(1/2);
c2 = det(Btild(2,:)*Btild(2,:)')^(1/2);
c3 = det(Btild(3,:)*Btild(3,:)')^(1/2);

%Cas 2 : actionneur entre S2 et S3
% [VecteursPropres,ValeursPropres] = eig(K_S2_S3,M_S2_S3);  
% B = [0 0 0; 0 0 -1; 0 0 1]; % -F3 et F3 sur S2 et S3 , masse mobile sur S3
% Btild = VecteursPropres'*B;
% c1 = det(Btild(1,:)*Btild(1,:)')^(1/2);
% c2 = det(Btild(2,:)*Btild(2,:)')^(1/2);
% c3 = det(Btild(3,:)*Btild(3,:)')^(1/2);
% % 
% Cas 3 : actionneur entre S1 et S3
% [VecteursPropres,ValeursPropres] = eig(K_S1_S3,M_S1_S3);  
% B = [0 0 -1; 0 0 0; 0 0 1]; % -F3 et F3 sur S1 et S3 
% Btild = VecteursPropres'*B;
% c1 = det(Btild(1,:)*Btild(1,:)')^(1/2);
% c2 = det(Btild(2,:)*Btild(2,:)')^(1/2);
% c3 = det(Btild(3,:)*Btild(3,:)')^(1/2);
% 

% Question 2.8
Q1 = 1e4*diag([3,3,3,3,3,3]);
Q2 = 1e7*diag([1e4,1e4,1e4,1e2,1e2,1e2]);


%Q2.9
C = [1 0 0 0 0 0];
A2 = [A_S1_S2 zeros(6,1); C 0]; 
B2 = [B_S1_S2(:,1);0]; 
Q3 = 1e8*diag([1000,1000,1000,1,1,1,1e5]);


%%%% Partie 3: ETUDE MODALE %%%%%%

% Question 3.1 
xi = 0; % à confirmer
[phi,omega_carre] = eig(K_S1_S2,M_S1_S2);
FreqMod = sqrt(abs(omega_carre))/(2*pi); 
Amod = [zeros(3) eye(3); -omega_carre -2*xi*sqrt(omega_carre)];
Bmod = [zeros(3); phi']; 
Cmod = [phi zeros(3,3)]; % Que deux capteurs de déplacements

Capt = [ eye(2) zeros(2,4) ; zeros(1,6)]; 


% Question 3.4

Amod2 = [Amod(1:2,1:2) Amod(1:2,4:5) ; Amod(4:5,1:2) Amod(4:5,4:5)];
Bmod2 = [Bmod(1:2,1);Bmod(4:5,1)];

% Augmente avec intégrateur 
Bcont = [Bmod2;0]; 
Amod2cont = [Amod2 zeros(4,1); Cmod(1,1:2) Cmod(1,4:5) 0];
Q = diag([1e7 1e7 1e5 1e5 1e12]); 



