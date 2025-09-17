function [A, B, C, D] = modello_giunto_teorico(Mm, M, K, Ts)
% Questa funzione definisce il modello state-space del giunto elastico
% per il toolbox di System Identification.

% Matrice di stato A
A = [0, 1, 0, 0;
     -K/M, 0, K/M, 0;
     0, 0, 0, 1;
     K/Mm, 0, -K/Mm, 0];

% Matrice di input B
B = [0;
     0;
     0;
     1/Mm];

% Matrice di output C
C = [0, 0, 1, 0;      % Uscita 1: y = theta = x3
     -K, 0, K, 0];   % Uscita 2: y = K*(theta - q) = K*x3 - K*x1

% Matrice di feedthrough D
D = [0;
     0];
end