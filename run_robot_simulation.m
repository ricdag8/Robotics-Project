%% Script principale per generare dati, avviare la simulazione Simulink e analizzare i risultati

clear all;
close all;
clc;

disp('--- Inizio processo di simulazione robotica ---');

%% 1. Generazione delle Traiettorie Desiderate e dei Profili di Coppia
% Questo è il contenuto del tuo script generate_robot_trajectories.m
% Puoi includere il codice direttamente qui o chiamare lo script.
% In questo esempio, lo includiamo direttamente per chiarezza.

disp('Generazione delle traiettorie desiderate...');

% Definizione dei Parametri del Modello "Vero" (Nominale)
% Questi sono i parametri che useremmo per la simulazione del robot.
Mm = 0.5;    % Inerzia del motore [kg*m^2]
K = 2000;    % Rigidezza del giunto elastico [Nm/rad]
M = 2.0;     % Inerzia del link [kg*m^2]
Pg = 10;     % Parametro di gravità [Nm]

% Parametri del controllore (necessari nel modello Simulink)
Kp_tau = 100;
Kd_tau = 5;
Kp_theta = 50;
Kd_theta = 2;

% Tempo di simulazione
T_sim = 10;  % Durata della simulazione in secondi
Fs = 1000;   % Frequenza di campionamento (Hz)
dt = 1/Fs;   % Passo temporale
time = 0:dt:T_sim; % Vettore tempo

% Traiettoria q_d(t) come somma di sinusoidi
A1 = 0.5; f1 = 0.5; w1 = 2*pi*f1;
A2 = 0.3; f2 = 1.0; w2 = 2*pi*f2;
A3 = 0.2; f3 = 1.5; w3 = 2*pi*f3;
offset = 0.0; % Posizione di riposo, es. 0 rad

qd = A1*sin(w1*time) + A2*sin(w2*time) + A3*sin(w3*time) + offset;

% Calcolo delle Derivate di q_d(t) analiticamente
dqd = A1*w1*cos(w1*time) + A2*w2*cos(w2*time) + A3*w3*cos(w3*time);
ddqd = -A1*w1^2*sin(w1*time) - A2*w2^2*sin(w2*time) - A3*w3^2*sin(w3*time);
% dddqd = -A1*w1^3*cos(w1*time) - A2*w2^3*cos(w2*time) - A3*w3^3*cos(w3*time);
% dddddqd = A1*w1^4*sin(w1*time) + A2*w2^4*sin(w2*time) + A3*w3^4*sin(w3*time);

% Funzione di gravità g(q) = Pg * cos(q)
g_qd = Pg * cos(qd);

% Calcolo del Profilo di Coppia Elastica Desiderata (tau_Jd)
tau_Jd = M * ddqd + g_qd;

% Calcolo della Traiettoria Desiderata del Motore (theta_d)
theta_d = qd + (1/K) * g_qd;
dtheta_d = dqd - (Pg/K) * sin(qd) .* dqd;
ddtheta_d = ddqd - (Pg/K) * (cos(qd) .* dqd.^2 + sin(qd) .* ddqd);

% Organizza i dati per i blocchi From Workspace in Simulink come oggetti timeseries
% Questo permette ai blocchi From Workspace di interpretare correttamente tempo e dati.

% Assicurati che i dati siano vettori colonna
time_col = time';
qd_col = qd';
dqd_col = dqd';
ddqd_col = ddqd';
tau_Jd_col = tau_Jd';
theta_d_col = theta_d';
dtheta_d_col = dtheta_d';
ddtheta_d_col = ddtheta_d';

% Crea oggetti timeseries per ciascun segnale desiderato
sim_data.qd = timeseries(qd_col, time_col);
sim_data.dqd = timeseries(dqd_col, time_col);
sim_data.ddqd = timeseries(ddqd_col, time_col);
sim_data.tau_Jd = timeseries(tau_Jd_col, time_col);
sim_data.theta_d = timeseries(theta_d_col, time_col);
sim_data.dtheta_d = timeseries(dtheta_d_col, time_col);
sim_data.ddtheta_d = timeseries(ddtheta_d_col, time_col);

% Non abbiamo bisogno di sim_data.time separato, perché è già incorporato in ogni timeseries object
% Tuttavia, se altri blocchi necessitano di sim_data.time come array, puoi lasciarlo:
sim_data.time = time_col; % Lascia questo se altri blocchi o il codice post-simulazione lo usano come array

disp('Traiettorie generate e variabili caricate nel workspace.');


%% 2. Configurazione e Avvio della Simulazione Simulink
% Assicurati che il nome del tuo file .slx sia corretto
model_name = 'MyElasticRobotJointSim'; % <--- CAMBIA QUESTO CON IL NOME DEL TUO MODELLO SIMULINK

% Apri il modello Simulink (se non è già aperto)
open_system(model_name);

% Configura il tempo di simulazione nel modello
set_param(model_name, 'StopTime', num2str(T_sim));

% Esegui la simulazione
disp(['Avvio della simulazione del modello: ', model_name, '...']);
simout = sim(model_name); % Esegue la simulazione e salva l'output (se usi Data Logging)

disp('Simulazione completata.');

%% 3. Post-Elaborazione dei Dati (Esempio)
% Recupera i dati dai blocchi To Workspace se li hai configurati
% Ad esempio, se hai salvato theta_meas e tau_J_meas nel workspace da Simulink

% Supponendo che tu abbia configurato i blocchi To Workspace per salvare:
% theta_sim_output (per la posizione del motore misurata)
% tau_J_sim_output (per la coppia al giunto misurata)
% time_sim_output (per il tempo della simulazione)
% Se hai usato logging automatico dei segnali (es. dalla root outports), puoi accedervi via simout.

% Esempio di recupero dati da 'To Workspace'
if exist('theta_sim_output', 'var') && exist('tau_J_sim_output', 'var')
    theta_meas = theta_sim_output;
    tau_J_meas = tau_J_sim_output;
    % Se non hai un blocco To Workspace per il tempo, usa sim_data.time
    time_sim = sim_data.time; % Usa il tempo generato, che sarà lo stesso della simulazione
else
    disp('Dati di output della simulazione non trovati direttamente nel workspace.');
    disp('Verifica le impostazioni dei blocchi "To Workspace" nel tuo modello Simulink.');
    % Alternativa: accedere ai dati registrati da Simulink se hai abilitato Data Logging
    % ad esempio: theta_meas = simout.logsout.get('theta_meas').Values.Data;
    % Richiede che tu abbia nominato i segnali e abilitato la registrazione.
end


%% 4. Plot dei Risultati Simulati (per verifica)
figure;
subplot(2,1,1);
plot(time_sim, theta_meas);
hold on;
plot(time_sim, sim_data.theta_d, '--');
title('Posizione Motore: Misurata (Simulata) vs Desiderata');
xlabel('Tempo (s)');
ylabel('Posizione (rad)');
legend('Misurata (Sim.)', 'Desiderata');
grid on;

subplot(2,1,2);
plot(time_sim, tau_J_meas);
hold on;
plot(time_sim, sim_data.tau_Jd, '--');
title('Coppia al Giunto: Misurata (Simulata) vs Desiderata');
xlabel('Tempo (s)');
ylabel('Coppia (Nm)');
legend('Misurata (Sim.)', 'Desiderata');
grid on;

disp('--- Processo di simulazione robotica completato ---');