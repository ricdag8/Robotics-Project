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
Pg = 5;     % Parametro di gravità [Nm]

% Parametri del controllore (necessari nel modello Simulink)
Kp_tau = 10;
Kd_tau = 0.1;
Kp_theta = 1;
Kd_theta = 0.1;

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

%% Nuova sezione in run_robot_simulation.m per il formato Matrix [tempo, dati]

% Assicurati che i dati siano vettori colonna
time_col = time';
qd_col = qd';
dqd_col = dqd';
ddqd_col = ddqd';
tau_Jd_col = tau_Jd';
theta_d_col = theta_d';
dtheta_d_col = dtheta_d';
ddtheta_d_col = ddtheta_d';

% Crea le variabili nella struttura sim_data come MATRICI [tempo, dati]
% Questo è il formato che il blocco 'From Workspace' sta richiedendo esplicitamente.
sim_data.qd = [time_col, qd_col];
sim_data.dqd = [time_col, dqd_col];
sim_data.ddqd = [time_col, ddqd_col];
sim_data.tau_Jd = [time_col, tau_Jd_col];
sim_data.theta_d = [time_col, theta_d_col];
sim_data.dtheta_d = [time_col, dtheta_d_col];
sim_data.ddtheta_d = [time_col, ddtheta_d_col];

% IMPORTANTE: Per il blocco From Workspace che caricherà sim_data.time_ref
% Questo specifico blocco non deve aspettarsi un formato [tempo, dati].
% Useremo questo per il blocco di 'time_ref' che hai nel sottosistema 'Input Desiderati'.
% Per il blocco From Workspace che richiede un formato [tempo, dati] anche per il tempo stesso
% Creiamo una matrice con il tempo nella prima colonna e un duplicato del tempo (o zeri) nella seconda.
sim_data.time_for_input_ref = [time_col, time_col]; % Ora è una matrice 10001x2
disp('Traiettorie generate e variabili caricate nel workspace.');
sim_data.time = time_col;

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
time_sim = sim_data.time; % Questo garantisce che time_sim sia sempre definito per il plot e le inizializzazioni

theta_meas = NaN(size(time_sim)); 
tau_J_meas = NaN(size(time_sim));

% Controlla se i dati di output di Simulink sono stati salvati correttamente
if exist('theta_sim_output', 'var') && exist('tau_J_sim_output', 'var') && exist('time_sim_output', 'var')
    theta_meas = theta_sim_output;
    tau_J_meas = tau_J_sim_output;
    time_sim = time_sim_output; % Se tutti esistono, usa il tempo salvato dal To Workspace
else
    disp('Dati di output della simulazione (theta_sim_output, tau_J_sim_output, time_sim_output) non trovati direttamente nel workspace.');
    disp('Verifica attentamente le impostazioni dei blocchi "To Workspace" nel tuo modello Simulink (nome variabile, save format: Array).');
    warning('I plot useranno il tempo e dati "NaN" per i segnali non trovati. La simulazione potrebbe non aver salvato i dati.');
end

%% 4. Plot dei Risultati Simulati (per verifica)
figure;
subplot(2,1,1);
plot(time_sim, theta_meas, 'DisplayName', 'Misurata (Sim.)'); % theta_meas è un array normale
hold on;
% Modifica qui per accedere a tempo e dati dalla matrice sim_data.theta_d
plot(sim_data.theta_d(:,1), sim_data.theta_d(:,2), '--', 'DisplayName', 'Desiderata');
title('Posizione Motore: Misurata (Simulata) vs Desiderata');
xlabel('Tempo (s)');
ylabel('Posizione (rad)');
legend show; % Mostra la legenda
grid on;

subplot(2,1,2);
plot(time_sim, tau_J_meas, 'DisplayName', 'Misurata (Sim.)');
hold on;
% Modifica qui per accedere a tempo e dati dalla matrice sim_data.tau_Jd
plot(sim_data.tau_Jd(:,1), sim_data.tau_Jd(:,2), '--', 'DisplayName', 'Desiderata');
title('Coppia al Giunto: Misurata (Simulata) vs Desiderata');
xlabel('Tempo (s)');
ylabel('Coppia (Nm)');
legend show;
grid on;
disp('--- Processo di simulazione robotica completato ---');