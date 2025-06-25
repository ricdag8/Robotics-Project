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
Kp_theta = 2;
Kd_theta = 0.5;

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

qd = A1*(1-cos(w1*time)) + A2*(1-cos(w2*time)) + A3*(1-cos(w3*time))+ offset;

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
%if exist('theta_sim_output', 'var') && exist('tau_J_sim_output', 'var') && exist('time_sim_output', 'var')
    %theta_meas = theta_sim_output;
    %tau_J_meas = tau_J_sim_output;
    %time_sim = time_sim_output; % Se tutti esistono, usa il tempo salvato dal To Workspace
%else
    %disp('Dati di output della simulazione (theta_sim_output, tau_J_sim_output, time_sim_output) non trovati direttamente nel workspace.');
    %disp('Verifica attentamente le impostazioni dei blocchi "To Workspace" nel tuo modello Simulink (nome variabile, save format: Array).');
    %warning('I plot useranno il tempo e dati "NaN" per i segnali non trovati. La simulazione potrebbe non aver salvato i dati.');
    %
    % %% 3. Post-Elaborazione dei Dati (NUOVO METODO con Signal Logging)

disp('Recupero dei dati loggati dalla variabile simout...');

% I dati sono ora dentro la struttura simout.yout, organizzati per numero di porta.
% simout.yout{1} è il segnale collegato all'Outport 1 (theta_meas)
% simout.yout{2} è il segnale collegato all'Outport 2 (tau_J_meas)
% simout.yout{3} è il segnale collegato all'Outport 3 (il tempo)

theta_meas = simout.yout{1}.Values.Data;
tau_J_meas = simout.yout{2}.Values.Data;
time_sim   = simout.yout{3}.Values.Data;

disp('Dati recuperati con successo da simout.');



% Estrazione dei dati specifici per il plotting dal nuovo ordine di 'simout'.
% L'ordine {n} corrisponde al numero di porta impostato in Simulink.
theta_meas_plot = simout.yout{1}.Values.Data;
tau_J_meas_plot = simout.yout{5}.Values.Data;
time_sim_plot   = simout.yout{7}.Values.Data;

figure;
subplot(2,1,1);
plot(time_sim_plot, theta_meas_plot, 'DisplayName', 'Misurata (Sim.)');
hold on;
plot(sim_data.theta_d(:,1), sim_data.theta_d(:,2), '--', 'DisplayName', 'Desiderata');
title('Posizione Motore: Misurata (Simulata) vs Desiderata');
xlabel('Tempo (s)');
ylabel('Posizione (rad)');
legend show;
grid on;

subplot(2,1,2);
plot(time_sim_plot, tau_J_meas_plot, 'DisplayName', 'Misurata (Sim.)');
hold on;
plot(sim_data.tau_Jd(:,1), sim_data.tau_Jd(:,2), '--', 'DisplayName', 'Desiderata');
title('Coppia al Giunto: Misurata (Simulata) vs Desiderata');
xlabel('Tempo (s)');
ylabel('Coppia (Nm)');
legend show;
grid on;

disp('Grafici di verifica generati correttamente.');



%% 4. IDENTIFICAZIONE DEI PARAMETRI DEL MODELLO
disp('--- Inizio processo di identificazione dei parametri ---');

% Estrazione completa dei dati da 'simout'
theta_meas = simout.yout{1}.Values.Data;
dtheta_meas= simout.yout{2}.Values.Data;
q_pos      = simout.yout{3}.Values.Data;
dq_pos     = simout.yout{4}.Values.Data;
tau_J_meas = simout.yout{5}.Values.Data;
tau_in     = simout.yout{6}.Values.Data; 
time_sim   = simout.yout{7}.Values.Data;

% --- A. Stima della Rigidezza K (non richiede accelerazione) ---
disp('Stima della rigidezza K...');
K_stimato = (theta_meas - q_pos) \ tau_J_meas; 
disp(['Valore "Reale" di K   : ', num2str(K)]);
disp(['--> Valore Stimato di K: ', num2str(K_stimato)]);
disp(' ');

% --- B. Filtraggio dei segnali per la stima dei parametri inerziali ---
disp('Filtraggio dei segnali di velocità per ridurre il rumore...');
Fc = 15; % Frequenza di taglio in Hz
lpFilt = designfilt('lowpassfir', 'FilterOrder', 50, 'CutoffFrequency', Fc, 'SampleRate', Fs);

% *** MODIFICA CHIAVE: Uso di filtfilt per un filtraggio a fase zero ***
dtheta_filt = filtfilt(lpFilt, dtheta_meas);
dq_filt     = filtfilt(lpFilt, dq_pos);

ddtheta_filt = diff(dtheta_filt) / dt;
ddq_filt     = diff(dq_filt) / dt;

% Allineamento dei vettori alla nuova lunghezza dopo 'diff'
N = length(ddtheta_filt);
tau_J_meas_filt = tau_J_meas(1:N);
tau_in_filt     = tau_in(1:N);
q_pos_filt      = q_pos(1:N);

% --- C. Stima dei parametri inerziali usando i dati filtrati ---
disp('Stima dell''inerzia motore Mm con dati filtrati...');
Mm_stimato = ddtheta_filt \ (tau_in_filt - tau_J_meas_filt);
disp(['Valore "Reale" di Mm   : ', num2str(Mm)]);
disp(['--> Valore Stimato di Mm: ', num2str(Mm_stimato)]);
disp(' ');

disp('Stima di M e Pg con dati filtrati...');
X_m_pg = [ddq_filt, cos(q_pos_filt)]; 
parametri_stimati = X_m_pg \ tau_J_meas_filt;
M_stimato = parametri_stimati(1);
Pg_stimato = parametri_stimati(2);
disp(['Valore "Reale" di M    : ', num2str(M)]);
disp(['--> Valore Stimato di M : ', num2str(M_stimato)]);
disp(['Valore "Reale" di Pg   : ', num2str(Pg)]);
disp(['--> Valore Stimato di Pg: ', num2str(Pg_stimato)]);

disp('--- Processo di simulazione e identificazione completato ---');