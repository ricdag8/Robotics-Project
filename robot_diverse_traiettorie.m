%% Script principale per generare dati, avviare la simulazione e analizzare i risultati
clear all;
close all;
clc;
disp('--- Inizio processo di simulazione robotica ---');

%% 1. DEFINIZIONE DEI PARAMETRI E SETUP
disp('Definizione dei parametri del modello e del controllore...');

% --- Parametri "Reali" del Modello Fisico (Impianto) ---
Mm = 0.5;    % Inerzia del motore [kg*m^2]
K = 2000;    % Rigidezza del giunto elastico [Nm/rad]
M = 2.0;     % Inerzia del link [kg*m^2]
Pg = 5;      % Parametro di gravità [Nm]

% --- Parametri del Controllore ---
Kp_tau = 10;
Kd_tau = 0.1;
Kp_theta = 70;  % Guadagno proporzionale di posizione (valore sintonizzato)
Kd_theta = 20;  % Guadagno derivativo di posizione (valore sintonizzato)

% --- Parametri di Simulazione ---
T_sim = 10;  % Durata della simulazione in secondi
Fs = 1000;   % Frequenza di campionamento (Hz)
dt = 1/Fs;   % Passo temporale
time = (0:dt:T_sim)'; % Vettore tempo (già come colonna)

% --- Setup per Esecuzioni Multiple ---
num_esperimenti = 3; % Numero di traiettorie diverse da testare
disp(['Pronto per eseguire ', num2str(num_esperimenti), ' esperimenti con traiettorie diverse.']);

% Inizializzazione delle matrici per aggregare i dati da tutti gli esperimenti
all_theta_meas = [];
all_dtheta_meas= [];
all_q_pos      = [];
all_dq_pos     = [];
all_tau_J_meas = [];
all_tau_in     = [];

%% 2. ESECUZIONE DELLE SIMULAZIONI IN UN CICLO
for i = 1:num_esperimenti
    disp(['--- Esecuzione Esperimento N.', num2str(i), ' ---']);

    % --- Generazione di una Traiettoria Desiderata DIVERSA per ogni ciclo ---
    disp('Generazione di una nuova traiettoria...');
    % Modifichiamo le ampiezze per rendere ogni traiettoria unica
    A1 = 0.4 * i; f1 = 0.5; w1 = 2*pi*f1;
    A2 = 0.2 * i; f2 = 1.0; w2 = 2*pi*f2;
    A3 = 0.1 * i; f3 = 1.5; w3 = 2*pi*f3;
    offset = 0.0;

    qd = A1*sin(w1*time) + A2*sin(w2*time) + A3*sin(w3*time) + offset;
    dqd = A1*w1*cos(w1*time) + A2*w2*cos(w2*time) + A3*w3*cos(w3*time);
    ddqd = -A1*w1^2*sin(w1*time) - A2*w2^2*sin(w2*time) - A3*w3^2*sin(w3*time);

    g_qd = Pg * cos(qd);
    tau_Jd = M * ddqd + g_qd;
    theta_d = qd + (1/K) * tau_Jd;
    dtheta_d = dqd - (Pg/K) * sin(qd) .* dqd;
    ddtheta_d = ddqd - (Pg/K) * (cos(qd) .* dqd.^2 + sin(qd) .* ddqd);

    sim_data.qd = [time, qd];
    sim_data.dqd = [time, dqd];
    sim_data.ddqd = [time, ddqd];
    sim_data.tau_Jd = [time, tau_Jd];
    sim_data.theta_d = [time, theta_d];
    sim_data.dtheta_d = [time, dtheta_d];
    sim_data.ddtheta_d = [time, ddtheta_d];
    
    % *** CORREZIONE: Aggiunta della variabile mancante per compatibilità con il modello Simulink ***
    sim_data.time_for_input_ref = [time, time];

    % --- Avvio della Simulazione ---
    model_name = 'MyElasticRobotJointSim';
    if i == 1 % Apri il modello solo la prima volta
        open_system(model_name);
    end
    set_param(model_name, 'StopTime', num2str(T_sim));

    disp(['Avvio della simulazione per l''esperimento N.', num2str(i), '...']);
    simout = sim(model_name);
    disp('Simulazione completata.');

    % --- Aggregazione dei Dati ---
    disp('Aggregazione dei dati raccolti...');
    all_theta_meas = [all_theta_meas; simout.yout{1}.Values.Data];
    all_dtheta_meas= [all_dtheta_meas; simout.yout{2}.Values.Data];
    all_q_pos      = [all_q_pos;      simout.yout{3}.Values.Data];
    all_dq_pos     = [all_dq_pos;     simout.yout{4}.Values.Data];
    all_tau_J_meas = [all_tau_J_meas; simout.yout{5}.Values.Data];
    all_tau_in     = [all_tau_in;     simout.yout{6}.Values.Data];
end

%% 3. PLOT DEI RISULTATI (Mostra solo l'ultimo esperimento per chiarezza)
disp('Generazione dei grafici di verifica per l''ULTIMO esperimento...');
theta_meas_plot = simout.yout{1}.Values.Data;
tau_J_meas_plot = simout.yout{5}.Values.Data;
time_sim_plot   = simout.yout{7}.Values.Data;

figure;
subplot(2,1,1);
plot(time_sim_plot, theta_meas_plot, 'DisplayName', 'Misurata (Sim.)');
hold on;
plot(sim_data.theta_d(:,1), sim_data.theta_d(:,2), '--', 'DisplayName', 'Desiderata');
title('Posizione Motore (Ultimo Esperimento)');
xlabel('Tempo (s)');
ylabel('Posizione (rad)');
legend show; grid on;

subplot(2,1,2);
plot(time_sim_plot, tau_J_meas_plot, 'DisplayName', 'Misurata (Sim.)');
hold on;
plot(sim_data.tau_Jd(:,1), sim_data.tau_Jd(:,2), '--', 'DisplayName', 'Desiderata');
title('Coppia al Giunto (Ultimo Esperimento)');
xlabel('Tempo (s)');
ylabel('Coppia (Nm)');
legend show; grid on;

%% 4. IDENTIFICAZIONE DEI PARAMETRI (Sull'intero set di dati aggregati)
disp('--- Inizio processo di identificazione usando TUTTI i dati raccolti ---');

% --- A. Stima della Rigidezza K ---
disp('Stima della rigidezza K...');
K_stimato = (all_theta_meas - all_q_pos) \ all_tau_J_meas;
disp(['Valore "Reale" di K   : ', num2str(K)]);
disp(['--> Valore Stimato di K: ', num2str(K_stimato)]);
disp(' ');

% --- B. Filtraggio e preparazione dei dati aggregati ---
disp('Filtraggio dei segnali aggregati per la stima dei parametri inerziali...');
Fc = 15;
lpFilt = designfilt('lowpassfir', 'FilterOrder', 50, 'CutoffFrequency', Fc, 'SampleRate', Fs);

dtheta_filt = filtfilt(lpFilt, all_dtheta_meas);
dq_filt     = filtfilt(lpFilt, all_dq_pos);

ddtheta_filt = diff(dtheta_filt) / dt;
ddq_filt     = diff(dq_filt) / dt;

N = length(ddtheta_filt);
tau_J_meas_filt = all_tau_J_meas(1:N);
tau_in_filt     = all_tau_in(1:N);
q_pos_filt      = all_q_pos(1:N);

% --- C. Stima dei parametri inerziali usando i dati filtrati e aggregati ---
disp('Stima dell''inerzia motore Mm...');
Mm_stimato = ddtheta_filt \ (tau_in_filt - tau_J_meas_filt);
disp(['Valore "Reale" di Mm   : ', num2str(Mm)]);
disp(['--> Valore Stimato di Mm: ', num2str(Mm_stimato)]);
disp(' ');

disp('Stima di M e Pg...');
X_m_pg = [ddq_filt, cos(q_pos_filt)];
parametri_stimati = X_m_pg \ tau_J_meas_filt;
M_stimato = parametri_stimati(1);
Pg_stimato = parametri_stimati(2);
disp(['Valore "Reale" di M    : ', num2str(M)]);
disp(['--> Valore Stimato di M : ', num2str(M_stimato)]);
disp(['Valore "Reale" di Pg   : ', num2str(Pg)]);
disp(['--> Valore Stimato di Pg: ', num2str(Pg_stimato)]);

disp('--- Processo di simulazione e identificazione completato ---');
