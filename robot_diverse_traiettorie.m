%% Script principale per generare dati, avviare la simulazione, analizzare i risultati e stimare i parametri
clear all;
close all;
clc;
disp('--- Inizio processo di simulazione robotica e identificazione ---');

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
% Guadagni "aggressivi" necessari per inseguire le traiettorie di stima
Kp_theta = 75; 
Kd_theta = 30;

% --- Parametri di Simulazione ---
T_sim = 10;  % Durata della simulazione in secondi
Fs = 1000;   % Frequenza di campionamento (Hz)
dt = 1/Fs;   % Passo temporale
time = (0:dt:T_sim)'; % Vettore tempo (già come colonna)

% --- Setup per Esecuzioni Multiple ---
num_esperimenti = 3; % Numero di traiettorie diverse da testare
disp(['Pronto per eseguire ', num2str(num_esperimenti), ' esperimenti con traiettorie diverse.']);

% Inizializzazione delle matrici per aggregare TUTTI i dati necessari
all_theta_meas = [];
all_q_pos      = [];
all_tau_J_meas = [];
all_tau_in     = [];
all_ddtheta_filt = []; % Nuovo: per accelerazione motore filtrata
all_ddq_filt     = []; % Nuovo: per accelerazione carico filtrata

%% 2. ESECUZIONE DELLE SIMULAZIONI E PRE-ELABORAZIONE IN UN CICLO
% --- Setup del Filtro (creato una sola volta prima del ciclo) ---
Fc = 20; % Hz (Frequenza di taglio più alta per traiettorie aggressive)
lpFilt = designfilt('lowpassfir', 'FilterOrder', 50, 'CutoffFrequency', Fc, 'SampleRate', Fs);

for i = 1:num_esperimenti
    disp(['--- Esecuzione Esperimento N.', num2str(i), ' ---']);

    % --- Generazione di una Traiettoria "ECCITANTE" per la STIMA ---
    disp('Generazione di una nuova traiettoria eccitante...');
    
    t_norm = time / T_sim; 
    s_t = 6*t_norm.^5 - 15*t_norm.^4 + 10*t_norm.^3; 
    s_t_dot = (1/T_sim) * (30*t_norm.^4 - 60*t_norm.^3 + 30*t_norm.^2);
    s_t_ddot = (1/T_sim^2) * (120*t_norm.^3 - 180*t_norm.^2 + 60*t_norm);

    A1 = 0.6 * i; f1 = 1.5; w1 = 2*pi*f1;
    A2 = 0.4 * i; f2 = 2.5; w2 = 2*pi*f2;
    A3 = 0.2 * i; f3 = 3.5; w3 = 2*pi*f3;
    
    qd_base = A1*sin(w1*time) + A2*sin(w2*time) + A3*sin(w3*time);
    dqd_base = A1*w1*cos(w1*time) + A2*w2*cos(w2*time) + A3*w3*cos(w3*time);
    ddqd_base = -A1*w1^2*sin(w1*time) - A2*w2^2*sin(w2*time) - A3*w3^2*sin(w3*time);

    qd = s_t .* qd_base;
    dqd = s_t_dot .* qd_base + s_t .* dqd_base;
    ddqd = s_t_ddot .* qd_base + 2 * s_t_dot .* dqd_base + s_t .* ddqd_base;
    
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
    sim_data.time_for_input_ref = [time, time];

    % --- Avvio della Simulazione ---
    model_name = 'MyElasticRobotJointSim';
    if i == 1 
        open_system(model_name);
    end
    set_param(model_name, 'StopTime', num2str(T_sim));

    disp(['Avvio della simulazione per l''esperimento N.', num2str(i), '...']);
    simout = sim(model_name);
    disp('Simulazione completata.');

    % --- Estrazione e Filtraggio per il SINGOLO esperimento ---
    dtheta_meas_i = simout.yout{2}.Values.Data;
    dq_pos_i      = simout.yout{4}.Values.Data;
    
    dtheta_filt_i = filtfilt(lpFilt, dtheta_meas_i);
    dq_filt_i     = filtfilt(lpFilt, dq_pos_i);

    ddtheta_filt_i = diff(dtheta_filt_i) / dt;
    ddq_filt_i     = diff(dq_filt_i) / dt;
    
    % --- Aggregazione dei Dati già elaborati ---
    disp('Aggregazione dei dati raccolti ed elaborati...');
    
    N = length(ddtheta_filt_i);
    all_ddtheta_filt = [all_ddtheta_filt; ddtheta_filt_i];
    all_ddq_filt     = [all_ddq_filt;     ddq_filt_i];
    
    all_theta_meas = [all_theta_meas; simout.yout{1}.Values.Data(1:N)];
    all_q_pos      = [all_q_pos;      simout.yout{3}.Values.Data(1:N)];
    all_tau_J_meas = [all_tau_J_meas; simout.yout{5}.Values.Data(1:N)];
    all_tau_in     = [all_tau_in;     simout.yout{6}.Values.Data(1:N)];
end

%% 3. PLOT DEI RISULTATI (Mostra solo l'ultimo esperimento per chiarezza)
% Puoi commentare questa sezione se non vuoi vedere i grafici ogni volta
disp('Generazione dei grafici di verifica per l''ULTIMO esperimento...');
figure;
subplot(2,1,1);
plot(simout.yout{7}.Values.Data, simout.yout{1}.Values.Data, 'DisplayName', 'Misurata (Sim.)');
hold on;
plot(sim_data.theta_d(:,1), sim_data.theta_d(:,2), '--', 'DisplayName', 'Desiderata');
title('Posizione Motore (Ultimo Esperimento)');
xlabel('Tempo (s)'); ylabel('Posizione (rad)');
legend show; grid on;

subplot(2,1,2);
plot(simout.yout{7}.Values.Data, simout.yout{5}.Values.Data, 'DisplayName', 'Misurata (Sim.)');
hold on;
plot(sim_data.tau_Jd(:,1), sim_data.tau_Jd(:,2), '--', 'DisplayName', 'Desiderata');
title('Coppia al Giunto (Ultimo Esperimento)');
xlabel('Tempo (s)');
ylabel('Coppia (Nm)');
legend show; grid on;

%% 4. IDENTIFICAZIONE DEI PARAMETRI (Metodo Sequenziale e Robusto)
disp('--- Inizio processo di identificazione usando TUTTI i dati raccolti ---');

% --- Passo A: Stima della Rigidezza K (il parametro più affidabile) ---
disp('Stima della rigidezza K...');
K_stimato = (all_theta_meas - all_q_pos) \ all_tau_J_meas;
disp(['Valore "Reale" di K   : ', num2str(K)]);
disp(['--> Valore Stimato di K: ', num2str(K_stimato)]);
disp(' ');

% --- Passo B: Stima di M e Pg usando la K stimata ---
disp('Stima di M e Pg usando il K stimato...');
% Ricostruiamo la coppia al giunto usando K_stimato per avere un segnale più pulito
tau_J_reconstructed = K_stimato * (all_theta_meas - all_q_pos);
% Ora risolviamo tau_J_reconstructed = M*ddq + Pg*cos(q)
X_m_pg = [all_ddq_filt, cos(all_q_pos)];
parametri_M_Pg = X_m_pg \ tau_J_reconstructed;
M_stimato  = parametri_M_Pg(1);
Pg_stimato = parametri_M_Pg(2);
disp(['Valore "Reale" di M    : ', num2str(M)]);
disp(['--> Valore Stimato di M : ', num2str(M_stimato)]);
disp(['Valore "Reale" di Pg   : ', num2str(Pg)]);
disp(['--> Valore Stimato di Pg: ', num2str(Pg_stimato)]);
disp(' ');

% --- Passo C: Stima di Mm usando M_stimato e Pg_stimato ---
disp('Stima di Mm usando i parametri M e Pg appena stimati...');
% Ricostruiamo la coppia al giunto usando M_stimato e Pg_stimato
tau_J_reconstructed_final = M_stimato * all_ddq_filt + Pg_stimato * cos(all_q_pos);
% Ora risolviamo tau_in - tau_J_reconstructed = Mm*ddtheta
Y_mm = all_tau_in - tau_J_reconstructed_final;
X_mm = all_ddtheta_filt;
Mm_stimato = X_mm \ Y_mm;
disp(['Valore "Reale" di Mm   : ', num2str(Mm)]);
disp(['--> Valore Stimato di Mm: ', num2str(Mm_stimato)]);

disp('--- Processo di simulazione e identificazione completato ---');
