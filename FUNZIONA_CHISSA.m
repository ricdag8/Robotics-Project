% =========================================================================
% FASE 1: INIZIALIZZAZIONE E PARAMETRI
% =========================================================================
clear; clc; close all;

disp('Fase 1: Inizializzazione...');

% --- Parametri Fisici "Veri" del Sistema ---
% Questi sono i valori che il nostro modello Simulink userà.
% L'obiettivo dell'identificazione è stimare questi valori partendo dai dati.
Mm = 0.5;  % Inerzia del motore [kg*m^2]
M  = 2;   % Inerzia del link [kg*m^2]
K  = 500;  % Rigidezza del giunto [Nm/rad]

% --- Parametri di Simulazione ---
Ts = 0.001;      % Tempo di campionamento [s] (deve essere piccolo)
Tf = 30;         % Durata della simulazione [s]

disp('Parametri reali definiti.');
% =========================================================================
% FASE 2: CREAZIONE DEL SEGNALE DI INGRESSO (CHIRP)
% =========================================================================
disp('Fase 2: Creazione del segnale di ingresso...');

% --- Vettore Tempo ---
t = (0:Ts:Tf)';                 % Vettore colonna del tempo [s]
N = numel(t);

% --- Parametri Chirp ---
f0 = 0.1;                       % Frequenza di partenza [Hz]
f1 = 20;                        % Frequenza di arrivo [Hz]
% Nota: con i parametri Mm, M, K dati, la risonanza ~ 7–8 Hz.
% Il range 0.1–20 Hz copre bene la banda utile.

% --- Ampiezza dalla torsione desiderata ---
twist_max = 0.03;               % Torsione massima desiderata [rad] (es. 1.7°)
tau_amp   = K * twist_max;      % Coppia di picco corrispondente [Nm]

% --- Chirp base (sinusoidale a frequenza variabile) ---
tau_input = tau_amp * chirp(t, f0, Tf, f1, 'linear', 0);

% --- Tapering per evitare colpi di inizio/fine ---
alpha = 0.2;                    % 20% di rampa (fade-in/out)
w = tukeywin(N, alpha);         % Richiede Signal Processing Toolbox
tau_input = tau_input .* w;

% --- (Opzionale) Offset nullo e limitazione di sicurezza ---
tau_input = tau_input - mean(tau_input);         % rimuove eventuale bias numerico
tau_max_safe = 1.1 * tau_amp;                    % safety cap al 110% dell'ampiezza target
tau_input = max(min(tau_input, tau_max_safe), -tau_max_safe);

% --- Prepara il segnale per Simulink ---
% a) Formato matrice [tempo, segnale] per blocco From Workspace
input_signal = [t, tau_input];

% b) In alternativa, come timeseries:
tau_ts = timeseries(tau_input, t);

% --- Visualizza l'input (controllo di sanità) ---
figure('Name', 'Segnale di Ingresso');
plot(t, tau_input, 'LineWidth', 1.2);
title('Segnale di Coppia di Ingresso \tau(t) - Chirp con Tapering');
xlabel('Tempo [s]');
ylabel('Coppia \tau [Nm]');
grid on;

% --- Debug rapido: stampa valori chiave ---
fprintf('Chirp: f0 = %.2f Hz, f1 = %.2f Hz, twist_{max} = %.4f rad (%.2f°)\n', ...
        f0, f1, twist_max, twist_max*180/pi);
fprintf('Ampiezza di coppia impostata: tau_{amp} = %.2f Nm\n', tau_amp);

disp('Segnale di ingresso creato.');
% =========================================================================
% FASE 3: ESECUZIONE DELLA SIMULAZIONE (To Workspace -> Timeseries)
% =========================================================================
disp('Fase 3: Esecuzione della simulazione Simulink...');

mdl = 'modello_giunto';  % <-- cambia se necessario
open_system(mdl);

% (facoltativo) mostra i To Workspace trovati
tw_blocks = find_system(mdl, 'BlockType','ToWorkspace');
for k = 1:numel(tw_blocks)
    fprintf('ToWorkspace: %s | Var="%s" | Format="%s"\n', ...
        tw_blocks{k}, get_param(tw_blocks{k},'VariableName'), get_param(tw_blocks{k},'SaveFormat'));
end

% --- Esegui la simulazione
sim(mdl, 'StopTime', num2str(Tf));

% --- Recupera dal base workspace (To Workspace)
have_theta = evalin('base','exist(''theta'',''var'')==1');
have_tauJ  = evalin('base','exist(''tau_J'',''var'')==1');  % J maiuscola
have_tauj  = evalin('base','exist(''tau_j'',''var'')==1');  % j minuscola

if have_theta && (have_tauJ || have_tauj)
    theta_ws = evalin('base','theta');
    if have_tauJ
        tau_ws = evalin('base','tau_J');
        tau_name_used = 'tau_J';
    else
        tau_ws = evalin('base','tau_j');
        tau_name_used = 'tau_j';
    end

    % Estrai dati/tempo
    if isa(theta_ws,'timeseries')
        t_theta    = theta_ws.Time;    theta_data = theta_ws.Data;
    else
        t_theta    = theta_ws(:,1);    theta_data = theta_ws(:,2);
    end
    if isa(tau_ws,'timeseries')
        t_tau_J    = tau_ws.Time;      tau_J_data = tau_ws.Data;
    else
        t_tau_J    = tau_ws(:,1);      tau_J_data = tau_ws(:,2);
    end

    fprintf('Dati letti da To Workspace: theta e %s.\n', tau_name_used);

else
    % --- Fallback: Signal Logging (logsout)
    warning('To Workspace non trovato/compatibile: passo al Signal Logging (logsout).');
    set_param(mdl, 'SignalLogging', 'on');
    set_param(mdl, 'SignalLoggingName', 'logsout');

    simOut = sim(mdl, 'StopTime', num2str(Tf), 'SaveOutput','on');

    % prova prima 'tau_J' poi 'tau_j'
    try
        el_theta = simOut.logsout.getElement('theta');
        try
            el_tau = simOut.logsout.getElement('tau_J');
            tau_name_used = 'tau_J';
        catch
            el_tau = simOut.logsout.getElement('tau_j');
            tau_name_used = 'tau_j';
        end

        t_theta    = el_theta.Values.Time;  theta_data = el_theta.Values.Data;
        t_tau_J    = el_tau.Values.Time;    tau_J_data = el_tau.Values.Data;

        fprintf('Dati letti da logsout: theta e %s.\n', tau_name_used);
    catch
        error(['Fallback fallito: assicurati che i segnali abbiano nome "theta" e "tau_J" (o "tau_j") ', ...
               'e che siano marcati per il logging (clic destro sulla linea -> Log Selected Signals).']);
    end
end

% --- Plot rapido
figure('Name','Uscite Simulazione');
subplot(2,1,1); plot(t_theta, theta_data, 'LineWidth',1.2); grid on;
xlabel('Tempo [s]'); ylabel('\theta [rad]'); title('Posizione giunto \theta(t)');
subplot(2,1,2); plot(t_tau_J, tau_J_data, 'LineWidth',1.2); grid on;
xlabel('Tempo [s]'); ylabel('\tau_J [Nm]'); title('Coppia di giunto \tau_J(t)');


%% FASE 4: IDENTIFICAZIONE (CON CORREZIONE PER LA DERIVA)
disp('--- FASE 4: Identificazione dei parametri...');

% --- 1) Stima del modello τ -> τ_J (Questa parte andava già bene) ---
z_tauJ  = iddata(tau_J_data, tau_input, Ts);
z_tauJ  = detrend(z_tauJ, 0);

opt = tfestOptions;
opt.InitialCondition = 'estimate';


% --- Creazione di un filtro passabanda come WeightingFilter ---
f_low = 0.5;  % Frequenza inferiore in Hz
f_high = 18;  % Frequenza superiore in Hz

% Converti le frequenze in rad/s per le funzioni di design
w_low = f_low * 2*pi;
w_high = f_high * 2*pi;

% Progetta un filtro Butterworth del 2° ordine (leggero e stabile)
order = 2;
[num, den] = butter(order, [w_low, w_high], 'bandpass', 's');

% Crea l'oggetto Funzione di Trasferimento per il filtro
W = tf(num, den);

% Assegna il filtro come opzione di pesatura a tfest
opt.WeightingFilter = W;

G_tauJ = tfest(z_tauJ, 2, 0, opt); % 2 poli, 0 zeri

% --- 2) MODIFICA: Pre-elaborazione dati di posizione per ottenere la velocità ---
disp('Calcolo della velocità per eliminare la deriva...');
theta_fix = theta_data; % Assumiamo segno corretto, altrimenti invertire

% Calcola la velocità tramite differenze finite
vel_theta_data = diff(theta_fix) / Ts;

% Adatta la lunghezza degli altri vettori (diff riduce la lunghezza di 1)
tau_input_vel = tau_input(1:end-1);
t_vel = t(1:end-1);

% Crea il dataset iddata per la VELOCITÀ
z_vel = iddata(vel_theta_data, tau_input_vel, Ts);
z_vel = detrend(z_vel, 0); % Rimuove l'offset costante della velocità

% --- 3) MODIFICA: Stima del modello τ -> velocità (dθ/dt) ---
% Il modello teorico τ -> dθ/dt ha 3 poli e 2 zeri
disp('Stima del modello di velocità G_vel...');
G_vel = tfest(z_vel, 3, 2, opt);

% --- 4) Estrazione dei coefficienti dai DUE modelli STIMATI ---
[numJ, denJ] = tfdata(G_tauJ, 'v');
[numV, denV] = tfdata(G_vel, 'v'); % Coefficienti dal modello di velocità

% Dai modelli normalizzati (con coefficiente di grado massimo al den = 1):
% Nn e B1 vengono dal modello G_tauJ (che era affidabile)
Nn = numJ(end);   % ≈ K/Mm
B1 = denJ(end);   % ≈ K(M+Mm)/(M*Mm)

% A1 viene dal modello G_vel (che ora è affidabile)
% La FdT τ->dθ/dt è (Ms^2+K) / (s(MmMs^2 + K(M+Mm)))
% Normalizzata per s^3, il termine noto al numeratore è K/(M*Mm)
A1 = numV(end);   % ≈ K/(M*Mm)

% --- 5) Calcolo dei parametri fisici ---
M_est  = Nn / A1;
r      = Nn / B1;                 % Rapporto M/(M+Mm)
Mm_est = M_est * (1 - r) / r;
K_est  = Nn * Mm_est;

%% FASE 5: ANALISI DEI RISULTATI E VALIDAZIONE
disp('--- FASE 5: Analisi dei risultati...');

% --- Stampa a schermo il confronto ---
fprintf('\n=======================================================\n');
fprintf('           RISULTATI DELL''IDENTIFICAZIONE (Corretti)\n');
fprintf('-------------------------------------------------------\n');
fprintf('Parametro | Valore Reale | Valore Stimato\n');
fprintf('-------------------------------------------------------\n');
fprintf('   Mm     |    %.4f    |    %.4f\n', Mm, Mm_est);
fprintf('   M      |    %.4f    |    %.4f\n', M, M_est);
fprintf('   K      |   %.2f    |   %.2f\n', K, K_est);
fprintf('=======================================================\n\n');

% --- Validazione grafica ---
figure('Name','Validazione Grafica dei Modelli Stimati (Corretta)');
subplot(2,1,1);
compare(z_tauJ, G_tauJ); % Questo grafico dovrebbe essere ancora ottimo
title('\tau \rightarrow \tau_J (Dati vs. Modello Stimato)');
grid on;

subplot(2,1,2);
compare(z_vel, G_vel); % MODIFICA: Confrontiamo i dati di VELOCITÀ
title('\tau \rightarrow d\theta/dt (Dati vs. Modello Stimato)');
grid on;

disp('Processo completato.');