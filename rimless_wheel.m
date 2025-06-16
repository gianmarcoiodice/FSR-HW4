clc
close all
clear all

% Parameters
g = 9.81;          % gravity (m/s^2)
l = 0.5;           % leg length (m) %%MODIFY HERE%% 
alpha = pi/8;      % half inter-leg angle (rad) %%MODIFY HERE%% 
gamma = 0.08;      % slope angle (rad) %%MODIFY HERE%% 

%Initial conditions
thetadot0 =0.95;%%MODIFY HERE%% 
if (thetadot0 >= 0)
    theta0 = gamma-alpha;
else
    theta0 = gamma+alpha;
end
double_support = 0;

y0 = [theta0; thetadot0];

% Simulation settings
t0 = 0; %initial time
tf = 25; %final time
dt = 0.01; %max step time

% Time/state storage
T = [];
Y = [];

while t0 < tf
    options = odeset('Events', @(t, y) impact_event(t, y, alpha,gamma), 'MaxStep', dt);
    [t, y, te, ye, ie] = ode45(@(t, y) dynamics(t, y, g, l, double_support), [t0 tf], y0, options);
    
    T = [T; t];
    Y = [Y; y];

    if ~isempty(te)
        [y0,double_support] = impact_map(ye, alpha,g,l); % apply impact map
        t0 = te;
    else
        break;
    end
end

% Plot results
figure(1)
plot(T, Y(:,1), 'b', 'DisplayName', '\theta (rad)');
hold on;
plot(T, Y(:,2), 'r', 'DisplayName', '\theta dot (rad/s)');
xlabel('Time (s)');
ylabel('State');
title('Rimless Wheel Dynamics');
legend show;
grid on;

% exportgraphics(gcf, 'rimless_dynamic_limit_cycle.pdf', 'ContentType', 'vector');


figure(2)
plot(Y(:,1), Y(:,2), 'b', 'DisplayName', '\theta (rad)');
hold on
plot(Y(1,1), Y(1,2), 'r', 'Marker','*','LineWidth',5,'DisplayName','Initial point');
xlabel('\theta (rad)');
ylabel('\theta dot (rad/s)');
title('Rimless Wheel Limit Cycle');
legend show;
grid on;

% exportgraphics(gcf, 'rimless_limit_cycle.pdf', 'ContentType', 'vector');
% 
%%
% % === SCANSIONE CONDIZIONI INIZIALI PER TROVARE EQUILIBRI ===
% theta_vals = linspace(gamma - alpha - 0.3, gamma + alpha + 0.3, 100);
% thetadot_vals = linspace(-2, 2, 100);
% 
% found_equilibria = [];
% 
% for theta0_scan = theta_vals
%     for thetadot0_scan = thetadot_vals
%         y0 = [theta0_scan; thetadot0_scan];
%         t0 = 0;
%         double_support = 0;
% 
%         for step = 1:50
%             options = odeset('Events', @(t, y) impact_event(t, y, alpha, gamma), 'MaxStep', 0.01);
%             [~, ~, te, ye, ~] = ode45(@(t, y) dynamics(t, y, g, l, double_support), [t0 10], y0, options);
%             if isempty(te)
%                 break;
%             end
%             [y0, double_support] = impact_map(ye, alpha, g, l);
%             t0 = te;
% 
%             if double_support
%                 break;
%             end
%         end
% 
%         if double_support && abs(y0(2)) < 0.01
%             found_equilibria = [found_equilibria, y0];
%         end
%     end
% end
% 
% % Rimuovi duplicati (approssimativamente)
% found_equilibria = unique(round(found_equilibria', 3), 'rows');
% 
% % Mostra risultati
% disp('Punti di equilibrio trovati:');
% disp(found_equilibria);
% 
% % Visualizza
% figure;
% scatter(found_equilibria(:,1), found_equilibria(:,2), 50, 'filled');
% xlabel('\theta (rad)');
% ylabel('\theta dot (rad/s)');
% title('Punti di equilibrio trovati per diverse condizioni iniziali');
% grid on;
% 


%%
%SCRIPT BASE D'ATTRAZIONE PER PUNTO A
theta1 = -0.3126;  % per gamma = 0.08
theta2 = 0.4726;
N_check = 5;
thetadot_range = linspace(-2, 2, 150);
limit_states = [];
limit_cycle_periods = NaN(size(thetadot_range));  % inizializza con NaN
limit_amplitudes = NaN(size(thetadot_range));  % Inizializza array

basin = zeros(size(thetadot_range));
impact_matrix = zeros(size(thetadot_range));

for i = 1:length(thetadot_range)
    thetadot0 = thetadot_range(i);

    % Calcolo di theta0 in base al segno di thetadot0
    if thetadot0 >= 0
        theta0 = gamma - alpha;
    else
        theta0 = gamma + alpha;
    end
    y0 = [theta0; thetadot0];
    t0 = 0; tf = 10; dt = 0.01;
    double_support = 0;
    impact_states = [];
    impact_count = 0;
    impact_times_all = [];
    theta_history_full = [];  % per raccogliere θ(t)

    for step = 1:30
        options = odeset('Events', @(t, y) impact_event(t, y, alpha, gamma), 'MaxStep', dt);
        [t, y, te, ye, ~] = ode45(@(t, y) dynamics(t, y, g, l, double_support), [t0 tf], y0, options);

        if isempty(te)
            break;
        end
        theta_history_full = [theta_history_full; y(:,1)];  % accumula θ(t)
        impact_times_all = [impact_times_all; te];


        [y0, double_support] = impact_map(ye, alpha, g, l);
        impact_count = impact_count + 1;

        impact_states = [impact_states, y0];
        if size(impact_states, 2) > N_check
            impact_states = impact_states(:, end-N_check+1:end);
        end

        t0 = te;

        if double_support
            break;
        end
    end

    impact_matrix(i) = impact_count;

    % Classificazione finale senza tolleranze esplicite
    if double_support && abs(y0(2)) < 1e-3  % condizione minima per considerare fermo
        if abs(y0(1) - theta1) < abs(y0(1) - theta2)
            basin(i) = 1;  % più vicino a theta1
        else
            basin(i) = 2;  % più vicino a theta2
        end
else
    basin(i) = 3;  % limit cycle
    limit_states = [limit_states, y0(:)];

    if isempty(theta_history_full)
        ampiezza = NaN;
    else
        ampiezza = max(theta_history_full(:)) - min(theta_history_full(:));
    end
    limit_amplitudes(i) = ampiezza;  

    if length(impact_times_all) >= N_check + 1
        recent_impacts = impact_times_all(end - N_check : end);
        T = mean(diff(recent_impacts));
    else
        T = NaN;
    end
    limit_cycle_periods(i) = T;

    % fprintf('Limit cycle at i = %d: Ampiezza = %.4f rad, T = %.5f s\n', i, ampiezza, T);
    end
end


    if length(impact_times_all) >= N_check + 1
        recent_impacts = impact_times_all(end - N_check : end);
        T = mean(diff(recent_impacts));
    else
        T = NaN;
    end

    limit_cycle_periods(i) = T;

%    fprintf('Limit cycle at i = %d: Ampiezza = %.4f rad, T = %.5f s\n', i, ampiezza, T);



% Estrai solo i cicli validi (dove T è definito)
valid_idx = ~isnan(limit_cycle_periods) & ~isnan(limit_amplitudes);
T_all = limit_cycle_periods(valid_idx);
A_all = limit_amplitudes(valid_idx);

% Parametri di tolleranza (1%)
tol_rel = 0.01;

% Inizializza cluster
clustered_T = [];
clustered_A = [];
cluster_labels = zeros(size(T_all));

n_clusters = 0;

for i = 1:length(T_all)
    T_i = T_all(i);
    A_i = A_all(i);
    
    found = false;
    
    for j = 1:n_clusters
        % confronta con cluster esistente j
        rel_diff_T = abs(clustered_T(j) - T_i) / clustered_T(j);
        rel_diff_A = abs(clustered_A(j) - A_i) / clustered_A(j);
        
        if rel_diff_T < tol_rel && rel_diff_A < tol_rel
            cluster_labels(i) = j;
            found = true;
            break;
        end
    end
    
    if ~found
        % nuovo cluster
        n_clusters = n_clusters + 1;
        clustered_T(n_clusters) = T_i;
        clustered_A(n_clusters) = A_i;
        cluster_labels(i) = n_clusters;
    end
end

fprintf('Numero di cicli limite distinti (T ±1%%, Ampiezza ±1%%): %d\n', n_clusters);


% === Visualizzazione base di attrazione rispetto a thetadot ===
figure;
plot(thetadot_range, basin, 'LineWidth', 1.5);
ylim([0 3.5]);
yticks([1 2 3]);
yticklabels({'Equilibrium 1', 'Equilibrium 2', 'Limit Cycle'});

xlabel('$\dot{\theta}_0$ (rad/s)', 'Interpreter', 'latex');
ylabel('Attractor', 'Interpreter', 'latex');
title('Attractor as a function of $\dot{\theta}_0$ (with $\theta_0$ selected by sign)', ...
      'Interpreter', 'latex');

grid on;
xlim([min(thetadot_range), max(thetadot_range)]);

% exportgraphics(gcf, 'basin_of_attraction_vs_thetadot.pdf', 'ContentType', 'vector');

% === Numero di impatti ===
figure;
plot(thetadot_range, impact_matrix, 'LineWidth', 2, 'Color', [0.2 0.4 0.8]);

xlabel('$\dot{\theta}_0$ (rad/s)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('Number of impacts', 'Interpreter', 'latex', 'FontSize', 14);

title(sprintf(['Number of impacts for $\\theta_0 = \\gamma \\pm \\alpha$ ', ...
               '(depending on sign of $\\dot{\\theta}_0$), $\\gamma = %.2f$ rad, ' ...
               '$\\alpha = %.2f$ rad, $\\ell = %.2f$ m'], gamma, alpha, l), ...
      'Interpreter', 'latex', 'FontSize', 9);

grid on;
xlim([min(thetadot_range), max(thetadot_range)]);
ylim([0, max(impact_matrix(:)) + 1]);

% exportgraphics(gcf, 'n_impacts_vs_thetadot.pdf', 'ContentType', 'vector');


%%

alpha_range = linspace(2.8*pi/180, 30*pi/180, 300);  % alpha da 2.8° a 30°
basin = zeros(size(alpha_range));
impact_vector = zeros(size(alpha_range));
limit_states = [];
limit_cycle_periods = NaN(size(alpha_range));
limit_amplitudes = NaN(size(alpha_range));

N_check = 5;

for i = 1:length(alpha_range)
    alpha = alpha_range(i);
    theta0 = gamma - alpha;
    y0 = [theta0; thetadot0];

    t0 = 0; tf = 10; dt = 0.01;
    double_support = 0;
    impact_count = 0;
    impact_states = [];
    impact_times_all = [];
    theta_history_full = [];

    for step = 1:30
        options = odeset('Events', @(t, y) impact_event(t, y, alpha, gamma), 'MaxStep', dt);
        [t, y, te, ye, ~] = ode45(@(t, y) dynamics(t, y, g, l, double_support), [t0 tf], y0, options);

        if isempty(te)
            break;
        end

        theta_history_full = [theta_history_full; y(:,1)];
        impact_times_all = [impact_times_all; te];

        [y0, double_support] = impact_map(ye, alpha, g, l);
        impact_count = impact_count + 1;

        impact_states = [impact_states, y0];
        if size(impact_states, 2) > N_check
            impact_states = impact_states(:, end - N_check + 1:end);
        end

        t0 = te;

        if double_support
            break;
        end
    end

    impact_vector(i) = impact_count;

    if double_support && abs(y0(2)) < 1e-3
        if y0(1) < 0
            basin(i) = 1;
        else
            basin(i) = 2;
        end
    else
        basin(i) = 3;
        limit_states = [limit_states, y0(:)];

        if isempty(theta_history_full)
            ampiezza = NaN;
        else
            ampiezza = max(theta_history_full(:)) - min(theta_history_full(:));
        end
        limit_amplitudes(i) = ampiezza;

        if length(impact_times_all) >= N_check + 1
            recent_impacts = impact_times_all(end - N_check : end);
            T = mean(diff(recent_impacts));
        else
            T = NaN;
        end
        limit_cycle_periods(i) = T;
        fprintf('Limit cycle at i = %d: Ampiezza = %.4f rad, T = %.5f s\n', i, ampiezza, T);
    end
end

% === Calcolo numero cicli limite distinti ===
valid_idx = ~isnan(limit_cycle_periods) & ~isnan(limit_amplitudes);
T_all = limit_cycle_periods(valid_idx);
A_all = limit_amplitudes(valid_idx);

tol_rel = 0.10;
clustered_T = [];
clustered_A = [];
cluster_labels = zeros(size(T_all));
n_clusters = 0;

for i = 1:length(T_all)
    T_i = T_all(i);
    A_i = A_all(i);
    found = false;

    for j = 1:n_clusters
        rel_diff_T = abs(clustered_T(j) - T_i) / clustered_T(j);
        rel_diff_A = abs(clustered_A(j) - A_i) / clustered_A(j);
        if rel_diff_T < tol_rel && rel_diff_A < tol_rel
            cluster_labels(i) = j;
            found = true;
            break;
        end
    end

    if ~found
        n_clusters = n_clusters + 1;
        clustered_T(n_clusters) = T_i;
        clustered_A(n_clusters) = A_i;
        cluster_labels(i) = n_clusters;
    end
end

fprintf('Numero di cicli limite distinti (T ±1%%, Ampiezza ±1%%): %d\n', n_clusters);

%%
figure;
plot(alpha_range, basin, 'LineWidth', 1.5);
ylim([0 3.5]);
yticks([1 2 3]);
yticklabels({'Equilibrium 1', 'Equilibrium 2', 'Limit Cycle'});
xlabel('$\alpha$ (rad)', 'Interpreter', 'latex');
ylabel('Attrattore', 'Interpreter', 'latex');
title(sprintf('Attractor as function of $\\alpha$ with $\\dot{\\theta}_0 = %.2f$ rad/s', thetadot0), ...
      'Interpreter', 'latex');
grid on;
xlim([min(alpha_range), max(alpha_range)]);

% Esporta in PDF
exportgraphics(gcf, sprintf('basin_of_attraction_alpha_var_gamma_0.04%.2f.pdf', gamma), 'ContentType', 'vector');

figure;
plot(alpha_range, impact_vector, 'LineWidth', 2, 'Color', [0.2 0.4 0.8]);

xlabel('$\alpha$ (rad)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('Numero di impatti', 'Interpreter', 'latex', 'FontSize', 14);

title(sprintf(['Number of impacts for $\\dot{\\theta}_0 = %.2f$ rad/s, $\\gamma = %.2f$ rad, $\\ell = %.2f$ m'], ...
               thetadot0, gamma, l), ...
      'Interpreter', 'latex', 'FontSize', 11);

grid on;
xlim([min(alpha_range), max(alpha_range)]);
ylim([0, max(impact_vector) + 1]);

% Esporta in PDF
exportgraphics(gcf, sprintf('numero_impatti_vs_alpha_var_gamma_0.04%.2f.pdf', gamma), 'ContentType', 'vector');


%% Basin of attraction al variare di L
%
N_check = 5;
% Condizioni iniziali: esplora solo theta0, con thetadot0 fisso
theta_range = linspace(gamma - alpha - 0.2, gamma + alpha + 0.2, 500);  % molto fitto
basin = zeros(size(theta_range));  % 1=eq1, 2=eq2, 3=limit cycle
impact_vector = zeros(size(theta_range));
limit_states = [];
limit_periods = [];
limit_amplitudes = [];

for i = 1:length(theta_range)
    y0 = [theta_range(i); thetadot0];
    t0 = 0; tf = 10; dt = 0.01;
    double_support = 0;
    impact_count = 0;
    impact_states = [];
    impact_times = [];
    theta_history_full = [];

    for step = 1:30
        options = odeset('Events', @(t, y) impact_event(t, y, alpha, gamma), 'MaxStep', dt);
        [t, y, te, ye, ~] = ode45(@(t, y) dynamics(t, y, g, l, double_support), [t0 tf], y0, options);

        if isempty(te)
            break;
        end

        theta_history_full = [theta_history_full; y(:,1)];
        impact_times = [impact_times, te];
        [y0, double_support] = impact_map(ye, alpha, g, l);
        impact_count = impact_count + 1;

        impact_states = [impact_states, y0];
        if size(impact_states, 2) > N_check
            impact_states = impact_states(:, end-N_check+1:end);
        end

        t0 = te;

        if double_support
            break;
        end
    end

    impact_vector(i) = impact_count;


    if double_support && abs(y0(2)) < 1e-3
        if y0(1) < 0
            basin(i) = 1;
        else
            basin(i) = 2;
        end
    else
        basin(i) = 3;  % ciclo limite
        limit_states = [limit_states, y0];

        if isempty(theta_history_full)
            ampiezza = NaN;
        else
            ampiezza = max(theta_history_full(:)) - min(theta_history_full(:));
        end
        limit_amplitudes = [limit_amplitudes, ampiezza];

        if length(impact_times) >= N_check + 1
            T = mean(diff(impact_times(end - N_check : end)));
        else
            T = NaN;
        end
        limit_periods = [limit_periods, T];
        %fprintf('Limit cycle at i = %d: Ampiezza = %.4f rad, T = %.5f s\n', i, ampiezza, T);

    end
end

% === Cluster cicli limite distinti (±1%) ===
tol_rel = 0.01;
T_all = limit_periods(~isnan(limit_periods));
A_all = limit_amplitudes(~isnan(limit_amplitudes));

clustered_T = [];
clustered_A = [];
cluster_labels = zeros(size(T_all));
n_clusters = 0;

for i = 1:length(T_all)
    T_i = T_all(i);
    A_i = A_all(i);
    
    found = false;
    for j = 1:n_clusters
        rel_diff_T = abs(clustered_T(j) - T_i) / clustered_T(j);
        rel_diff_A = abs(clustered_A(j) - A_i) / clustered_A(j);
        
        if rel_diff_T < tol_rel && rel_diff_A < tol_rel
            cluster_labels(i) = j;
            found = true;
            break;
        end
    end
    
    if ~found
        n_clusters = n_clusters + 1;
        clustered_T(n_clusters) = T_i;
        clustered_A(n_clusters) = A_i;
        cluster_labels(i) = n_clusters;
    end
end

fprintf('Estimated number of distinct limit cycles (T ±1%%, Ampiezza ±1%%): %d\n', n_clusters);




%% Visualizzazione
figure;
plot(theta_range, basin, 'LineWidth', 1.5);
ylim([0 3.5]);
yticks([1 2 3]);
yticklabels({'Equilibrium 1', 'Equilibrium 2', 'Limit Cycle'});
xlabel('$\theta_0$ (rad)', 'Interpreter', 'latex');
ylabel('Attrattore', 'Interpreter', 'latex');
title('Attractor in function of $\theta_0$ with $\dot{\theta}_0 = 0.95$ rad/s', 'Interpreter', 'latex');
grid on;
xlim([min(theta_range), max(theta_range)]);


exportgraphics(gcf, 'basin_of_attraction_con_limit_cycle_alpha_pi_8_L_1.5_gamma_0.08.pdf', 'ContentType', 'vector');


figure(4);
plot(theta_range, impact_vector, 'LineWidth', 2, 'Color', [0.2 0.4 0.8]);

% Etichette asse x e y con LaTeX
xlabel('$\theta_0$ (rad)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('Numero di impatti', 'Interpreter', 'latex', 'FontSize', 14);

title(sprintf(['Number of impacts for $\\dot{\\theta}_0 = %.2f$ rad/s, $\\gamma = %.2f$ rad, ' ...
               '$\\alpha = %.2f$ rad, $\\ell = %.2f$ m'], ...
               0.95, gamma, alpha, l), ...
      'Interpreter', 'latex', 'FontSize', 11);



% Miglioramenti estetici
grid on;
xlim([min(theta_range), max(theta_range)]);
ylim([0, max(impact_vector) + 1]);

exportgraphics(gcf, 'numeroimpatti_alpha_pi_8_L_1.5_gamma_0.08.pdf', 'ContentType', 'vector');








%%


function dydt = dynamics(~, y, g, l, ds)
    theta = y(1);
    thetadot = y(2);
    if (~ds)
        dtheta = thetadot;
        dthetadot = (g/l) * sin(theta);
    else
        dtheta = 0;
        dthetadot = 0;
    end
    dydt = [dtheta; dthetadot];
end

function [value, isterminal, direction] = impact_event(~, y, alpha,gamma)
    
    value = [y(1)-alpha-gamma; y(1)-gamma+alpha];% Trigger when theta = gamma+alpha
                                     %Trigger when theta = gamma-alpha
    isterminal = [1;1];         % Stop the integration
    direction = [1;-1];          % Detect only when increasing
end

function [yplus,ds] = impact_map(y_minus, alpha,g,l)%minus: before impact time; plus: after impact time
    if (y_minus(2)>=0)
        theta_plus = y_minus(1)-2*alpha;
    else
        theta_plus = y_minus(1)+2*alpha;
    end
    thetadot_plus = cos(2*alpha) * y_minus(2);
    if (thetadot_plus < 0.01*sqrt(g/l) && thetadot_plus >-0.01*sqrt(g/l)) 
        thetadot_plus = 0;
        ds = 1;
    else
        ds = 0;
    end
    yplus = [theta_plus; thetadot_plus];
end