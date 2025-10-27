clc
clear all
close all;

%% Definizione dei Waypoint
% Ogni waypoint è definito da 7 elementi: 
% [x, y, z, qw, qx, qy, qz]
waypoints = [
0.295701 1.88624e-06 0.423674 0.707265 1.14721e-05 0.706949 7.49171e-06
0.296204 0.00147582 0.423585 0.707854 0.00149907 0.706354 -0.00202965
0.297667 0.00592832 0.423312 0.709599 0.00598175 0.704533 -0.0081255
0.299952 0.0134491 0.422835 0.712434 0.0135187 0.701372 -0.0182322
0.302819 0.0241794 0.422119 0.716246 0.0242064 0.696681 -0.0322656
0.305915 0.0382984 0.421118 0.720869 0.0381757 0.690203 -0.0500993
0.308762 0.0560026 0.419772 0.726083 0.0555868 0.68161 -0.0715554
0.310738 0.0774772 0.418007 0.731602 0.0766228 0.670519 -0.0963949
0.311064 0.102856 0.415739 0.737068 0.101481 0.656491 -0.124304
0.308794 0.132171 0.412872 0.742043 0.130359 0.639054 -0.154883
0.302819 0.165288 0.4093 0.746001 0.163445 0.617709 -0.187627
0.291879 0.201832 0.404908 0.748317 0.200894 0.591959 -0.221917
0.274606 0.241103 0.399575 0.748266 0.242806 0.561329 -0.257007
0.249598 0.281991 0.393174 0.745015 0.289202 0.525401 -0.292009
0.215544 0.322878 0.38558 0.73763 0.339974 0.483858 -0.325884
0.17504 0.358842 0.377384 0.726228 0.390629 0.440264 -0.355203
0.132153 0.387087 0.369231 0.711977 0.437498 0.398121 -0.378397
0.0887469 0.40805 0.361277 0.695698 0.480321 0.358145 -0.396269
0.0463806 0.4225 0.353655 0.678178 0.51896 0.320882 -0.409621
0.00628677 0.431411 0.346481 0.660156 0.553377 0.286733 -0.419229
-0.0306152 0.435856 0.339854 0.642304 0.583609 0.255973 -0.425821
-0.0636847 0.436922 0.333855 0.625226 0.60975 0.228782 -0.430065
-0.0925134 0.435645 0.328552 0.60945 0.631931 0.205261 -0.432552
-0.116873 0.432965 0.324 0.595426 0.6503 0.185457 -0.433801
-0.136665 0.429693 0.320242 0.58353 0.665006 0.16938 -0.434246
-0.151873 0.426502 0.317312 0.574063 0.67619 0.157016 -0.43424
-0.162525 0.423908 0.315236 0.567253 0.68397 0.148344 -0.434054
-0.168663 0.422278 0.31403 0.563261 0.688433 0.143341 -0.433878
];

%% 2. Generazione della Traiettoria Desiderata (tramite Spline)
% Creiamo una curva morbida che passa esattamente per le posizioni dei waypoint.

n_waypoints = size(waypoints, 1);
% Creiamo un parametro "tempo" fittizio per l'interpolazione (da 1 al numero di waypoint)
t_waypoints = 1:n_waypoints; 

% Creiamo un vettore di "tempo" più denso per avere una curva fluida
n_smooth_points = 500;
t_smooth = linspace(1, n_waypoints, n_smooth_points);

% Estraiamo le coordinate X, Y, Z dei waypoint
waypoints_xyz = waypoints(:, 1:3);

% Interpoliamo ogni coordinata usando una spline cubica
smooth_x = spline(t_waypoints, waypoints_xyz(:,1), t_smooth);
smooth_y = spline(t_waypoints, waypoints_xyz(:,2), t_smooth);
smooth_z = spline(t_waypoints, waypoints_xyz(:,3), t_smooth);

% Uniamo le coordinate interpolate per formare la traiettoria desiderata
desired_trajectory = [smooth_x', smooth_y', smooth_z'];


%% 3. Definizione della Traiettoria del Robot (Simulata)
% Questa parte rimane invariata. Quando avrai i dati reali, 
% sostituirai questa sezione caricando i tuoi dati in una variabile
% con lo stesso nome 'robot_trajectory'.
% robot_trajectory = [];
% for i = 1:(n_waypoints - 1)
%     start_point = waypoints(i, 1:3);
%     end_point = waypoints(i+1, 1:3);
%     segment = zeros(20, 3);
%     for j = 1:3
%         segment(:, j) = linspace(start_point(j), end_point(j), 20);
%     end
%     robot_trajectory = [robot_trajectory; segment];
% end
% noise = 0.003 * (rand(size(robot_trajectory)) - 0.5);
% robot_trajectory = robot_trajectory + noise;

%ESCE TUTTO STORTO
% actual_waypoints = [
% 0.296 0.000 0.425 0.71 0.00 0.71 0.00
% 0.296 -0.000 0.413 0.71 -0.00 0.71 0.00
% 0.295 0.000 0.409 0.71 0.00 0.71 -0.00
% 0.294 0.002 0.424 0.71 0.00 0.71 -0.00
% 0.299 0.029 0.438 0.72 0.03 0.69 -0.04
% 0.301 0.028 0.424 0.72 0.03 0.69 -0.04
% 0.308 0.085 0.422 0.73 0.08 0.67 -0.10
% 0.309 0.084 0.437 0.73 0.08 0.67 -0.11
% 0.314 0.129 0.424 0.74 0.12 0.64 -0.15
% 0.320 0.132 0.416 0.74 0.13 0.64 -0.15
% 0.320 0.098 0.419 0.74 0.10 0.66 -0.12
% 0.310 0.066 0.429 0.73 0.07 0.67 -0.09
% 0.302 0.065 0.433 0.73 0.07 0.67 -0.09
% 0.303 0.088 0.430 0.73 0.09 0.66 -0.12
% 0.308 0.117 0.422 0.74 0.12 0.65 -0.15
% 0.309 0.146 0.417 0.74 0.15 0.63 -0.17
% 0.299 0.162 0.415 0.75 0.16 0.62 -0.18
% 0.291 0.204 0.412 0.75 0.21 0.59 -0.23
% 0.262 0.253 0.404 0.75 0.25 0.55 -0.26
% 0.242 0.286 0.401 0.74 0.30 0.52 -0.30
% 0.193 0.338 0.387 0.73 0.36 0.47 -0.34
% 0.129 0.380 0.382 0.71 0.43 0.39 -0.40
% 0.098 0.400 0.368 0.70 0.47 0.37 -0.39
% -0.011 0.430 0.348 0.65 0.57 0.28 -0.42
% -0.059 0.426 0.356 0.62 0.60 0.21 -0.46
% -0.088 0.433 0.331 0.61 0.63 0.21 -0.43
% -0.233 0.388 0.326 0.50 0.72 0.04 -0.47
% -0.292 0.368 0.294 0.46 0.77 -0.00 -0.44
% -0.232 0.417 0.283 0.52 0.74 0.09 -0.42
% -0.199 0.419 0.2 0.52 0.74 0.09 -0.42
% -0.178 0.422 0.2 0.50 0.75 0.09 -0.40
% ];

%Soglia norma errore 0.001
actual_waypoints = [
0.296, -0.000, 0.423 0.71, 0.00, 0.71 0.00
0.296, 0.001, 0.423  0.71, 0.00, 0.71 -0.00
0.298, 0.007, 0.424 0.71, 0.01, 0.70 -0.01
0.301, 0.017, 0.424 0.71, 0.02, 0.70 -0.02
0.304, 0.031, 0.423 0.72, 0.03, 0.69 -0.04
0.307, 0.047, 0.422 0.72, 0.05, 0.69 -0.06
0.309, 0.067, 0.421 0.73, 0.07, 0.68 -0.08
0.310, 0.092, 0.419 0.73, 0.09, 0.66 -0.11
0.308, 0.121, 0.416 0.74, 0.12, 0.65 -0.14
0.304, 0.153, 0.413 0.74, 0.15, 0.63 -0.17
0.295, 0.188, 0.409 0.75, 0.19, 0.60 -0.21
0.281, 0.224, 0.405 0.75, 0.22, 0.58 -0.24
0.262, 0.259, 0.399 0.75, 0.26, 0.55 -0.27
0.237, 0.295, 0.394 0.74, 0.31, 0.51 -0.30
0.207, 0.328, 0.387 0.74, 0.35, 0.48 -0.33
0.173, 0.358, 0.380 0.73, 0.39, 0.44 -0.35
0.136, 0.383, 0.373 0.71, 0.43, 0.40 -0.38
0.097, 0.403, 0.365 0.70, 0.47, 0.37 -0.39
0.057, 0.418, 0.358 0.68, 0.51, 0.33 -0.41
0.018, 0.428, 0.350 0.67, 0.54, 0.30 -0.42
-0.021, 0.434, 0.343 0.65, 0.58, 0.26 -0.42
-0.062, 0.436, 0.336 0.63, 0.61, 0.23 -0.43
-0.095, 0.434, 0.330 0.61, 0.63, 0.20 -0.43
-0.123, 0.431, 0.324 0.59, 0.65, 0.18 -0.43
-0.148, 0.427, 0.319 0.58, 0.67, 0.16 -0.43
-0.157, 0.424, 0.317 0.58, 0.68, 0.15 -0.44
-0.157, 0.424, 0.318 0.59, 0.67, 0.14 -0.44
-0.157, 0.424, 0.320 0.59, 0.66, 0.14 -0.44
-0.157, 0.423, 0.321 0.60, 0.65, 0.13 -0.44
-0.157, 0.423, 0.322 0.61, 0.64, 0.13 -0.45
];

n_actual_waypoint = size(actual_waypoints, 1);
% Creiamo un parametro "tempo" fittizio per l'interpolazione (da 1 al numero di waypoint)
t_ac_waypoints = 1:n_actual_waypoint; 

% Creiamo un vettore di "tempo" più denso per avere una curva fluida
n_ac_smooth_points = 500;
t_ac_smooth = linspace(1, n_actual_waypoint, n_ac_smooth_points);

% Estraiamo le coordinate X, Y, Z dei waypoint
act_waypoints_xyz = actual_waypoints(:, 1:3);

% Interpoliamo ogni coordinata usando una spline cubica
smooth_x_act = spline(t_ac_waypoints, act_waypoints_xyz(:,1), t_ac_smooth);
smooth_y_act = spline(t_ac_waypoints, act_waypoints_xyz(:,2), t_ac_smooth);
smooth_z_act = spline(t_ac_waypoints, act_waypoints_xyz(:,3), t_ac_smooth);

% Uniamo le coordinate interpolate per formare la traiettoria desiderata
robot_trajectory = [smooth_x_act', smooth_y_act', smooth_z_act'];




%% Plot 3D
figure('Name', 'Confronto Traiettorie 3D', 'NumberTitle', 'off');
hold on;
grid on;
axis equal;

% Plot della traiettoria ESEGUITA dal robot (linea blu)
plot3(robot_trajectory(:,1), robot_trajectory(:,2), robot_trajectory(:,3), ...
      'b-', 'LineWidth', 1, 'DisplayName', 'Trajectory followed by the robot');

% Plot della traiettoria DESIDERATA (linea nera)
plot3(desired_trajectory(:,1), desired_trajectory(:,2), desired_trajectory(:,3), ...
      'k-', 'LineWidth', 1, 'DisplayName', 'Desired trajectory');

% % Plot dei WAYPOINT (pallini rossi)
% plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), ...
%       'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Waypoint');

% % Plot dei WAYPOINT ACTUAL (pallini verdi)
% plot3(actual_waypoints(:,1), actual_waypoints(:,2), actual_waypoints(:,3), ...
%       'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Waypoint');

% Impostazioni del grafico
title('Comparison between Desired and Executed Trajectory');
xlabel('Asse X (m)');
ylabel('Asse Y (m)');
zlabel('Asse Z (m)');
legend('show', 'Location', 'best');
view(30, 25);
hold off;