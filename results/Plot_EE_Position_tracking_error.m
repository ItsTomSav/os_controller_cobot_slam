clear;
clc;
close all;

%% Dati dell'errore e del tempo
% Vettore con i 27 valori di errore di posizione
errori_posizione = [
    0.0008, 0.0003, 0.0023, 0.0016, 0.0013, 0.0013, 0.0013, ...
    0.0012, 0.0011, 0.0100, 0.0106, 0.0113, 0.0017, 0.0029, ...
    0.0043, 0.0057, 0.0068, 0.0075, 0.0075, 0.0070, 0.0062, ...
    0.0049, 0.0034, 0.0019, 0.0019, 0.0033, 0.0011
];

% Vettore con i 27 timestamp (in secondi).
timestamp = [
    1750772600.111212508,
    1750772609.491283747,
    1750772613.753580732,
    1750772627.776902816,
    1750772641.993462163,
    1750772656.018970523,
    1750772669.573805579,
    1750772682.438557362,
    1750772685.362277485,
    1750772694.362277485,
    1750772706.880090709,
    1750772721.337677705,
    1750772734.661652036,
    1750772747.705907148,
    1750772760.648642978,
    1750772773.736359509,
    1750772786.659239108,
    1750772799.222057649,
    1750772813.581655100,
    1750772825.656389492,
    1750772838.506001568,
    1750772850.816087324,
    1750772862.781791756,
    1750772874.928921506,
    1750772886.844494834,
    1750772899.063300128,
    1750772911.638548052
];

% Calcoliamo il tempo relativo per far partire il grafico da t=0
tempo_iniziale = timestamp(1);
tempo_relativo_s = timestamp - tempo_iniziale;


%% Plot
figure('Name', 'Norm of the position tracking error of the end-effector', 'NumberTitle', 'off');

plot(tempo_relativo_s, errori_posizione, 'r-s', ... % 'r-s' = linea rossa con quadrati
    'LineWidth', 2, ...
    'MarkerSize', 8, ...
    'MarkerFaceColor', 'r');

title('Evolution of the error norm over time');
xlabel('Time (s)');
ylabel('e_p = ||p_d_e_s - p_a_c_t||_2');
grid on;

xlim([0, max(tempo_relativo_s)]);
ylim([0, max(errori_posizione) * 1.1]);