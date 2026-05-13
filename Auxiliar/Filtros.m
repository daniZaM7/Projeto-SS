% -------------------------------------------------------------------------
% Projeto: Sistema de Controlo de Acesso via Tons
% Autores: Daniel Zamurca (118799) e Nicolas Sousa (119744) - Turma P3
% Script: Gerador de Coeficientes FIR e Gráfico de Frequência
% -------------------------------------------------------------------------

clear all; clc; close all;
pkg load signal; % (Deixa esta linha se estiveres no Octave)

%% 1. Parâmetros do Sistema
Fs = 8000;              % Frequência de amostragem (Hz)
N_taps = 128;           % Tamanho do filtro (128 amostras)
order = N_taps - 1;     % Ordem do filtro FIR (127)
BW = 100;               % Largura de banda (+/- 100 Hz)

% As frequências da Turma P3 e NMECs
f0 = 1500;
f1 = 2220;
f2 = 2940;

%% 2. Cálculo dos Filtros FIR (Com janela de Hamming)
h0 = fir1(order, [f0-BW, f0+BW]/(Fs/2), 'bandpass');
h1 = fir1(order, [f1-BW, f1+BW]/(Fs/2), 'bandpass');
h2 = fir1(order, [f2-BW, f2+BW]/(Fs/2), 'bandpass');

%% 3. Exportar Código C para o Terminal
fprintf('\n// --- COPIAR ESTES ARRAYS PARA O TOPO DO MAIN.C ---\n');
fprintf('#define N_TAPS %d\n\n', N_taps);

% Tom 0
fprintf('__attribute__((aligned(16))) const float filtro_tom0[%d] = {', N_taps);
fprintf('%f, ', h0(1:end-1)); fprintf('%f};\n\n', h0(end));

% Tom 1
fprintf('__attribute__((aligned(16))) const float filtro_tom1[%d] = {', N_taps);
fprintf('%f, ', h1(1:end-1)); fprintf('%f};\n\n', h1(end));

% Tom 2
fprintf('__attribute__((aligned(16))) const float filtro_tom2[%d] = {', N_taps);
fprintf('%f, ', h2(1:end-1)); fprintf('%f};\n', h2(end));
fprintf('// -----------------------------------------------------\n\n');

%% 4. Visualização: Resposta em Frequência (Para o Relatório)
figure('Position', [100, 100, 800, 500]);

% Calcular a resposta em frequência dos 3 filtros
[H0, f_plot] = freqz(h0, 1, 2048, Fs);
[H1, ~]      = freqz(h1, 1, 2048, Fs);
[H2, ~]      = freqz(h2, 1, 2048, Fs);

% Desenhar o gráfico de Magnitude (dB)
plot(f_plot, 20*log10(abs(H0)), 'b', 'LineWidth', 1.5); hold on;
plot(f_plot, 20*log10(abs(H1)), 'r', 'LineWidth', 1.5);
plot(f_plot, 20*log10(abs(H2)), 'g', 'LineWidth', 1.5);

% Formatação do gráfico
grid on;
xlabel('Frequência (Hz)');
ylabel('Magnitude (dB)');
title('Resposta em Frequência dos Filtros Passa-Banda (Janela de Hamming)');
legend('Tom 0 (1500 Hz)', 'Tom 1 (2220 Hz)', 'Tom 2 (2940 Hz)');
xlim([0 Fs/2]);
ylim([-80 5]);

saveas(gcf, 'Filtros.png');
