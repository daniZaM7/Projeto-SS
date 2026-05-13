% -------------------------------------------------------------------------
% Projeto: Sistema de Controlo de Acesso via Tons
% Autores: Daniel Zamurca (118799) e Nicolas Sousa (119744) - Turma P3
% -------------------------------------------------------------------------

clear all; close all; clc;
pkg load signal; % (Necessário se usares Octave. No MATLAB podes apagar/comentar esta linha)

%% 1. Parâmetros do Sistema (Baseado nas nossas contas)
Fs = 8000;              % Frequência de amostragem (Hz)
Ts = 1 / Fs;            % Período de amostragem
N_taps = 128;           % Tamanho do buffer no ESP32 (e tamanho do filtro)
order = N_taps - 1;     % Ordem do filtro FIR (127)
BW = 100;               % Largura de banda de +/- 100 Hz para cada tom

% As nossas Frequências Secretas
f0 = 1500;
f1 = 2220;
f2 = 2940;

%% 2. Desenho dos Filtros FIR (Usando fir1 com Janela de Hamming embutida)
% Tal como o professor fez no h_alpha e h_beta, mas para os nossos tons
f_banda0 = [f0-BW, f0+BW];
f_banda1 = [f1-BW, f1+BW];
f_banda2 = [f2-BW, f2+BW];

h0 = fir1(order, f_banda0/(Fs/2), 'bandpass');
h1 = fir1(order, f_banda1/(Fs/2), 'bandpass');
h2 = fir1(order, f_banda2/(Fs/2), 'bandpass');

%% 3. Imprimir os Coeficientes para o ESP32 (Formato C)
fprintf('\n// --- COPIAR ESTES ARRAYS PARA O TOPO DO MAIN.C ---\n');
fprintf('#define N_TAPS %d\n\n', N_taps);

% Imprimir Tom 0
fprintf('__attribute__((aligned(16))) const float filtro_tom0[%d] = {', N_taps);
fprintf('%f, ', h0(1:end-1)); fprintf('%f};\n\n', h0(end));

% Imprimir Tom 1
fprintf('__attribute__((aligned(16))) const float filtro_tom1[%d] = {', N_taps);
fprintf('%f, ', h1(1:end-1)); fprintf('%f};\n\n', h1(end));

% Imprimir Tom 2
fprintf('__attribute__((aligned(16))) const float filtro_tom2[%d] = {', N_taps);
fprintf('%f, ', h2(1:end-1)); fprintf('%f};\n', h2(end));
fprintf('// -----------------------------------------------------\n\n');

%% 4. SIMULAÇÃO: Como o ESP32 vai processar o som?
% Vamos criar um "sinal falso" de 1 segundo que tem o Tom '1' (2220 Hz) e muito ruído
t = 0:Ts:1-Ts;
sinal_mic = 1.0 * sin(2*pi*f1*t) + 0.5 * randn(size(t)); % Tom 1 + Ruído

% Passamos o sinal pelos 3 filtros
saida0 = filter(h0, 1, sinal_mic);
saida1 = filter(h1, 1, sinal_mic);
saida2 = filter(h2, 1, sinal_mic);

% Lógica de Janelas (Thresholding) tal como no _30_EEGSignalRestDetection.m
% Vamos analisar apenas o primeiro bloco de amostras (tal como o FIFO do ESP32 faz)
janela0 = saida0(1:N_taps);
janela1 = saida1(1:N_taps);
janela2 = saida2(1:N_taps);

% Cálculo de Energia (Usando a soma dos quadrados, que é proporcional à potência RMS)
energia0 = sum(janela0.^2);
energia1 = sum(janela1.^2);
energia2 = sum(janela2.^2);

fprintf('--- RESULTADO DA SIMULAÇÃO NO PRIMEIRO BLOCO (128 amostras) ---\n');
fprintf('Som injetado no microfone: Tom 1 (2220 Hz) com ruído.\n');
fprintf('Energia calculada após Filtro 0 (1500 Hz): %.2f\n', energia0);
fprintf('Energia calculada após Filtro 1 (2220 Hz): %.2f\n', energia1);
fprintf('Energia calculada após Filtro 2 (2940 Hz): %.2f\n', energia2);

if (energia1 > energia0 && energia1 > energia2)
    fprintf('\nSUCESSO: O sistema filtrou corretamente e detetou o Tom 1!\n');
else
    fprintf('\nERRO: Deteção falhou.\n');
end

%% 5. Visualização de Resultados (Para o vosso Relatório)
figure('Position', [100, 100, 1000, 800]);

% Gráfico 1: Resposta em Frequência (Igual ao _11_LowPassStudyStud.m)
[H0, f_plot] = freqz(h0, 1, 2048, Fs);
[H1, ~] = freqz(h1, 1, 2048, Fs);
[H2, ~] = freqz(h2, 1, 2048, Fs);

subplot(2,1,1);
plot(f_plot, 20*log10(abs(H0)), 'b', 'LineWidth', 1.5); hold on;
plot(f_plot, 20*log10(abs(H1)), 'r', 'LineWidth', 1.5);
plot(f_plot, 20*log10(abs(H2)), 'g', 'LineWidth', 1.5);
grid on;
xlabel('Frequência (Hz)');
ylabel('Magnitude (dB)');
title('Resposta em Frequência dos Filtros Passa-Banda (FIR de ordem 127)');
legend('Tom 0 (1500 Hz)', 'Tom 1 (2220 Hz)', 'Tom 2 (2940 Hz)');
xlim([0 Fs/2]);
ylim([-80 5]);

% Gráfico 2: O Sinal original vs Sinal Filtrado (No domínio do tempo)
subplot(2,1,2);
plot(t(1:200)*1000, sinal_mic(1:200), 'k', 'LineWidth', 0.5); hold on;
plot(t(1:200)*1000, saida1(1:200), 'r', 'LineWidth', 1.5);
grid on;
xlabel('Tempo (ms)');
ylabel('Amplitude');
title('Sinal Bruto com Ruído vs Sinal Filtrado (Tom 1)');
legend('Sinal ADC (Mic)', 'Saída do Filtro 1');
xlim([0 25]); % Mostramos apenas os primeiros 25 milissegundos

saveas(gcf, 'graficos_relatorio.png');
