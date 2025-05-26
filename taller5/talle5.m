num = 1;
den = [2 -1];
G = tf(num, den);
[y, t] = step(G);
step(G)
title('Respuesta al escalón del sistema G(s) = -1/(-2s+1)')
xlabel('Tiempo (s)')
ylabel('Salida y(t)')
grid on


[n, d] = tfdata(G, 'v');
polos = roots(d)
%%
s = tf('s');
G = 1 / (2*s + 1);
[y, t] = step(G);

% Valor de tau
tau = 2;
t4tau = 4 * tau; % 8 segundos

% Buscar el índice del tiempo más cercano a 8
[~, idx] = min(abs(t - t4tau));

% Valor de y en t = 4tau
y_4tau = y(idx);

% Mostrarlo en la gráfica
figure
plot(t, y, 'b')
hold on
plot(t(idx), y_4tau, 'ro', 'MarkerSize', 8, 'LineWidth', 1)
xline(t4tau, '--r', 't = 4\tau', 'LabelOrientation','horizontal')
title('Respuesta al escalón del sistema G(s) = 1/(2s+1)')
xlabel('Tiempo (s)')
ylabel('Salida y(t)')
legend('y(t)', 'y(4\tau)', 'Location', 'southeast')
grid on

% Mostrar en consola
fprintf('El valor de y(t = %.2f) es %.4f\n', t4tau, y_4tau);
%% Segundo orden
% Definición de funciones de transferencia
s = tf('s');

G1 = 3 / s;
G2 = 1 / (s^2 + 1);
H1 = 3;

% Sistema en serie
G = series(G1, G2); % o simplemente G = G1 * G2;

% Sistema en lazo cerrado
Gs = feedback(G, H1);

% Mostrar el resultado
disp('Función de transferencia en lazo cerrado G(s):');
Gs

% Obtener el denominador para el criterio de Routh-Hurwitz
[num, den] = tfdata(Gs, 'v');

% Mostrar los coeficientes del denominador
disp('Coeficientes del denominador:');
disp(den);

% Encontrar polos del sistema
p = pole(Gs);% Suponiendo que ya tienes definida la función de transferencia Gs

figure; % Crea una nueva figura

% Parte de arriba: respuesta al escalón
subplot(2,1,1); % 2 filas, 1 columna, primera subfigura
step(Gs);
title('Respuesta al Escalón');
ylim([-7*10^11 7*10^11])
xlim([0 35])
% Parte de abajo: mapa de polos y ceros
subplot(2,1,2); % 2 filas, 1 columna, segunda subfigura
pzmap(Gs);
title('Mapa de Polos y Ceros');


%%
% Función auxiliar para construir tabla de Routh (puedes copiarla a tu script)
function routh_table = routh_hurwitz(coeffs)
    n = length(coeffs);
    m = ceil(n/2);
    routh_table = zeros(n, m);
    
    routh_table(1, :) = coeffs(1:2:end);
    routh_table(2, 1:length(coeffs(2:2:end))) = coeffs(2:2:end);
    
    for i = 3:n
        for j = 1:m-1
            a = routh_table(i-2, 1);
            b = routh_table(i-2, j+1);
            c = routh_table(i-1, 1);
            d = routh_table(i-1, j+1);
            routh_table(i, j) = ((c*b - a*d) / c);
        end
    end
end

% Llamar a la función con los coeficientes
R = routh_hurwitz(den)
%% Reto
clc;
clear;

%Parámetros del sistema RLC y PID
R = 5;
L = 0.1;
Cap = 220e-6;

Kp = 26.3168;
Ki = 2903.52;
Kd = 0.047182;
N = 252599.681;

% Función de transferencia del sistema RLC
% G(s) = 1 / (L*C*s^2 + R*C*s + 1)
num_plant = [1];
den_plant = [L*Cap, R*Cap, 1];
G = tf(num_plant, den_plant);

% PID con filtro derivativo (implementación estándar)
% PID(s) = Kp + Ki/s + (Kd*N)/(1 + N/s)
s = tf('s');
PID = Kp + Ki/s + (Kd*N*s)/(s + N);

% Sistema en lazo cerrado
open_loop = series(PID, G);
closed_loop = feedback(open_loop, 1);

% Ecuación característica del lazo cerrado
[num_cl, den_cl] = tfdata(closed_loop, 'v');
disp('Ecuación característica del sistema (denominador):');
disp(den_cl);

% Criterio de Routh-Hurwitz
% Usamos la función routh para generar la tabla (si no la tienes, puedo darte el código)
disp('Tabla de Routh-Hurwitz:');
routh_table = routh(den_cl);

% Análisis de estabilidad
disp('Raíces del denominador (polos del sistema):');
p = roots(den_cl);
disp(p);

figure;
subplot(2,1,1);
step(closed_loop);
title('Respuesta al escalón del sistema con PID');
grid on;

subplot(2,1,2);
pzmap(closed_loop);
title('Mapa de polos y ceros');
grid on;
function routh_table = routh(coeff)
    % Número de filas necesarias
    n = length(coeff);
    m = ceil(n/2);

    % Inicializa la tabla con ceros
    routh_table = zeros(n, m);

    % Primera fila: coeficientes pares
    routh_table(1, :) = coeff(1, 1:2:end);
    
    % Segunda fila: coeficientes impares
    if length(coeff) > 1
        routh_table(2, 1:length(coeff(2:2:end))) = coeff(2:2:end);
    end

    % Llenado de la tabla
    for i = 3:n
        for j = 1:m-1
            a = routh_table(i-2,1);
            b = routh_table(i-2,j+1);
            c = routh_table(i-1,1);
            d = routh_table(i-1,j+1);
            
            % Prevención de división por cero
            if c == 0
                c = 1e-6;
            end
            
            routh_table(i,j) = ((c*b - a*d) / c);
        end

        % Si la fila queda completamente en ceros, usar método auxiliar
        if all(routh_table(i,:) == 0)
            order = n - i + 1;
            aux_poly = routh_table(i-1,:);
            aux_deriv = polyder(aux_poly);
            routh_table(i,1:length(aux_deriv)) = aux_deriv;
        end
    end
end
