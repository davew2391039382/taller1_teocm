%% leemos el archivo     
data = readtable('data_motor.csv');
% extraemos los datos de las columnas 
tiempo = data{:,2}; % Columna de tiempo
entrada = data{:,3}; % Señal de entrada
respuesta = data{:,4}; % Respuesta del sistema

%% calculamos la linea base que es el numero minimo
base_value = min(respuesta); % Encontrar el valor mínimo de la respuesta


%% Calculamos la linea 100% valor maximo de la respuesta del sistema
n = 20; % Número de valores a promediar
ultimo_promedio = mean(respuesta(end-n+1:end)); % Calcular el promedio

max_value = max(respuesta); % Encontrar el valor máximo de la respuesta

%% Calculamos la linea tangente
% Definir los puntos P1 y P2 en la respuesta
P1_x = 0.328; % Tiempo del primer punto
P2_x = 1.01;   % Tiempo del segundo punto

% Encontrar los valores de la respuesta en estos tiempos
P1_y = interp1(tiempo, respuesta, P1_x); % Interpolación para encontrar el valor de la respuesta en P1_x
P2_y = interp1(tiempo, respuesta, P2_x); % Interpolación para encontrar el valor de la respuesta en P2_x

m = (P2_y - P1_y) / (P2_x - P1_x); % Calcular la pendiente

% Definir el rango de tiempo donde se graficará thela recta tangente
t_tangente = linspace(0.2, 1.45, 100); % Puntos para la recta

% Ecuación de la recta
y_tangente = m * (t_tangente - P1_x) + P1_y;
%%  Calculo de k
% Definir dos puntos en el eje de tiempo (x1, x2)
x1 = 0.35; % Primer punto de tiempo (ejemplo)
x2 = 2.425;  % Segundo punto de tiempo (ejemplo)

% Encontrar los valores de la respuesta en estos tiempos
y1 = interp1(tiempo, respuesta, x1); % Respuesta en x1
y2 = ultimo_promedio % Respuesta en x2

% Variación en la salida (eje Y)
delta_Y = y2 - y1; 

% Variación en el eje de tiempo (eje X)
delta_u =1.5-0;

% Calcular K
K = 0.655;
%% Metodo de Ziegler and Nichols
% Calcular el valor de theta (el punto de intersección con el eje horizontal)
% La ecuación de la recta tangente es: y = m * (t - P1_x) + P1_y
% Queremos que y = 0, por lo que resolvemos m * (theta - P1_x) + P1_y = 0
theta1 = (P1_y) / (-m) + P1_x;
theta1 = 0.10
% Calcular el valor de tau (el desplazamiento desde el punto de intersección con la línea 100%)
% Queremos que y = max_value, por lo que resolvemos m * (tau - P1_x) + P1_y = max_value
tau1 = (max_value - P1_y) / m + P1_x - theta1;

% Crear la función de transferencia con los parámetros calculados
G1 = tf(K, [tau1 1], 'InputDelay', theta1);
% Simular la respuesta del sistema con lsim
y_simulada1 = lsim(G1, entrada, tiempo); % Simulamos la salida del sistema
%% Método de Miller
% Calcular el cambio de la salida

% Encontrar el valor del 63.2% de delta_Y
y_0_632 = 0.632 * delta_Y;

% Obtener tau2 interpolando el tiempo en el que la respuesta alcanza y_0_632

salida = respuesta;
% Encontrar el valor más cercano en la columna de salida
[~, idx] = min(abs(salida - y_0_632));  % 'idx' es el índice del valor más cercano

% Obtener el valor de tiempo correspondiente
tau2 = tiempo(idx);



% Calcular theta2 (intersección de la tangente con el eje X)
theta2 = P1_x - (P1_y / m);
theta2 = 0.10
% Definir la función de transferencia G
K = delta_Y / (max(entrada) - min(entrada)); % Ganancia
G2 = tf(K, [tau2 1], 'InputDelay', theta2); 

% Simular la respuesta del sistema con lsim
y_simulada2 = lsim(G2, entrada, tiempo);
%% Metodo analitico

% Encontrar el valor en el cual el sistema alcanza el 63.2% de delta_Y
y_0_632 =0.632 * delta_Y; 
y_0_284 = 0.284 * delta_Y;

%aproximamos el valor en la cloumna de salida para encontrar y asi mapeando
%en el tiempo y obtener

% Encontrar el valor más cercano en la columna de salida
[~, idxa] = min(abs(salida - y_0_632));  % 'idx' es el índice del valor más cercano

% Encontrar el valor más cercano en la columna de salida
[~, idxb] = min(abs(salida - y_0_284));  % 'idx' es el índice del valor más cercano





x_a = tiempo(idxa);
x_b = tiempo(idxb);

A = [1, 1/3; 1, 1];
b = [x_b; x_a];
x = A\b;

% Guardar la solución en otras variables
theta3 = x(1);  % Guardamos el primer valor de la solución
theta3 = 0.10
tau3 = x(2);  % Guardamos el segundo valor de la solución
G3 = tf(K, [tau3 1], 'InputDelay', theta3); % Definimos la transferencia

% Finalmente, graficamos la y_0salida del sistema usando lsim
y_simulada3 = lsim(G3, entrada, tiempo); % Respuesta del sistema para la entrada dada

%% graficamos 
figure; % Crear una nueva figura
%entrada de señal
plot(tiempo, entrada, 'b', 'LineWidth', 1.5); % Graficar señal de entrada en azul
hold on; % Mantener la gráfica activa para agregar más datos
% respuesta 
plot(tiempo, respuesta, 'r', 'LineWidth', 1.5); % Graficar respuesta del sistema en rojo

% linea base
plot(tiempo, ones(size(tiempo)) * base_value, '--k', 'LineWidth', 1.2); % Línea base en negro punteado
%linea al 100%
plot(tiempo, ones(size(tiempo)) * ultimo_promedio, '--g', 'LineWidth', 1.2); % Línea 100% en verde punteado
%linea tangente
plot(t_tangente, y_tangente, '--m', 'LineWidth', 1.5); % Graficar la recta tangente en magenta


%%
%linea metodo 1
plot(tiempo, y_simulada1, 'b', 'LineWidth', 1.5); % Respuesta simulada en azul

plot(tiempo, y_simulada2, 'g', 'LineWidth', 1.5);

plot(tiempo, y_simulada3, 'm', 'LineWidth', 1.5);
hold off; % Liberar la gráfica


% Configurar la gráfica
grid on; % Activar la cuadrícula
xlabel('Tiempo'); % Etiqueta del eje X
ylabel('Amplitud'); % Etiqueta del eje Y
title('Señal de entrada y respuesta del sistema'); % Título
legend('Entrada', 'Respuesta','Linea Base','Liena al 100%','tangente','Ziegler & Nichols','Miller','M. Analitico'); % Leyenda

