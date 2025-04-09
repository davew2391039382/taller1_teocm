%% TAREA 1 – Definir la función de transferencia y graficar respuesta impulso y escalón

num = [3];        % Numerador
den = [1 2 3];    % Denominador
Gs = tf(num, den);  % Crear función de transferencia continua

% Respuesta al impulso
figure;
subplot 211
impulse(Gs);
title('Tarea 1: Respuesta al Impulso');

% Respuesta al escalón
hold on
subplot 212
step(Gs);
title('Tarea 1: Respuesta al Escalón');

%% TAREA 2 – Almacenar los valores en vectores y graficar

[y_step, t_step] = step(Gs);  % Guardamos la respuesta al escalón
figure;
plot(t_step, y_step, 'b', 'LineWidth', 1);
title('Tarea 2: Respuesta al Escalón almacenada en vectores');
xlabel('Tiempo (s)');
ylabel('Salida');
grid on;

%% TAREA 3 – Agregar tiempo muerto (retardo puro) al sistema

delay = 3;  % Tiempo muerto de 2 segundos
Gs_delay = tf(num, den, 'InputDelay', delay);  % Sistema con retardo
step(Gs_delay)
title('Tarea 3: Respuesta de un sistema de segundo orden con retardo.');
xlabel('Tiempo (s)');
ylabel('Salida');


%% TAREA 4 – Graficar respuestas con retardo (impulso y escalón)

% Respuesta al impulso
figure;
subplot 211
impulse(Gs_delay);
title('Tarea 4: Respuesta al Impulso con retardo');

% Respuesta al escalón

subplot 212
step(Gs_delay);
title('Tarea 4: Respuesta al Escalón con retardo');

%% TAREA 5 – Marcar el valor máximo de la respuesta al escalón con retardo

[y_delay, t_delay] = step(Gs_delay);

[max_val, idx_max] = max(y_delay);    % Valor máximo
t_max = t_delay(idx_max);             % Tiempo en que ocurre

% Graficar y marcar el punto máximo
figure;
plot(t_delay, y_delay, 'b', 'LineWidth', 1); hold on;
plot(t_max, max_val, 'x', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
title('Tarea 5: Máximo de la respuesta al Escalón con retardo');
xlabel('Tiempo (s)');
ylabel('Salida');
legend('Respuesta', 'Máximo');
grid on;

%% TAREA 6 – Verificar el tiempo en que el sistema empieza a responder

umbral = 0.0001;  % Umbral para considerar que la respuesta ha iniciado
idx_inicio = find(y_delay > umbral, 1, 'first');
t_inicio = t_delay(idx_inicio);

% Mostrar por consola
fprintf('Tarea 6: El sistema comienza a responder a partir de t ≈ %.2f segundos\n', t_inicio);

% Graficar el punto de inicio de la respuesta
figure;
plot(t_delay, y_delay, 'b', 'LineWidth', 2); hold on;
plot(t_inicio, y_delay(idx_inicio), 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
title('Tarea 6: Inicio de la respuesta al Escalón');
xlabel('Tiempo (s)');
ylabel('Salida');
legend('Respuesta', 'Inicio de respuesta');
grid on;



%% TAREA 1 – Crear señal arbitraria (Figura 2.2a)

t = linspace(0,30,200);  % Tiempo de 0 a 30 con paso 1s
% lo que hace eso es que va comparando los valores y si es verdadero
% regresa un 1 entonces podemos usar como un escalon comparando todos los
% valores que hay en el vector t
u = (t >= 10)*5 + (t>=20)*5;


% Graficar la señal arbitraria
figure;

plot(t, u, 'LineWidth', 2);
title('Tarea 1: Señal Arbitraria Figura 2.2 (a)');
xlabel('Tiempo (s)');
ylabel('Amplitud');
grid on;


%% TAREA 2 – Respuesta del sistema de segundo orden con retardo a la señal arbitraria 2.2

% Reutilizamos Gs_delay (con InputDelay=2)
y = lsim(Gs_delay, u, t);  % Simular respuesta

% Graficar respuesta
figure;
plot(t, y, 'b', 'LineWidth', 2);
hold on;
plot(t, u, 'LineWidth', 2);
title('Tarea 2: Respuesta del sistema a señal Figura 2.2');
xlabel('Tiempo (s)');
ylabel('Salida');
legend('respuesta con retardo','señal')
grid on;


%% TAREA 3 – Repetir para señal arbitraria Figura 2.3

t2 = linspace(0,40,200);  % Tiempo de 0 a 40
% como esta puesto la condicion entonces solo se multiplicaria el vector
% punto a punto 
u2 = (t2 >= 10 & t2 < 20)* 5 + (t2 >= 20 & t2 < 30).*(t2+-5)+ (t2 >= 30 & t < 40)* 25   ;
% Graficar señal Figura 2.3
figure;
plot(t2, u2, 'LineWidth', 2);
title('Tarea 3: Señal Arbitraria Figura 2.3');
xlabel('Tiempo (s)');
ylabel('Amplitud');
grid on;

%% Respuesta del sistema a esta señal
y2 = lsim(Gs_delay, u2, t2);

figure;
plot(t2, y2, 'b', 'LineWidth', 2);
hold on;
plot(t2, u2, 'LineWidth', 2);
title('Tarea 3: Respuesta del sistema a señal Figura 2.3');
xlabel('Tiempo (s)');
ylabel('Salida');
legend('Respuesta con retardo','Señal')
grid on;

%% parte del reto 

% señal aleatoria 

t3 = linspace(0,60,300);

u3 = (t3 >= 5 & t3 < 20)*5 + (t3 >= 20 & t3 < 30).*(t3-10) + (t3 >= 30 & t3<= 40)*-5 + (t3 >= 40 & t3 < 50)*10+(t3>=50 & t3<60).*(-t3+50);

figure;
% plot(t2, y2, 'b', 'LineWidth', 2);
hold on;
plot(t3, u3, 'LineWidth', 2);
title('Tarea 3: Respuesta del sistema a señal aleatoria');
xlabel('Tiempo (s)');
ylabel('Salida');
grid on;
%% Respuesta del sistema a esta señal RETO
y3 = lsim(Gs_delay, u3, t3);

figure;
plot(t3, y3, 'b', 'LineWidth', 2);
hold on;
plot(t3, u3, 'LineWidth', 2);
title('Tarea 3: Respuesta del sistema a señal Figura 2.3');
xlabel('Tiempo (s)');
ylabel('Salida');
legend('Respuesta con retardo','Señal')