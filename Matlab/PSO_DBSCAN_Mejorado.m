clear; clc; close all;

%% === PARÁMETROS GENERALES ===
n_puntos = 150;                            % Número total de puntos aleatorios simulados (representan datos o eventos)
n_agentes = 10;                            % Número de agentes robóticos simulados
n_iteraciones = 100;                       % Número de iteraciones 
obstaculos = [3 3 0.7; 6 7 0.9; 8 2 0.6; 5 3 0.7]; % Obstáculos: cada fila = [posición x, posición y, radio]
posiciones = rand(n_puntos, 2) * 10;       % Posiciones iniciales aleatorias de los puntos en el área 10x10
pos_agentes = rand(n_agentes, 2) * 10;     % Posiciones iniciales aleatorias de los agentes
trayectoria_agente = cell(n_agentes, 1);   % Celdas para guardar el historial de trayectorias de cada agente

% === PARÁMETROS PSO ===
inercia = 0.7; coef1 = 1.5; coef2 = 1.5;   % Parámetros del algoritmo PSO: inercia, atracción individual y global
dim = 2;                                   % Dimensión del espacio (2D)

for t = 1:n_iteraciones
    % === AGRUPAMIENTO DBSCAN Y CENTROIDES ===
    if t == 1
        etiquetas = dbscan(posiciones, 0.6, 5);           % Aplica DBSCAN a los puntos para agruparlos
        grupos = unique(etiquetas); grupos(grupos == -1) = []; % Identifica los grupos válidos y elimina el ruido 
        centroides = arrayfun(@(i) mean(posiciones(etiquetas==grupos(i),:),1), 1:length(grupos), 'UniformOutput', false); 
        centroides = vertcat(centroides{:});              % Matriz de centroides

        % Repetir centroides si hay menos que agentes (cada agente necesita un objetivo)
        repeticiones = ceil(n_agentes / size(centroides,1)); 
        pos_objetivo = repmat(centroides, repeticiones, 1); 
        pos_objetivo = pos_objetivo(1:n_agentes,:);       
    end

    % === VISUALIZACIÓN ===
    clf; hold on; axis equal;              
    gscatter(posiciones(:,1), posiciones(:,2), etiquetas, 'kbgmr', '.', 10); % Grafica los puntos agrupados por color
    for o = 1:size(obstaculos,1)
        viscircles(obstaculos(o,1:2), obstaculos(o,3), 'Color', 'k', 'LineStyle', '--'); % Dibuja los obstáculos
    end

    % === MOVIMIENTO CON PSO PARA CADA AGENTE ===
    for i = 1:n_agentes
        % Inicializar partículas cercanas al agente i
        n_particulas = 20;                                   % Número de partículas por agente
        particulas = pos_agentes(i,:) + rand(n_particulas,2)*0.5 - 0.25; % Posiciones iniciales de partículas
        velocidades = zeros(n_particulas,2);                 % Velocidades iniciales
        mejor_personal = particulas;                         % Mejor posición individual de cada partícula
        mejor_global = particulas(1,:);                      % Mejor posición global inicial

        % Función de costo: distancia al objetivo + penalización por obstáculos
        funcion_costo = @(pos) norm(pos - pos_objetivo(i,:)) + ...
            sum(arrayfun(@(j) (norm(pos - obstaculos(j,1:2)) < obstaculos(j,3)+0.2)*100, 1:size(obstaculos,1)));

        % Iteraciones internas del PSO
        for iter = 1:15
            f = zeros(n_particulas,1);                       % Vector para valores de la función costo
            for j = 1:n_particulas
                f(j) = funcion_costo(particulas(j,:));       % Evalúa cada partícula
                if funcion_costo(particulas(j,:)) < funcion_costo(mejor_personal(j,:)) % Mejora del mejor personal
                    mejor_personal(j,:) = particulas(j,:);   % Actualiza el mejor personal
                end
            end
            [~, mejor_idx] = min(f);                         % Índice de la mejor partícula del grupo
            mejor_global = mejor_personal(mejor_idx,:);      % Actualiza mejor global

            % Actualización de velocidad y posición
            for j = 1:n_particulas
                velocidades(j,:) = inercia*velocidades(j,:) + ...             % Componente de inercia
                    coef1*rand*(mejor_personal(j,:) - particulas(j,:)) + ...  % Atracción hacia el mejor personal
                    coef2*rand*(mejor_global - particulas(j,:));              % Atracción hacia el mejor global
                particulas(j,:) = particulas(j,:) + velocidades(j,:);         % Actualiza posición
            end
        end

        % === Movimiento del agente hacia el mejor global con evasión ===
        paso = 0.2;                                      % Tamaño del paso del agente
        direccion = mejor_global - pos_agentes(i,:);     % Dirección hacia el objetivo
        distancia = norm(direccion);                     % Distancia al objetivo
        if distancia > 1e-3                              % Evitar división por cero
            direccion = direccion / distancia;           % Normaliza dirección
        else
            direccion = [0 0];
        end
        siguiente_pos = pos_agentes(i,:) + paso * direccion; % Calcula siguiente posición

        % === Verificar colisiones con obstáculos y ajustar trayectoria ===
        for o = 1:size(obstaculos,1)
            d_obs = norm(siguiente_pos - obstaculos(o,1:2));   % Distancia al centro del obstáculo
            if d_obs < obstaculos(o,3) + 0.3                   % Si se acerca demasiado
                repulsion = siguiente_pos - obstaculos(o,1:2); % Vector de repulsión
                repulsion = repulsion / norm(repulsion + 1e-6);% Normalización segura
                siguiente_pos = pos_agentes(i,:) + paso * (0.5*direccion + 1.0*repulsion); % Ajuste de trayectoria
                break;                                         % Sale del bucle tras ajustar
            end
        end

        % Actualizar posición y trayectoria del agente
        pos_agentes(i,:) = siguiente_pos;                       % Actualiza posición del agente
        trayectoria_agente{i}(end+1,:) = siguiente_pos;         % Guarda el punto en su trayectoria

        % Dibujar agente y su trayectoria
        plot(pos_agentes(i,1), pos_agentes(i,2), 'ro', 'MarkerFaceColor','r'); % Dibuja el agente
        plot(trayectoria_agente{i}(:,1), trayectoria_agente{i}(:,2), 'r:');    % Dibuja su trayectoria
    end

    % === Título y límites de la simulación ===
    title(['Iteración ', num2str(t)]);                 % Muestra el número de iteración
    xlim([0 10]); ylim([0 10]);                        % Límites del marco de trabajo
    pause(0.03);                                       
end
