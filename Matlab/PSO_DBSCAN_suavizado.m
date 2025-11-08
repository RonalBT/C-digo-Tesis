clear; clc; close all; 

%% === PARÁMETROS GENERALES ===
n_puntos = 150;         % Número total de puntos aleatorios simulados (representan datos o eventos)
n_agentes = 20;         % Número de agentes robóticos simulados
n_iteraciones = 200;    % Número de iteraciones 
obstaculos = [3 3 0.7; 6 7 0.9; 8 2 0.6];  % Obstáculos: cada fila = [posición x, posición y, radio]
posiciones = rand(n_puntos, 2) * 10;       % Posiciones iniciales aleatorias de los puntos en el área 10x10
pos_agentes = rand(n_agentes, 2) * 10;     % Posiciones iniciales aleatorias de los agentes
trayectorias_agente = cell(n_agentes, 1);  % Celdas para guardar el historial de trayectorias de cada agente

% === PARÁMETROS DEL ALGORITMO PSO ===
inercia = 0.7; coef1 = 1.5; coef2 = 1.5;   % Parámetros del algoritmo PSO: inercia, atracción individual y global
dim = 2;                                   % Dimensión del espacio (2D)

for t = 1:n_iteraciones
    % === AGRUPAMIENTO DBSCAN Y CENTROIDES ===
    if t == 1
        etiquetas = dbscan(posiciones, 0.6, 5);      % Aplica DBSCAN a los puntos para agruparlos
        grupos = unique(etiquetas); grupos(grupos == -1) = []; % Identifica los grupos válidos y elimina el ruido 
        centroides = arrayfun(@(i) mean(posiciones(etiquetas==grupos(i),:),1), 1:length(grupos), 'UniformOutput', false);
        centroides = vertcat(centroides{:});         % % Matriz de centroides

        % Repetir centroides si hay menos que agentes (cada agente necesita un objetivo)
        repeticiones = ceil(n_agentes / size(centroides,1));
        pos_objetivo = repmat(centroides, repeticiones, 1);
        pos_objetivo = pos_objetivo(1:n_agentes,:);  
    end

    % === VISUALIZACIÓN ===
    clf; hold on; axis equal;                      
    etiquetas_validas = unique(etiquetas);           
    etiquetas_validas(etiquetas_validas == -1) = []; 
    colores = lines(length(etiquetas_validas));      % Generación de colores para los grupos
    etiquetas_temp = etiquetas;
    etiquetas_temp(etiquetas == -1) = 0;             % gris para el ruido
    gscatter(posiciones(:,1), posiciones(:,2), etiquetas_temp, [0.6 0.6 0.6; colores], '.', 10); % Grafica de puntos y ruido en gris

    for o = 1:size(obstaculos,1)
        viscircles(obstaculos(o,1:2), obstaculos(o,3), 'Color', 'k', 'LineStyle', '--'); % Dibuja los obstáculos
    end

    % === MOVIMIENTO CON PSO PARA CADA AGENTE ===
    for i = 1:n_agentes
        n_particulas = 20;                                       % Número de partículas por agente
        particulas = pos_agentes(i,:) + rand(n_particulas,2)*0.5 - 0.25; % Posiciones iniciales de partículas
        velocidades = zeros(n_particulas,2);                     % Velocidades iniciales
        mejor_personal = particulas;                             % Mejor posición individual de cada partícula
        mejor_global = particulas(1,:);                          % Mejor posición global inicial

        % Función de costo: distancia al objetivo + penalización por obstáculos cercanos
        funcion_costo = @(pos) norm(pos - pos_objetivo(i,:)) + ...
            sum(arrayfun(@(j) (norm(pos - obstaculos(j,1:2)) < obstaculos(j,3)+0.2)*100, 1:size(obstaculos,1)));

        % Iteraciones internas del PSO
        for iter = 1:15
            f = zeros(n_particulas,1);                           % Vector para valores de la función costo
            for j = 1:n_particulas
                f(j) = funcion_costo(particulas(j,:));           % Evalúa cada partícula
                if f(j) < funcion_costo(mejor_personal(j,:))     % Mejora del mejor personal
                    mejor_personal(j,:) = particulas(j,:);       % Actualiza el mejor personal
                end
            end
            [~, mejor_idx] = min(f);                             % Encuentra la mejor partícula del grupo
            mejor_global = mejor_personal(mejor_idx,:);          % Actualiza el mejor global

            % Actualización de velocidad y posición de partículas
            for j = 1:n_particulas
                velocidades(j,:) = inercia*velocidades(j,:) + ...           % Componente de inercia
                    coef1*rand*(mejor_personal(j,:) - particulas(j,:)) + ...% Atracción hacia mejor personal
                    coef2*rand*(mejor_global - particulas(j,:));            % Atracción hacia mejor global
                particulas(j,:) = particulas(j,:) + velocidades(j,:);       % Actualiza posición
            end
        end

        %MOVIMIENTO DEL AGENTE HACIA EL MEJOR GLOBAL 
        paso = 0.2;                                             % Tamaño del paso del agente
        direccion = mejor_global - pos_agentes(i,:);            % Vector dirección hacia el objetivo
        distancia = norm(direccion);                            % Distancia al objetivo
        if distancia > 1e-3
            direccion = direccion / distancia;                  % Normaliza dirección
        else
            direccion = [0 0];                                  % Si ya está cerca ya no se mueve el agente
        end
        siguiente_pos = pos_agentes(i,:) + paso * direccion;    % Calcula la siguiente posición

        %EVASIÓN DE OBSTÁCULOS 
        for o = 1:size(obstaculos,1)
            d_obs = norm(siguiente_pos - obstaculos(o,1:2));    % Distancia al centro del obstáculo
            if d_obs < obstaculos(o,3) + 0.3                    % Si se acerca demasiado
                repulsion = siguiente_pos - obstaculos(o,1:2);  % Vector de repulsión
                repulsion = repulsion / norm(repulsion + 1e-6); % Normaliza para evitar división por cero
                siguiente_pos = pos_agentes(i,:) + paso * (0.5 * direccion + 1.0 * repulsion); % Desvía trayectoria
                break;                                          % Solo evita un obstáculo por vez
            end
        end

        %%SUAVIZADO EXPONENCIAL DE LA TRAYECTORIA 
        alpha = 0.3;                                            % Factor de suavizado
        if isempty(trayectorias_agente{i})                      % Si es el primer punto
            posicion_suave = siguiente_pos;
        else
            posicion_suave = alpha * siguiente_pos + (1 - alpha) * trayectorias_agente{i}(end,:); % Filtro exponencial
        end

        %%ACTUALIZAR POSICIÓN Y TRAZAR MOVIMIENTO 
        pos_agentes(i,:) = posicion_suave;                      % Actualiza posición del agente
        trayectorias_agente{i}(end+1,:) = posicion_suave;       % Guarda trayectoria

        % Dibuja agente y trayectoria
        plot(pos_agentes(i,1), pos_agentes(i,2), 'ro', 'MarkerFaceColor','r'); 
        plot(trayectorias_agente{i}(:,1), trayectorias_agente{i}(:,2), 'r:'); 
    end

    % === Título y límites de la simulación ===
    title(['Iteración ', num2str(t)]);                          % Muestra el número de iteración
    xlim([0 10]); ylim([0 10]);                                   % Límites del marco de trabajo
    pause(0.03);                                                
end

