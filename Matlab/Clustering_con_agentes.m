clear; clc; close all; % Limpiar el workspace

%% PARÁMETROS DE LA SIMULACIÓN

n_puntos = 150;        % Número total de puntos de datos aleatorios a generar
n_iteraciones = 100;   % Número de iteraciones de la simulación
velocidad_max = 0.05;  % Velocidad máxima de los puntos
n_agentes = 10;        % Número total de agentes robóticos disponibles

% Generación de datos aleatorios
posiciones = rand(n_puntos, 2) * 10;                    % Posiciones iniciales aleatorias dentro del área 10x10
velocidades = (rand(n_puntos, 2) - 0.5) * velocidad_max; % Velocidades aleatorias 

% Movimiento de datos
for t = 1:n_iteraciones
    posiciones = posiciones + velocidades;              % Actualiza las posiciones conforme a las velocidades
    clf;                                                % Limpia la figura antes de dibujar
    scatter(posiciones(:,1), posiciones(:,2), 20, 'k', 'filled'); % Muestra los puntos
    xlim([0 10]); ylim([0 10]);                         % Área visible
    title(['Simulación - Movimiento de Datos | Iteración ', num2str(t)]); 
    pause(0.01);                                        
end

%% Aplicación de agrupamiento DBSCAN

radio = 0.6;  % Radio de vecindad para considerar puntos cercanos
min_vecinos = 5; % Mínimo número de vecinos para formar un grupo
etiquetas = dbscan(posiciones, radio, min_vecinos); % Ejecuta el algoritmo DBSCAN

% Eliminación de ruido
grupos = unique(etiquetas);                     % Identifica las etiquetas únicas
grupos(grupos == -1) = [];                      % Elimina los puntos considerados ruido
n_grupos = length(grupos);                      % Conteo del número de grupos válidos

% Calcular tamaños de los grupos
conteos = arrayfun(@(g) sum(etiquetas == g), grupos); % Número de puntos por grupo

% Ordenar por tamaño descendente
[conteos_ordenados, idx_ordenados] = sort(conteos, 'descend'); % Ordenamiento
grupos_ordenados = grupos(idx_ordenados);                       % Reordena las etiquetas

% Reetiquetar grupos: 1 = más grande
nuevas_etiquetas = zeros(size(etiquetas));       % Vector para las nuevas etiquetas
for i = 1:n_grupos
    nuevas_etiquetas(etiquetas == grupos_ordenados(i)) = i; % Asigna etiquetas nuevas
end

% Visualización de grupos
figure;
gscatter(posiciones(:,1), posiciones(:,2), nuevas_etiquetas); % Grafica los grupos
title('Datos agrupados por DBSCAN (reordenados por tamaño)');
xlabel('X'); ylabel('Y');

%% Asignación de agentes a grupos

% Distribución de agentes
asignacion_agentes = [4 3 2 1];  

% Ajuste de distribución según número de grupos
asignacion_agentes = asignacion_agentes(1:min(n_grupos, length(asignacion_agentes))); 

% Posiciones iniciales de los agentes
pos_agentes = rand(n_agentes, 2) * 10;            

% Calcular centroides de los grupos
centroides = zeros(n_grupos, 2);                  
for i = 1:n_grupos
    centroides(i,:) = mean(posiciones(nuevas_etiquetas == i, :)); % Centroide de cada grupo
end

% Asignar agentes a grupos según distribución
agente_a_grupo = [];                              
for i = 1:length(asignacion_agentes)
    agente_a_grupo = [agente_a_grupo; repmat(i, asignacion_agentes(i), 1)]; % Repite el índice del grupo
end

% Visualización final
figure;
hold on;
gscatter(posiciones(:,1), posiciones(:,2), nuevas_etiquetas); % Visualiza los grupos
for i = 1:length(agente_a_grupo)
    id_grupo = agente_a_grupo(i);                % Grupo asignado al agente i
    posicion = centroides(id_grupo,:) + 0.5 * randn(1,2); % Posición cercana al centroide
    pos_agentes(i,:) = posicion;                 % Guarda la posición
    plot(posicion(1), posicion(2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8); % Muestra agente
end
title('Agentes asignados a grupos según tamaño');
xlabel('X'); ylabel('Y');
legend('Grupo 1', 'Grupo 2', 'Grupo 3', 'Grupo 4', 'Agentes'); 
