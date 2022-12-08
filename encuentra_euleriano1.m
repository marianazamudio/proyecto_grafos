clc; clear all; 
caminos_e = []
ciclos_e = []

% matriz = [0 1 0 1 0; 
%           1 0 1 1 1;
%           0 1 0 1 0;
%           1 1 1 0 1;
%           0 1 0 1 0];

matriz = [0 1 0 1 1; 
          1 0 1 1 1;
          0 1 0 1 0;
          1 1 1 0 1;
          1 1 0 1 0];

v_visitados = []
vertice_inicial = 1;
v_actual = 1;
v_anterior = 0;  % 0 cuando el vector actual es el primero de todo el camino/ciclo
[caminos_e, ciclos_e] = encuentra_euleriano(matriz, v_visitados, v_actual, v_anterior)

function[caminos_e, ciclos_e] = encuentra_euleriano(matriz, v_visitados, v_actual, v_anterior)
    global caminos_e
    global ciclos_e
    global vertice_inicial
    
    % Añadir vértice actual a los visitados
    v_visitados(end+1) = v_actual
    v_anterior
    disp("hello")
    if (not(v_anterior == 0))
        % Eliminar arista recorrida anteriormente
        v_actual
        v_anterior
        disp("hello1")
        matriz(v_actual,v_anterior)= 0;
        matriz(v_anterior,v_actual)= 0;
    end
    
    % CASO BASE: matriz con solo ceros (no hay más aristas por recorrer)
    % Determinar tamaño de la matriz
    tamano = size(matriz)
    tamano = tamano(1)
    % Generar matriz de 0's
    ceros = zeros(tamano)
    % Determinar si la matriz tiene solo 0's
    matriz
    if (matriz == ceros)
        v_actual
        vertice_inicial
        if isequal(v_actual,vertice_inicial)
            ciclos_e(end+1,:) = v_visitados;
        else 
            caminos_e(end+1,:) = v_visitados; 
        end
    
    % CASO RECURSIVO: aun quedan aristas por recorrer
    else
        % Encuentra todos los vértices accesibles desde el vértice actual
        vertices_accesibles = matriz(v_actual,:)
        vertices_accesibles = find(vertices_accesibles==1)
        
        % Iterar entre vértices disponibles
        for vertice = vertices_accesibles
            % checar que los vértices accesibles tengan otros vértices
            % accesibles
            % tomar renglon de conecciones del vertice
            renglon = matriz(vertice,:)
            r_tamano = size(renglon)
            renglon_cero = zeros(r_tamano(1), r_tamano(2))
            isequal(renglon_cero, renglon)
            if isequal(renglon_cero, renglon) %*****************
                continue
            else
                % Llamar a la funcióon recursiva para encontrar los vértices a
                % los que se puede conectar el vértice actual. De manera que se
                % vaya completando una parte más del camino euleriano
                encuentra_euleriano(matriz,v_visitados,vertice, v_actual)
            end
        end  
    end
end