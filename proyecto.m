clc;
clear all;

% Variables para guardar resultados 
caminos_h = [];
ciclos_h = [];
caminos_e = [];
ciclos_e = [];
% Matriz de entrada que representa un grafo
matriz = [0 1 0 ;
          0 0 1 ;
          1 0 0];
      
% PENDIENTE: 
% convertir a 1 y 0s grafos con aristas ponderados. 
      
% Determinar si el grafo es dirigido o no dirigido
% Un grafo dirigido se representa por medio de una matriz simétrica
% Si una matriz es simétrica su transpuesta es igual a la matriz original
% Grafo dirigido    --> dirigido = 1
% Grafo no dirigido --> dirigido = 0
dirigido = not(isequal(matriz, matriz'))

% Si el grafo es dirigido ver si existen ciclos y caminos hamiltoneanos
if (dirigido == 1)
    % tamaño de matriz
    [renglon, columna] = size(matriz)
    v_no_visitados = (1:columna)
    v_visitados = []
    [caminos_h, ciclos_h] = encuentra_hamiltoniano(matriz, 1, v_no_visitados, v_visitados)
            
% Si el grafo es no dirigido ver si existen ciclos y caminos eulerianos
else
    
    
end 
  
function [caminos_h, ciclos_h] = encuentra_hamiltoniano(matriz, v_actual, v_no_visitados, v_visitados)
    global caminos_h
    global ciclos_h
    display("------")
    % Quitar vertice actual
    posicion_v_act = find(v_no_visitados==v_actual)
    v_no_visitados(posicion_v_act) = []
    % Añadir vértice actual 
    v_visitados(end+1) = v_actual
    % Caso base 1: queda 1 vértice no visitado, tenemos un camino hamiltoniano 
    if length(v_no_visitados)==1
        caminos_h(end+1,:)= v_visitados;
    end
    % Caso base 2: no quedan vértices sin visitar, tenemos un ciclo
    % hamiltoniano
    if isempty(v_no_visitados)==1
        ciclos_h(end+1,:)= v_visitados;
        return
    end
        % Caso recursivo: 
    if length(v_no_visitados)>1
            % Encuentra los vértices  accesibles
            vertices_accesibles = matriz(v_actual,:)
           
            for pos = (1:length(vertices_accesibles))
                if (matriz(v_actual,pos) == 1)
                    % Checar que no exista el vértice igual a pos, en
                    % vértices visitados
                    if (isempty(find(v_visitados == pos)))
                        % Llamar la función recursiva
                        encuentra_hamiltoniano(matriz, pos, v_no_visitados, v_visitados)
                        
                    else 
                        continue
                    end
                end
            end
    end
end
            
        
        
