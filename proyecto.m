% limpiar ventana de comandos y workspace 
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

%         a b c d e f g h i 
matriz = [0 1 0 1 0 0 0 0 0;  %a
          0 0 1 0 0 0 0 0 0;  %b
          0 0 0 0 0 1 0 0 1;  %c
          0 0 0 0 1 0 1 0 0;  %d
          0 1 0 0 0 1 0 0 0;  %e
          0 0 1 0 0 0 0 0 1;  %f
          1 0 0 0 0 0 0 1 0;  %g
          0 0 0 0 0 0 1 0 0;  %h
          0 0 0 0 0 0 0 1 0;] %i
      
% TODO: 
% Detectar si la matriz es cuadrada
% Detectar si la matriz tiene solo 0's y 1's, si tiene otros valores,
% entonces:
% convertir a 1 y 0s grafos con aristas ponderados. 
matriz_secundaria = [];

% Determinar si el grafo es dirigido o no dirigido
% Un grafo dirigido se representa por medio de una matriz simétrica
% Si una matriz es simétrica su transpuesta es igual a la matriz original
% Grafo dirigido    --> dirigido = 1
% Grafo no dirigido --> dirigido = 0
dirigido = not(isequal(matriz, matriz'));

% ----[ DIRIGIDO ]----%
% Si el grafo es dirigido ver si existen ciclos y caminos hamiltoneanos
if (dirigido == 1)
    % Obtener tamaño de matriz
    [renglon, columna] = size(matriz);
    % Iterar entre vértices, para empezar a buscar caminos/ciclos
    % hamiltoneanos que comiencen desde el vértice actual
    for v_actual= (1:columna)
        % Inicializar los vértices no visitados como un vector que
        % contiene todos los vértices
        v_no_visitados = (1:columna);
        
        % Inicializar un vector vacío donde se irán almacenando los
        % vértices visitados, al ir recorriendo los caminos/ciclos
        v_visitados = [];
        
        % Llamar a la función que busca los caminos y ciclos hamiltoneanos,
        % del grafo representado por la matriz y que comienzan desde el
        % vector actual
        [caminos_h, ciclos_h] = encuentra_hamiltoniano(matriz, v_actual, v_no_visitados, v_visitados, 1)
    end
    
    % -----[ NO DIRIGIDO ]-----%
    %Si el grafo es no dirigido ver si existen ciclos y caminos eulerianos
    else
    
    
end 

%-------------------------------------------------------------------%
% encuentra_hamiltoniano
% función que enncuentra los caminos y/o ciclos hamiltonianos del 
% grafo representado por una matriz
%
% Entradas:
%   matriz: matriz cuadrada que representa un grafo
%   v_actual: vértice actual, a partir del cual se busca un camino/ciclo h
%   v_no_visitados: vector columna con vértices no visitados aun en el
%                   camino/ciclo
%   v_visitados: vector columna con los vértices que ya forman parte del
%                camino/ciclo formado hasta el momento
%   primero: 
%           1: indica que el vector actual es el inicio del ciclo/camino
%           0: indica que el vector actual no es el inicio del ciclo/camino
% 
% Salidas: 
%   caminos_h: matriz, donde cada renglon es un camino hamiltoneano
%   ciclos_h: matriz, donde cada renglon es un ciclo hamiltoneano
%-------------------------------------------------------------------%
function [caminos_h, ciclos_h] = encuentra_hamiltoniano(matriz, v_actual, v_no_visitados, v_visitados, primero)
    % Importar variables globales para almacenar los resultados
    global caminos_h;
    global ciclos_h;
    
    % Verificar que el vértice actual visitado no sea el primero
    if primero ~= 1
        % Quitar vértice actual del vector que contienen los vértices no visitados
        % NOTA 1: Solo se elimina cuando v_actual cuando éste no es el primero del camino/ciclo,
        % así, al término del recorrido por todos los vértices, se puede determinar
        % si es posible volver al vértice inicial y formar un ciclo 
        posicion_v_act = find(v_no_visitados==v_actual);
        v_no_visitados(posicion_v_act) = [];
        
    end
    
    % Añadir vértice actual a los vértices visitados
    v_visitados(end+1) = v_actual;
    
    % Caso base 1: queda 1 vértice no visitado(el primero que se visitó), y
    % el vector con los vértices visitados contiene a todos los vértices
    % del grafo, por lo tanto tenemos un camino hamiltoniano. Se puede
    % verificar a partir de esto, si existe un ciclo hamiltoniano
    
    % Obtener tamaño de la matriz
    tamano_matriz = size(matriz);
    % verificar que solo hay un véctor sin visitar (el primero) y que el
    % numero de vectores visitados es igual a el numero de vectores del grafo
    if (length(v_no_visitados)== 1) && (length(v_visitados) == tamano_matriz(1))
        % Añadir el camino hamiltoniano a la matriz caminos_h
        caminos_h(end+1,:)= v_visitados;
        
        % Determinar si existe un ciclo hamiltoniano
        % Se verifica que existe un arista que conecte al último véctor del
        % camino hamiltoniano con el primer véctor del camino hamiltoniano
        % obtenido
        comienzo = caminos_h(end,1);
        penultimo = caminos_h(end,end);
            if matriz(penultimo,comienzo)==1
                nuevo_ciclo = caminos_h(end,:);
                nuevo_ciclo(end+1) = comienzo;
                ciclos_h(end+1,:)= nuevo_ciclo;
            end
        
        
    end
   
    % Caso recursivo: Cuando aun hay vértoces por visitar para completar el
    % camino/ciclo hamiltoniano
    if length(v_no_visitados)>1
            % Encuentra los vértices  accesibles desde el vértice actual
            % éstos se encuentran en el rénglon de la matriz
            % correspondiente al vértice actual.
            vertices_accesibles = matriz(v_actual,:)
           
            % Iterar entre vertices disponibles
            for pos = (1:length(vertices_accesibles))
                if (matriz(v_actual,pos) == 1)
                    % Checar que no exista el vértice accessible en el
                    % véctor de vértices previamente visitados
                    if (isempty(find(v_visitados == pos)))
                        
                        % Llamar la función recursiva para encontrar los
                        % vértices a los que se puede conectar pos, de
                        % manera que se vaya completando una parte más del
                        % ciclo/camino hamiltoniano
                        encuentra_hamiltoniano(matriz, pos, v_no_visitados, v_visitados, 0)
                        
                    else 
                        continue
                    end
                end
            end
    end
end
            
        
        
