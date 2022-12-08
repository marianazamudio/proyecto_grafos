% Limpiar ventana de comandos y workspace 
clc;
clear all;

% Variables para guardar resultados 
caminos_h = [];
ciclos_h = [];
caminos_e = [];
ciclos_e = [];

fprintf('Ingrese la matriz de la siguiente manera:\n')
fprintf('Si A = 1 2 3\n')
fprintf('       4 5 6\n')
fprintf('\nEntonces:\n')
fprintf('A = [1,2,3; 4,5,6]\n')
fprintf('O también:\n')
fprintf('A = [1 2 3; 4 5 6]\n')

A= input('\nIngrese una matriz que represente un grafo\n');

% Verficar que la matriz que se ingreso sea cuadrada
[numReng,numColum] = size(A);
cuadrada = isequal(numReng, numColum');
%Si la matriz no es cuadrada, solicitar una nueva
if (cuadrada == 0)
    A=input('\nIngrese una matriz cuadrada\n');
end

% Numero de elementos de la matriz proporcionada
numero = numel(A);
% Definir una matriz de ceros de la misma dimension de A
matriz = zeros(size(A));

% Hacer una matriz secundaria para las que tienen aristas ponderdas
for i = (1:1:numero)
    if (A(i) > 0)
        matriz(i) = 1;
    else
        matriz(i)= 0;
    end
end
matriz;



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
        [caminos_h, ciclos_h] = encuentra_hamiltoniano(matriz, v_actual, v_no_visitados, v_visitados, 1);
    end
    % Imprimir resultados
    matriz_a_grafo(matriz, 1)
    disp(" ")
    disp("=============================================================")
    disp("En la ventana emergente se muestra el grafo ")
    disp('El grafo dirigido tiene los siguientes caminos hamiltoneanos')
    caminos_h
    disp('El grafo dirigido tiene los siguientes ciclos hamiltoneanos')
    ciclos_h
    
    
    
    % -----[ NO DIRIGIDO ]-----%
    %Si el grafo es no dirigido ver si existen ciclos y caminos eulerianos
    else
    
   
end 
 
% si la matriz esta conformada solo de 1's y 0's entonces el grafo 
% no es ponderado
if not(isempty(find (A > 1))) == 1
    if isempty(caminos_h) == 0
        matriz_resultante = caminos_h;
        [c_menor_peso, peso_menor] = encuentra_c_menores (A, matriz_resultante);
        disp('El grafo ponderado tiene los siguientes caminos de menor peso')
        c_menor_peso
        disp('El menor peso es')
        peso_menor

    end
    if isempty(ciclos_h) == 0
        matriz_resultante = ciclos_h;
        [c_menor_peso, peso_menor] = encuentra_c_menores (A, matriz_resultante);
        disp('El grafo ponderado tiene los siguientes ciclos de menor peso')
        c_menor_peso
        disp('El menor peso es')
        peso_menor
    end
end

%----------------------------------------------- %
% Función que detecta si existen caminos o ciclos eulerianos en el 
% grafo
% Entradas: 
%  matriz: representación matricial del grafo
% Salidas:
%   euleriano: 1 si es euleriano
%              0 si no es euleriano
%----------------------------------------------- %
function euleriano = es_par(matriz)
    impar = 0; % Variable para contar los vertices con grado impar.
    reng = size(matriz); % Obtención de tamaño de la matriz.
    
    for i=1:reng 
        grado = numel(find(matriz(i,:))); % Obtención del grado de cada 
                                          % vertice.
        if rem(grado,2) ~= 0    % Si se tiene residuo en la división
            impar = impar + 1;  % de grado entre dos, es impar.
        end
    end

    if impar == 0 && euleriano == 1; % Recorrido euleriano.
        euleriano = 1;
    else
        euleriano = 0;
        disp('NO ES EULERIANO')
    end
end
%-------------------------------------------------------------------------%
% Función que determina si el grafo es conexo
% Entrada:
%   matriz: representación matricial del grafo
% Salida:
%   conexión: 1 --- grafo es conexo
%             0 --- grafo disconexo
%-------------------------------------------------------------------------%
function conexion = conexo(matriz)
    Col = sum(matriz);    % Suma valores de cada columna.
    Reng = sum(matriz,2); % Suma valores de cada renglón.
    
    c = find(Col == 0, 1);   % Columnas con sumatoria igual a 0.
    r = find(Reng == 0, 1);  % Renglones con sumatoria igual a 0.
    
    if ~isempty(c) && ~isempty(r) % Buscando vector desconectado.
        disp('GRAFO DESCONECTADO')
        conexion = 0; % Desconectado
    else 
        conexion = 1; % Conectado
    end
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
            vertices_accesibles = matriz(v_actual,:);
           
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
                        encuentra_hamiltoniano(matriz, pos, v_no_visitados, v_visitados, 0);
                        
                    else 
                        continue
                    end
                end
            end
    end
end

% --------------------------------------------------------------------------------
% matriz_a _grafo
% Función que representa gráficamente l representación matricial de un
% grafo en una ventana emergente. 
% Entradas:
%   matriz:
%   dirigido: 1 --> indica que el grafo representado por la matriz es dirigido
%             0 --> indica que el grafo representado por la matriz es no dirigido
% Salidas: 
%   ventana emergente
% ---------------------------------------------------------------------------------
function matriz_a_grafo(matriz, dirigido)
    % Encontrar el tamaño de la matriz
    tamano = size(matriz);
    % Lista que almacena vértice donde empieza arista
    x = [];
    % Lista que almacena vértice donde termina arista
    y = [];
    % Determinar si el grafo es dirigido
    % Si el grafo es dirigido, se revisa toda la matriz, para almacenar en
    % 2 listas distintas el vértice donde empieza y el vertice donde
    % terminan las aristas del grafo.
    if dirigido == 1
        for fila = (1:tamano(1))
            for columna =(1:tamano(1))
                if matriz(fila,columna)==1
                    x(end+1)= fila;
                    y(end+1)=columna;
        
                end
            end
        end
        % Generar grafo dirigido
        G = digraph(x,y);
        
    end
    
    % Si el grafo es no dirigido, se revisa solo la diagonal principal y la parte superior a ésta
    % para almacenar en 2 listas distintas el vértice donde empieza y el vertice donde terminan las aristas del grafo.
    % Esto se hace para evitar generar aristas duplicadas. 
    if dirigido == 0
      for fila = (1:tamano(1))
            for columna =(1:tamano(1))
                if columna >= fila
                    if matriz(fila,columna)==1
                        x(end+1)= fila;
                        y(end+1)=columna;
                    end
                end
            end
        end  
        G = graph(x,y)  
    end
    plot(G)
end

%*************************************************************************

%------------------------------------------------------------------------
% función que encuentra los caminos y/o ciclos hamiltonianos/eulerianos de menor 
% peso del grafo con aristas ponderadas
%
% Entradas:
% A: matriz cuadrada que representa un grafo con aristas ponderadas es
%    la matriz que ingresa el usuario
% matriz_resultante: es la matriz que contiene todos los caminos y/o
%                    ciclos (sin que se repitan) del grafo
% Salidas:
% caminos_menores: es una matriz que en donde cada uno de sus renglones 
% representa un camino de menor peso
%------------------------------------------------------------------------
function [c_menor_peso, peso_menor] = encuentra_c_menores(A, matriz_resultante)
c_menor_peso = [];
% con este ciclo se revisan todos los caminos y/o ciclos del grafo
% el número de veces que se repite es igual al numero de columnas de
% la matriz resultante
 for j = (1:length(matriz_resultante(:,1)))
     % se extrae el renglón de la matriz resultante para tomarlo como
     % el camino actual
     camino_actual = matriz_resultante(j,:);
     % en la variable suma se guarda el resultado del peso del camino
     suma = 0;
     % con este ciclo se va realizando la suma de todas las aristas
     % el número de veces que se repite es igual al número de elementos 
     % del renglón menos 1
    for k = (1:length(matriz_resultante(1,:))-1)
        % se identifica el valor del elemento en la matriz que representa
        % la primera arista del camino y/o ciclo
        peso = A(matriz_resultante(j,k),matriz_resultante(j,k+1));
        % se va realizando la suma de todas las aristas del camino y/o
        % ciclo
        suma = suma + peso;
    end   
         % solo en la primera iteración se le asigna a la variable
         % peso_menor la suma obtenida
         if (j == 1)
            peso_menor = suma;
         % se agrega el camino actual a la matriz que contiene los caminos
         % y/o ciclos menores
            c_menor_peso = camino_actual;
            
         else
             % en la segunda iteración, si la nueva suma obtenida es menor
             % a la variable peso menor, significa que hay un camino de
             % menor ponderación entonces se reemplazan los valores en
             % peso_menor y en caminos menores
             if (suma < peso_menor)
               peso_menor = suma;
               c_menor_peso = camino_actual;
             % si la nueva suma obtenida es igual al valor guardado en 
             % peso_menor significa que puede haber más de un camino y/o 
             % ciclo con el mismo peso, si es el caso el camino actual se
             % agrega como un nuevo renglón a la matriz de caminos_menores
             % para no reemplazar el ya existente con el mismo peso
            elseif (suma == peso_menor)
                c_menor_peso(end + 1,:) = camino_actual;
            end
         end
 end 
end


