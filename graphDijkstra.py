

## u = vertice inicial/origem

## v = vertice final/destino

## tamanho = numero de arestas

## ordem = numero de vertices

## peso =  valor das arestas

## Ponto 0: Construtor do Grafo de lista de adjacencias
from collections import defaultdict
import heapq

class Grafo:
    def __init__(self, direcionado=False, ponderado=False ):
        self.adjacente_list = defaultdict(list)
        self.tamanho = 0 
        self.ordem = 0
        self.direcionado = direcionado
        self.ponderado = ponderado
        
    ## Ponto A: Função adciona vertice, faz o teste da existencia do mesmo e cria a vertice
    def adiciona_vertice(self, u):
        if u not in self.adjacente_list:
            self.adjacente_list[u]
            self.ordem += 1
            print(f"Vértice '{u}' adicionado ao grafo.")
        else:
            print(f"Vértice '{u}' já existe no grafo.")
            
    ## Ponto B: Função que adciona arestas
    def adiciona_aresta(self, u, v, peso=1):
        if not self.ponderado:
            peso = 1  # Se o grafo não é ponderado, usar peso padrão 1
        if not self.tem_vertice(u):
            self.adiciona_vertice(u)
        if not self.tem_vertice(v):
            self.adiciona_vertice(v)
        self.adjacente_list[u].append((v, peso))
        if not self.direcionado:
            self.adjacente_list[v].append((u, peso))  # Adiciona a aresta reversa para grafos não direcionados
        self.tamanho += 1
        print(f"Aresta adicionada de '{u}' para '{v}' com peso {peso}.")    

    #Nova Função que retorna a quantidade máxima de arestas que o grafo pode possuir
    def get_max_arestas(self):
        if self.direcionado:
            return self.ordem * (self.ordem - 1)
        else:
            return self.ordem * (self.ordem - 1) // 2
            
    #Nova Função que retorna uma lista com os vértices adjacentes ao vértice u
    def retorna_adjacentes(self, u):
        if u in self.adjacente_list:
            return [v for v, _ in self.adjacente_list[u]]
        else:
            return []
            
    ##Ponto C: Função para remoção de arestas do vertice u para vertice v
    def remove_aresta(self, u, v):
        if u in self.adjacente_list:
            nova_lista = []
            for destino, peso in self.adjacente_list[u]:
                if destino != v:
                    nova_lista.append((destino, peso))
            if len(nova_lista) == len(self.adjacente_list[u]):
                print(f"Nenhuma aresta de '{u}' para '{v}' foi encontrada.")
            else:
                self.tamanho -= 1
                print(f"Aresta de '{u}' para '{v}' removida.")
            self.adjacente_list[u] = nova_lista
        else:
            print(f"Vértice '{u}' não existe no grafo.")

    #Ponto D: Função que retira todos os vertices
    def remove_vertice(self, u):
        if u in self.adjacente_list:
            self.tamanho -= len(self.adjacente_list[u])
            del self.adjacente_list[u]
            self.ordem -= 1
            print(f"Vértice '{u}' e todas as arestas saindo dele foram removidos.")
        for vertice in list(self.adjacente_list.keys()):
            nova_lista = []
            for destino, peso in self.adjacente_list[vertice]:
                if destino != u:
                    nova_lista.append((destino, peso))
            if len(nova_lista) != len(self.adjacente_list[vertice]):
                self.tamanho -= (len(self.adjacente_list[vertice]) - len(nova_lista))
            self.adjacente_list[vertice] = nova_lista
        print(f"Todas as arestas entrando em '{u}' foram removidas")

    #Ponto E: Função de verificação de arestas entres os vertices u e v
    def tem_aresta(self, u, v):
        if u in self.adjacente_list:
            for destino, _ in self.adjacente_list[u]:
                if destino == v:
                    print(f"Existe uma aresta de '{u}' para '{v}'.")
                    return True
            print(f"Não existe aresta de '{u}' para '{v}'.")
            return False

    # Função que verifica na lista de adjacencias a existencia do vertice
    def tem_vertice(self, u):
        return u in self.adjacente_list

    #Ponto F: Função que contabiliza aresta que entram naquele vertice
    def grau_entrada(self, u):
        contador = 0
        for vertice in self.adjacente_list:
            for destino, _ in self.adjacente_list[vertice]:
                if destino == u:
                    contador += 1
        print(f"O grau de entrada do vértice '{u}' é {contador}.")
        return contador

    #Ponto G: Função que contabiliza aresta que saem naquele vertice
    def grau_saida(self, u):
        if u in self.adjacente_list:
            grau = len(self.adjacente_list[u])
            print(f"Grau de saída do vértice '{u}': {grau}")
            return grau
        else:
            print(f"Vértice '{u}' não existe no grafo.")
            return 0
    #Ponto H: Função que define qual o grau daquele vertice totalizando a soma dos graus de saida e entrada
    def grau(self, u):
        if u not in self.adjacente_list:
            print(f"Vértice '{u}' não existe no grafo.")
            return 0
        grau_entrada = self.grau_entrada(u)
        grau_saida = self.grau_saida(u)
        total_grau = grau_entrada + grau_saida
        print(f"Grau total do vértice '{u}': {total_grau}")
        return total_grau
        
    #Ponto I: Função gera os valores para arestas ponderadas
    def get_peso(self, u, v):
        if self.tem_aresta(u, v):
            for destino, peso in self.adjacente_list[u]:
                if destino == v:
                    print(f"Peso da aresta de '{u}' para '{v}': {peso}")
                    return peso

    #Ponto J: Função que imprime a lista de adjacencias
    def imprime_lista_adjacencias(self):
        print("Lista de adjacências do grafo:")
        for vertice, arestas in self.adjacente_list.items():
            arestas_str = " -> ".join(f"('{destino}', {peso})" for destino, peso in arestas) + " ->" if arestas else ""
            print(f"{vertice}: {arestas_str}")

    #Função Dijkstra
    def Dijkstra(self, source_node, destination_node):
        distances = {}
        for vertex in self.adjacente_list:
            distances[vertex] = float('infinity')

        previous_nodes = {}
        for vertex in self.adjacente_list:
            previous_nodes[vertex] = None

        distances[source_node] = 0
        pq = [(0, source_node)] 

        while pq:
            current_distance, current_vertex = heapq.heappop(pq)
            
            
            for neighbor, weight in self.adjacente_list[current_vertex]:
                distance = current_distance + weight
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous_nodes[neighbor] = current_vertex
                    heapq.heappush(pq, (distance, neighbor))

            if current_vertex == destination_node:
                break

        shortest_path = []
        current_vertex = destination_node
        while current_vertex is not None:
            shortest_path.insert(0, current_vertex)
            current_vertex = previous_nodes[current_vertex]

        if shortest_path and shortest_path[0] == source_node:
            shortest_path_cost = distances[destination_node]
        else:
            shortest_path_cost = float('infinity')

        return (shortest_path, shortest_path_cost)

    
grafo = Grafo()

##Chamadas das funções implemntadas
grafo.adiciona_vertice("A")
grafo.adiciona_vertice("B")
grafo.adiciona_vertice("C")

grafo.adiciona_aresta("A", "B", 1)
grafo.adiciona_aresta("B", "C", 2)
grafo.adiciona_aresta("C", "A", 3)

grafo.grau_entrada("A")
grafo.grau_saida("A")
grafo.grau("A")

exists = grafo.tem_aresta("B", "C")
print(f"Aresta de 'B' para 'C' existe? {exists}")

grafo.get_peso("C", "A")

grafo.imprime_lista_adjacencias()

grafo.remove_aresta("A", "B")

grafo.remove_vertice("C")

exists = grafo.tem_aresta("B", "C")
print(f"Aresta de 'B' para 'C' existe? {exists}")

grafo.imprime_lista_adjacencias()

print("\n")

#Chamando a função Dijkstra
grafo.adiciona_vertice("A")
grafo.adiciona_vertice("B")
grafo.adiciona_vertice("C")
grafo.adiciona_vertice("D")
grafo.adiciona_vertice("E")

grafo.adiciona_aresta("A", "B", 2)
grafo.adiciona_aresta("A", "C", 4)
grafo.adiciona_aresta("B", "C", 1)
grafo.adiciona_aresta("B", "D", 7)
grafo.adiciona_aresta("C", "D", 3)
grafo.adiciona_aresta("C", "E", 3)
grafo.adiciona_aresta("D", "E", 1)
grafo.adiciona_aresta("E", "A", 8)
grafo.adiciona_aresta("B", "E", 5)
grafo.adiciona_aresta("D", "A", 2)

# Chamada para encontrar o menor caminho de 'A' para 'E'
caminho, custo = grafo.Dijkstra("A", "E")
print(f"O menor caminho de A para E é {caminho} com custo {custo}")

print("\n")

# Criando uma instância de um grafo direcionado e ponderado
grafo_direcionado = Grafo(direcionado=True, ponderado=True)
grafo_direcionado.adiciona_vertice("A")
grafo_direcionado.adiciona_vertice("B")
grafo_direcionado.adiciona_vertice("C")
grafo_direcionado.adiciona_aresta("A", "B", 5)
grafo_direcionado.adiciona_aresta("B", "C", 3)

# Criando uma instância de um grafo não direcionado e não ponderado
grafo_nao_direcionado = Grafo(direcionado=False, ponderado=False)
grafo_nao_direcionado.adiciona_vertice("A")
grafo_nao_direcionado.adiciona_vertice("B")
grafo_nao_direcionado.adiciona_vertice("C")
grafo_nao_direcionado.adiciona_aresta("A", "B")
grafo_nao_direcionado.adiciona_aresta("B", "C")

# Utilização das duas funções
print("Max arestas em grafo direcionado: ", grafo_direcionado.get_max_arestas())
print("Adjacências de 'B' em grafo direcionado: ", grafo_direcionado.retorna_adjacentes("B"))

print("Max arestas em grafo não direcionado: ", grafo_nao_direcionado.get_max_arestas())
print("Adjacências de 'B' em grafo não direcionado: ", grafo_nao_direcionado.retorna_adjacentes("B"))
