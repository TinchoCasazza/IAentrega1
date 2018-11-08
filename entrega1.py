
# coding: utf-8
from simpleai.search import astar, breadth_first, SearchProblem,depth_first,greedy
from simpleai.search.viewers import WebViewer, BaseViewer

orillas = ((0,0),(0,1),(0,2),(0,3),(0,4),(0,5),(1,0),(2,0),(3,0),(4,0),(5,0),(5,1), (5,2),(5,3),(5,4),(5,5),(4,5),(3,5),(4,5))

state = ((0,0),((2,1),(3,4),(4,2)),())
        #((POS ROBOT),(POS PEOPLE ),(POS BLOQUES YA PASE))

class TpProblem(SearchProblem):

    def is_goal(self, state):
        # Es meta cuando la cantidad de personas a rescatar es 0 
        # y la posicion del robot esta en una de las orillas
        return len(state[1]) == 0 and state[0] in orillas

    def cost(self,s1,action,s2):
        return 1

    def actions(self,state):
        #Las acciones posibles son movimientos a casillas adyacentes
        row, col = state[0]
        actions = []
        if row > 0:
            if is_valid((row -1, col), state):
                actions.append((row -1, col ))
        if row < 5:
            if is_valid((row +1, col), state):
                actions.append((row+1, col ))
        if col > 0:
            if is_valid((row, col -1), state):
                actions.append((row, col  -1))
        if col < 5:
            if is_valid((row, col +1), state):
                actions.append((row, col + 1))
        if len(state[1]) != 0 and state[0] != (0,0) :
            for action in actions:
                if action in orillas:
                    actions.remove(action)
        return tuple(actions)


    def result(self,state,action):
        #En el result se cambia la posicion del robot, se borra una
        #Persona del state[1] si es que esta en una posicion donde hay una persona
        #Y si es un casillero que no es una orilla, si agrega a la lista de casilleros
        #Visitados para que no pueda volver a pasar por ahi
        row_action, col_action = action
        support_state = list(state)
        if (row_action, col_action) in state[1]:
            count = 0
            aux_state = list(state[1])
            for people in aux_state:
                if people == (row_action, col_action):

                    del aux_state[count]
                count += 1
            support_state[1] = tuple(aux_state)
        support_state[0] = action
        aux_state = list(state[2])
        if (row_action, col_action) not in orillas:
            aux_state.append(action)
            support_state[2] = tuple(aux_state)
        state = tuple(support_state)
        return state

    def heuristic(self,state):
        # La heuristica es la distancia desde el robot a la persona mas lejana
        # Sumada la distancia de esa persona mas lejana con la orilla mas cercana a la misma
        # Como extra, y segun algunas pruebas que hicimos tambien decidimos sumar la cantidad
        # de personas que quedaban rescatar, con esto obtuvimos mejores resultados
        distances = []
        people_distance = []

        more_distance = 0
        if len(state[1]) > 0:
            for people in state[1]:
                distances.append(people)
            people_distance = [manhattan(x, state[0]) for x in distances]
            more_distance = max(people_distance)
            farther_person = state[1][people_distance.index(more_distance)]
        distances.clear()
        for pos in orillas:
            distances.append(pos)
        if more_distance != 0:
            minium_distance = min([manhattan(x, farther_person) for x in distances])
        else:
            minium_distance = min([manhattan(x, state[0]) for x in distances])
        return minium_distance + more_distance + len(state[1])



def is_valid(pos,state):
    #Valida que la posicion a la que me muevo no este en la lista de casillas
    # Ya visitadas
    for pos_traveled in state[-1]:
        if pos == pos_traveled:
            return False
    return True

def manhattan(pos1,pos2):
    x1, y1 = pos1
    x2, y2 = pos2
    return (abs(x2 - x1) + abs(y2 - y1))


def resolver(metodo_busqueda,posiciones_personas):
    state = ((0,0),(posiciones_personas),())

    my_viewer = BaseViewer()
    if 'astar':
        result = astar(TpProblem(state), graph_search=True, viewer=my_viewer)
    elif 'breadth_first':
        result = breadth_first(TpProblem(state), graph_search=True, viewer=my_viewer)
    elif 'depth_first':
        result = depth_first(TpProblem(state), graph_search=True, viewer=my_viewer)
    elif 'greedy':
        result = greedy(TpProblem(state), graph_search=True, viewer=my_viewer)

    return result

if __name__ == "__main__":
    my_viewer = BaseViewer()
    state = ((0,0),((2, 1), (3, 4), (4, 2)),())
    result = breadth_first(TpProblem(state), graph_search=True, viewer=my_viewer)
    print("Stats: ",my_viewer.stats)
    print("Solucion: ",result.state)
    print("profundidad_solucion: ",len(result.path()))
    print("costo_solucion:", result.cost)
    
    
    
