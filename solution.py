#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

#from asyncio.windows_events import INFINITE
#from logging import root
import os  # for time functions
import math  # for infinity
#import heapq
from search import *  # for search engines
from sokoban import sokoban_goal_state, SokobanState, Direction, PROBLEMS  # for Sokoban specific classes and problems

def sokoban_goal_state(state):
    '''
    @return: Whether all boxes are stored.
    '''
    i = 0
    same = 0
    for boxen in state.boxes:
        i += 1
        for storage in state.storage:
            if storage == boxen:
                same += 1
    if same == i:
        return True
    else:
        return False       

    #pass #CHANGE THIS!

def heur_manhattan_distance(state):
    # IMPLEMENT
    dist_sum = 0
    for position in state.boxes:
        mini = math.inf
        #Loop through each box distance
        for storage_pos in state.storage:
            #Find the distance between each storage spot and determine 
            temp = abs(position[0] - storage_pos[0]) + abs(position[1] - storage_pos[1])
            if temp < mini:
                mini = temp
        dist_sum += mini


    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # We want an admissible heuristic, which is an optimistic heuristic.
    # It must never overestimate the cost to get from the current state to the goal.
    # The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    # When calculating distances, assume there are no obstacles on the grid.
    # You should implement this heuristic function exactly, even if it is tempting to improve it.
    # Your function should return a numeric value; this is the estimate of the distance to the goal.
    return dist_sum  # CHANGE THIS

# SOKOBAN HEURISTICS
def trivial_heuristic(state):
    '''trivial admissible sokoban heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
    return 0  # CHANGE THIS

def heur_alternate(state):
    # IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # heur_manhattan_distance has flaws.
    # Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    # Your function should return a numeric value for the estimate of the distance to the goal.

    #DESCRIPTION: This heuristic takes into account that a storage placement can only hold one box and takes into account that storage boxes that are on edges cannot be moved to a solution location outside of those edges

   #Treat other boxes as obstacles  
    def edge_check(state, box_pos, solution):
        if (((box_pos[0] != solution[0]) and (box_pos[0] == (state.width - 1) or  box_pos[0] == 0))) or ((box_pos[1] != solution[1]) and (box_pos[1] == (state.height - 1) or box_pos[1] == 0)):
            return math.inf
        return 0
            
    
    dist_sum = 0
    used_storage_list = []
    #used_box_list = []
    best_storage = 0
    for storage_pos in state.storage:
        mini = math.inf
        #Loop through each box distance
        for boxen in state.boxes:
            #Find the distance between each storage spot and determine
            if used_storage_list.count(boxen) > 0:
                continue
            else:
                temp = abs(boxen[0] - storage_pos[0]) + abs(boxen[1] - storage_pos[1]) + edge_check(state, boxen, storage_pos)
                if ((temp < mini) and (used_storage_list.count(boxen) == 0)):
                    mini = temp
                    best_storage = boxen
        used_storage_list.append(best_storage)
        dist_sum += mini
      
    return dist_sum  # CHANGE THIS


def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    #IE Djikstras
    return 0

def fval_function(sN, weight):
    # IMPLEMENT
    #Heuristic Value
    h = sN.hval
    #Cost of path to node
    g = sN.gval
    returner = g + weight * h
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    return returner #CHANGE THIS

# SEARCH ALGORITHMS
def weighted_astar(initial_state, heur_fn, weight, timebound):
    #heur_fn returns heuristic value
    #
    '''Provides an implementation of weighted a-star, as described in the HW1 handout'''
    '''INPUT: a warehouse state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of weighted astar algorithm'''
    custom_search = SearchEngine(strategy='custom', cc_level='default') #Type of cycle checking, also with "custom" we have to specify the way that f values are calulated
    #Set up specific search with
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))

    custom_search.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)
    #Initial state is of object type StateSpace, start state
    costbound = [math.inf, math.inf, math.inf]
    var = custom_search.search(timebound, costbound) #Returns solution path and a SearchStats object (if solution is found) 
    path = var[0]
    search_stat = var[1]

    return path, search_stat  # CHANGE THIS

def iterative_astar(initial_state, heur_fn, weight=1, timebound=5):  # uses f(n), see how autograder initializes a search line 88
    # IMPLEMENT
    #Questions
    #When do we know there are no more nodes left???
    '''Provides an implementation of realtime a-star, as described in the HW1 handout'''
    '''INPUT: a warehouse state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of realtime astar algorithm'''
    path = False
    search_stat = False
    costbound = [math.inf, math.inf, math.inf]
    i = 1 
    while True:
        start_time = os.times()[0]
        var = weighted_astar(initial_state, heur_fn, weight, timebound)
        end_time = os.times()[0]
        timebound -= end_time - start_time
        if timebound <= 0:
            break
        start_time = os.times()[0]
        if var[0] == False: #We have not found a solution
            break
            '''
            if (timebound == 0):
                #We will return
                break
            else:
                #Do we need to continue in the loop when we cant find a solution with weight?
                break
            '''
        else:
            #We have found a path
            if path == False:
                #We initialize our values
                path = var[0]
                search_stat = var[1]
            if path.gval > var[0].gval: #New path is better (less distance)
                path = var[0]
                search_stat = var[1]
            if costbound[2] > path.gval: #We prune based on the cost of the BEST path to the goal
                costbound[2] = path.gval
        end_time = os.times()[0]
        timebound -= (end_time - start_time)
        if (timebound <= 0): #QQQQQIs there somehow we can account for time in the next search to prevent exceeding timebounds???
            break
        weight = weight * 0.8

    return path, search_stat  # CHANGE THIS
    

def iterative_gbfs(initial_state, heur_fn, timebound=5):  # only use h(n)
    # IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of weighted astar algorithm'''
    
    path = False 
    search_stat = False
    costbound = [math.inf, math.inf, math.inf]
    i = 1
    var = [True, True]
    while True: #
        custom_search = SearchEngine(strategy='best_first', cc_level='full') #QQQQQQ???????Do we use BF????And what cycle??
        #Set up specific search with
        weight = 1
        wrapped_fval_function = (lambda sN: fval_function(sN, weight))#Wont be used because we are not using custom

        custom_search.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function) #Do I need an fval here?
        #Initial state is of object type StateSpace, start state
        start_time = os.times()[0]
        var = custom_search.search(timebound, costbound) #Returns solution path and a SearchStats object (if solution is found)
        end_time = os.times()[0]
        elapsed_time = end_time - start_time
        timebound -= elapsed_time
        
        if var[0] == False: #We have not found a path
            break
        else:
            if path == False:
                #We initialize our values
                path = var[0]
                search_stat = var[1]            
            #When this loop has run more than once we can compare to previous result
            if path.gval > var[0].gval: #New path is better (less distance)
                path = var[0]
                search_stat = var[1]
        
            #Update our bounds with ..
            if costbound[0] > path.gval: #We prune based on g value with the cost of the BEST path to the goal
                costbound[0] = path.gval
        if (timebound <= 0):
            break
    return path, search_stat
    #How do we expand nodes with lowest h(node) projected distance from start to finish  A: Im guessing by using best first we remedy this
    
    #return None, None #CHANGE THIS
