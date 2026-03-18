import numpy as np
from helperFunctions import objectiveComputation

#Tabu search algorithm
def search_goodsolution(initial_sol,t,s,tau,w,N,cap,tabu=True,iters=50,depot_loc=0,E=list):
    import time
    n_routes = initial_sol.copy()
    tabu_insert_list = set()
    no_more_neighbors = 0
    accepted_move = 1
    iteration = 0
    best_solution = initial_sol.copy()
    pi = {}
    best_cost = objectiveComputation.solution_cost(initial_sol,t,s,pi,tau,w,depot_loc,N)
    start_time = time.time()
    max_time = 2.0  # seconds
    max_iters = min(iters, 100)
    while True:
        for idx in range(len(n_routes)):
            if (time.time() - start_time) > max_time:
                no_more_neighbors = 1
                break
            
            if accepted_move == 1:
                route = n_routes[idx]
                insertion_paths = {}
                insertion_costs = {}
                N_scan = list(N)
                for i in route:
                    if i in N_scan:
                        N_scan.remove(i)
                for j in N_scan:
                    for pos,old_i in enumerate(route):
                        if old_i != depot_loc:
                            prv = route[pos-1]
                            nxt = route[pos+1]
                            if ((prv,j) in E)&((j,nxt) in E):
                                tabu_key = (j,idx,pos)
                                if tabu_key not in tabu_insert_list: #Only considering those that are not in the tabu list
                                    prop_route = n_routes[idx].copy()
                                    prop_route[pos] = j
                                    pi = {}
                                    cost = objectiveComputation.path_cost(prop_route,t,s,pi,tau,w,depot_loc,N)
                                    insertion_paths[(idx,j,old_i,pos)] = prop_route
                                    insertion_costs[(idx,j,old_i,pos)] = cost
                                    if tabu:
                                        tabu_insert_list.add(tabu_key)

                if len(insertion_costs) > 0:          
                    best_neighbor_insertion = np.argmin(list(insertion_costs.values())) #Getting the least cost neighbor for insertion.

                    (selected_idx,inserted_i,removed_i,path_pos) = list(insertion_paths.keys())[best_neighbor_insertion]
                    add_node = removed_i
                    addToRoute = selected_idx
                    addInPosition = path_pos
                    # Already added to tabu_insert_list above
                    #print(n_routes)
                    n_routes[idx]= insertion_paths[list(insertion_paths.keys())[best_neighbor_insertion]] #updating the route that we just changed
                    for other_idx,other_route in n_routes.items():
                        if other_idx != idx:
                            if inserted_i in other_route:
                                #print(f'Encontre {inserted_i} in {other_route} y lo vamos a eliminar')
                                path_r_pos = other_route.index(inserted_i)
                                prop_route = other_route.copy()
                                prop_route.remove(inserted_i)
                                n_routes[other_idx] = prop_route

                                add_r_node = inserted_i
                                add_r_ToRoute = other_idx
                                add_r_InPosition = path_r_pos
                                # Already added to tabu_insert_list above
                                break
                else: #We cannot improve, thus we stop and reverse to the last solution
                    accepted_move = 0
                #print(f'Vamos a intercambiar {removed_i} por {inserted_i}')
            
            
            
            #Removing the client from its original place and updating the routes 
            # (instead of directly making a pairwise substituion with freedom in the position,
            # We now can consider the least cost movement for any route with available capacity).
            #For tight capacities, this construction generalizes what we already had on a pairwise substition
            #As the only route with capacity is the one that had just removed one element. But now we consider any route with capacity.
            
            #print(n_routes)
            if accepted_move == 1: # could take a client out, we now check if we can take them into some route, otherwise, we revert to the last state
                
                #print(n_routes)
                removal_paths = {}
                removal_costs = {}
                for other_idx,other_route in n_routes.items(): # We allow to insert it in any route (even then one we just removed) with enough capacity
                    if len(other_route)-2 < cap: #Consider if capacity not full
                        prop_route = other_route.copy()
                        for pos,_ in enumerate(prop_route):
                            if (pos != 0)&(pos != len(prop_route)-1): #avoiding depot_loc
                                prv = prop_route[pos-1]
                                nxt = prop_route[pos+1]
                                if ((prv,j) in E)&((j,nxt) in E):
                                    if (removed_i,other_idx,pos) not in tabu_insert_list:
                                        prop_route = other_route.copy()
                                        prop_route.insert(pos,removed_i)
                                        removal_paths[(other_idx,removed_i,inserted_i,pos)] = prop_route
                                        pi = {}
                                        cost = objectiveComputation.path_cost(prop_route,t,s,pi,tau,w,depot_loc,N)
                                        removal_costs[(other_idx,removed_i,inserted_i,pos)] = cost
                if len(removal_costs) > 0:
                    best_neighbor_removal = np.argmin(list(removal_costs.values())) #Getting the least cost neighbor for insertion.
                    (other_r_idx,removed_r_i,inserted_r_i,path_r_pos) = list(removal_costs.keys())[best_neighbor_removal]
                    #print(f'Vamos a asignar {removed_r_i} en {n_routes[other_r_idx]}')
                    n_routes[other_r_idx] = removal_paths[list(removal_paths.keys())[best_neighbor_removal]]
                    
                    #print(n_routes)
                else:
                    accepted_move = 0

            if accepted_move == 1:
                initial_sol = n_routes.copy()
                #Check to update best route
                pi = {}
                current_cost = objectiveComputation.solution_cost(initial_sol,t,s,pi,tau,w,depot_loc,N)
                if current_cost < best_cost: # if we get better, we save the current best
                    best_solution = initial_sol.copy()
                    best_cost = current_cost


            else:
                n_routes = initial_sol.copy() #reverting to last state and breaking the loop
                no_more_neighbors = 1 #finishing execution
                #print('No more neighbors to explore')
                initial_sol = n_routes.copy()
                #Check to update best route
                current_cost = objectiveComputation.solution_cost(initial_sol,t,s,pi,tau,w,depot_loc,N)
                if current_cost < best_cost: # if we get better, we save the current best
                    best_solution = initial_sol.copy()
                    best_cost = current_cost
                break
            
            #tabu_list.append([add_node,addToRoute,addInPosition,add_r_node,add_r_ToRoute,add_r_InPosition,inserted_i,other_r_idx,path_r_pos])
            # tabu_list.append({'addNode':add_node,'addToRoute':addToRoute,'addInPosition':addInPosition,
            #                   'addRNode':add_r_node,'addRToRoute':add_r_ToRoute,'addRInPosition':add_r_InPosition,
            #                   'removeNode':inserted_i,'removeFromRoute':other_r_idx,'removeFromPosition':path_r_pos}) #adding opposite complete move to tabu list from path pos to old pos 
            #print('---------')
            iteration += 1
        if no_more_neighbors == 1:
            break
        if iteration >= max_iters:
            break

    return best_solution,best_cost

def get_best_solution(initial_sol,t,s,tau,w,N,cap,iters=500,depot_loc=0,E=list):
    best_solution_1,best_cost_1 = search_goodsolution(initial_sol,t,s,tau,w,N,cap,tabu=True,iters=iters,depot_loc=depot_loc,E=E)
    best_solution_2,best_cost_2 = search_goodsolution(initial_sol,t,s,tau,w,N,cap,tabu=True,iters=iters,depot_loc=depot_loc,E=E)
    if best_cost_1 < best_cost_2:
        best_solution = best_solution_1
    else:
        best_solution = best_solution_2
    return best_solution