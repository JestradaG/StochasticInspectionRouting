import random

def initial_feasible(N,front_sets,E,depot_loc,cap,drone_num):
    nodes2visit = list(N)[1:]
    r_routes = {}
    route_counter = 0
    feasible_solution = False
    global_dead_end = False
    dead_ends = 0
    max_dead_ends = (2**len(N))/(len(N))
    while feasible_solution == False:
        i = depot_loc
        path = [i]
        feasible_path = False
        node_count = 0
        removed_nodes = [] #keeping the list of nodes removed in this path in case of failure
        while feasible_path == False:
            options = list(set(front_sets[i]) & set(nodes2visit))

            if (len(path) == cap+1)|(len(nodes2visit) == 0):
                if depot_loc in front_sets[i]:
                    path.append(depot_loc)
                    #print(f'Getting arc back {i,depot_loc}')
                    feasible_path = True
                else:
                    if len(r_routes) >= 1:
                        #print('Global infeasibility!! 1')
                        global_dead_end = True
                        feasible_path = True
                        break
                    else: #only a local infeasibility
                        #print('Local infeasibility')
                        feasible_path = False
                        i = depot_loc
                        path = [i]
                        nodes2visit = list(set(nodes2visit+removed_nodes)) #restoring the removed nodes in case of failure


            elif len(options) > 0:
                j = random.choice(options)
                if j in nodes2visit:
                    if (i,j) in E:
                        #print(f'we find a valid {i,j} thats not yet visited')
                        path.append(j)
                        nodes2visit.remove(j)#REmoving i from nodes2visit
                        removed_nodes.append(j)
                        i = j
            
            else:
                if len(r_routes) >= 1:
                    #print('Global infeasibility!! 2')
                    global_dead_end = True
                    feasible_path = True
                    break

                else: #only a local infeasibility
                    #print('Local infeasibility')
                    feasible_path = False
                    i = depot_loc
                    path = [i]
                    nodes2visit = list(set(nodes2visit+removed_nodes)) #restoring the removed nodes in case of failure
            

        r_routes[route_counter] = path
        route_counter += 1
        #print(nodes2visit)

        if global_dead_end == True:
            #print('We reach a gobal dead end. Restarting the solutions globally')
            r_routes = {}
            nodes2visit = list(N)[1:]
            route_counter = 0
            global_dead_end = False
            dead_ends += 1
            if dead_ends >= max_dead_ends:
                break


        if (route_counter == drone_num)|(len(nodes2visit)==0):
            feasible_solution = True
    
    return r_routes

def infeasibility_check(r_routes,N,depot_loc):
    infeasible = 0
    visited_nodes = []
    unique_nodes = []
    for idx in range(len(r_routes)):
        for pos,i in enumerate(r_routes[idx][:-1]):
            j = r_routes[idx][pos+1]
            visited_nodes.append(j)
            if j != depot_loc:
                if j in unique_nodes:
                    #print(f'repeated node {j} infeasible solution!')
                    infeasible = 1
                    break
                else:
                    unique_nodes.append(j)
        if infeasible == 1:
            break
    if infeasible == 0:
        visited_nodes = list(set(visited_nodes))
        if len(visited_nodes) < len(N):
            #print('Infeasible because we miss nodes')
            infeasible = 1

    return infeasible