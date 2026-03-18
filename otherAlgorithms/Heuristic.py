import pandas as pd
import random
def heuristic_solve(node_num,A,depot_loc,tau_realizations,s_realizations,t,num_scenarios,cap,drone_num):
    #Heuristic based on distance and failing times
    tau_means = {}
    s_means = {}
    for node in range(node_num):
        tau_means[node] = sum(tau_realizations[node].values())/num_scenarios
        s_means[node] = sum(s_realizations[node].values())/num_scenarios

    #Parallel distribution
    #organizing by distance from origin and service time and deadline
    ratios = {}
    for node in range(node_num):
        if node == depot_loc:
            continue
        else:
            ratios[node]=(t[depot_loc,node]+s_means[node])/tau_means[node]
    rat_df = pd.DataFrame(ratios,index=[0]).T

    feasible = False

    
    order = list(rat_df[0].sort_values(ascending=False).index)

    while feasible == False:
        to_visit = order.copy()
        x_heuristic = {drone: [depot_loc] for drone in range(drone_num)}
        max_iters = 1000
        iters = 0
        available_drones = list(range(drone_num))
        # Track which drones are still available in each round
        while len(to_visit) > 0 and len(available_drones) > 0:
            next_available_drones = []
            for drone in available_drones:
                if len(x_heuristic[drone]) < cap + 1 and len(to_visit) > 0:
                    assigned = False
                    pos = 0
                    while not assigned and pos < len(to_visit):
                        if (x_heuristic[drone][-1], to_visit[pos]) in A:
                            x_heuristic[drone].append(to_visit[pos])
                            to_visit.pop(pos)
                            assigned = True
                        else:
                            pos += 1
                        if iters >= max_iters:
                            print('Warning! could not find any heuristic path!!')
                            to_visit = []
                            break
                        iters += 1
                    # If assigned, drone can continue in next round if not full
                    if assigned and len(x_heuristic[drone]) < cap + 1:
                        next_available_drones.append(drone)
                    # If not assigned and not full, drone is not available anymore
                    elif not assigned:
                        pass  # drone not added to next_available_drones
                    else:
                        pass  # drone reached capacity
                # If drone is already full, do not add to next_available_drones
            available_drones = next_available_drones
        # After assignment, return all drones to depot
        for drone in range(drone_num):
            if x_heuristic[drone][-1] != depot_loc:
                x_heuristic[drone].append(depot_loc)

        x_heu = {}
        for drone in range(drone_num):
            for i in range(len(x_heuristic[drone]) - 1):
                x_heu[(x_heuristic[drone][i], x_heuristic[drone][i + 1], drone)] = 1

        visited_nodes = set()
        for (i, j, k), val in x_heu.items():
            visited_nodes.add(i)
            visited_nodes.add(j)
        if len(sorted(visited_nodes)) == node_num:
            feasible = True
        else:
            # Randomly swap order to try a new permutation
            for _ in range(3):
                if len(order) > 1:
                    i = random.choice(list(range(int(len(order)/2))))
                    j = random.choice(list(range(int(len(order)/2), len(order))))
                    order[i], order[j] = order[j], order[i]
    return x_heu