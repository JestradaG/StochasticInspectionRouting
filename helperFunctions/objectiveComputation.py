
def path_cost(path,t,s,pi,tau,w,depot_loc,N,report=False):

    if len(pi) == 0:
        pi = {}
        for i in N:
            pi[i] = 0
    
    cost_route = 0
    cost_prob = 0
    a = {depot_loc:0} #Arrival time
    q = {}
    for pos,orig in enumerate(path[:-1]):
        dest = path[pos+1]
        #print('Registering cost from',orig,'->',dest,'with cost',t[orig,dest])
        cost_route += t[orig,dest]-pi[dest]#*drone_data['pk'].iloc[i]
        a[dest] = t[orig,dest]+a[orig]+s[orig]
        
        if dest == 0:
            q[dest] = 0
        elif a[dest] > tau[dest]:
            q[dest] = 1 #registering late
        else:
            q[dest] = 0
        
        cost_prob += (w[dest]-pi[dest])*q[dest]

    objective = cost_route+cost_prob
    if report == False:
        return objective
    else:
        return objective,q

def solution_cost(solution,t,s,pi,tau,w,depot_loc,N,report=False): #Most simple cost computation
    tot_cost = 0
    if report == True:
        fails = {}
    for idx,path in enumerate(solution.values()):
        if report == False:
            tot_cost += path_cost(path,t,s,pi,tau,w,depot_loc,N)
        else:
            cost,q = path_cost(path,t,s,pi,tau,w,depot_loc,N,True)
            fails[idx] = q
            tot_cost += cost
    if report ==False:
        return tot_cost
    else:
        return tot_cost,fails
    
def expected_objective(IP,x,drone_num,num_scenarios,s_realizations_r,tau_realizations_r,t,s,w,depot_loc,N,report = False):
    routes = {}
    for (i,j,k),val in x.items():
        if k not in routes.keys():
            routes[k] = {}
    for drone in routes.keys():
        for (i,j,k),val in x.items():      
            if k == drone:
                if IP == True:
                    if val.X >= 0.5:
                        routes[drone][(i,j)] = 1
                else:
                    if val >= 0.5:
                        routes[drone][(i,j)] = 1
    #Registering new route and adding it to possible routes
    paths = {}
    for drone in routes.keys():
        solution_arcs = list(routes[drone].keys())
        curr_node = 0
        path = [curr_node] #Adding new route
        while solution_arcs:
            for (i,j) in solution_arcs:
                if i == curr_node:
                    path.append(j)
                    curr_node = j
                    solution_arcs.remove((i,j))
        paths[drone] = path

    objective = 0
    if report == True:
        fail_stats = {}
    for scenario in range(num_scenarios):
        s = s_realizations_r[scenario]
        tau = tau_realizations_r[scenario]
        if report == False:
            objective += solution_cost(paths,t,s,{},tau,w,depot_loc,N)/num_scenarios
        else:
            obj,q = solution_cost(paths,t,s,{},tau,w,depot_loc,N,True)
            fail_stats[scenario] = q
            objective += obj/num_scenarios
    if report == False:
        return objective
    else:
        return objective,fail_stats