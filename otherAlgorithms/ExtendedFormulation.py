import gurobipy as gp
from gurobipy import GRB


def solve2optimality(N,K,A,back_sets,front_sets,depot_loc,t,s,tau,w,cap,bigM,num_scenarios):
    #Setting the usual vrp for shortest path
    IPmod = gp.Model("simple_model")
    IPmod.reset()
    IPmod.Params.TIME_LIMIT = 900
    #Loading parameters
    #Parameters for drones
    IPmod._cap = cap
    IPmod._K = K
    IPmod._N = N
    IPmod._A =list(set(A))
    #print(front_sets)
    #print(back_sets)
    IPmod._A_unq = []
    for i in IPmod._A:
        if (i[0] != i[1]):
            if i[1] == depot_loc:
                continue
            else:
                IPmod._A_unq.append(i)
    IPmod._back_sets = back_sets
    IPmod._front_sets = front_sets

    IPmod._s = s
    IPmod._bigM = bigM
    IPmod._tau = tau
    IPmod._w = w
    IPmod._omega = range(num_scenarios)


    IPmod._x = IPmod.addVars(IPmod._A,IPmod._K, vtype=GRB.BINARY, name="x") #Routing constraints
    IPmod._u = IPmod.addVars(N,K,vtype=GRB.CONTINUOUS,name='u') #MTZ constraints

    IPmod._a = IPmod.addVars(N,IPmod._omega,vtype=GRB.CONTINUOUS,name='a') #Arrival time variable
    IPmod._q = IPmod.addVars(N,IPmod._omega,vtype=GRB.BINARY,name='q') #Late arrival indicator
    #IPmod._y = IPmod.addVars(N,vtype=GRB.CONTINUOUS,name='y') #Total penalty variable

    #IPmod.setObjective(alpha*(gp.quicksum(t[i,j]*IPmod._x[i,j,k] for (i,j)  in A for k in K) ) + (1-alpha)*(gp.quicksum(IPmod._w[i]*IPmod._y[i] for i in N)), GRB.MINIMIZE)
    IPmod.setObjective((gp.quicksum(t[i,j]*IPmod._x[i,j,k] for (i,j)  in A for k in K) ) + ((1/len(IPmod._omega))*gp.quicksum(IPmod._w[i]*IPmod._q[i,omega] for i in N for omega in IPmod._omega)), GRB.MINIMIZE)
    for j in N:
        if j != depot_loc:
            IPmod.addConstr(gp.quicksum(IPmod._x[i,j,k] for i in back_sets[j] for k in IPmod._K) == 1, name="exit"+str(j))
    for i in N:
        if i != depot_loc:
            IPmod.addConstr(gp.quicksum(IPmod._x[i,j,k] for j in front_sets[i] for k in IPmod._K) == 1, name="enter"+str(i))
    for i in N:
        for k in K:
            IPmod.addConstr( gp.quicksum(IPmod._x[i,j,k] for j in N if (i,j) in IPmod._A) - gp.quicksum(IPmod._x[j,i,k] for j in N if (j,i) in IPmod._A) == 0, name=f'flow_balance_{i}_{k}')

    for k in K:
        IPmod.addConstr(gp.quicksum(IPmod._x[i,j,k] for (i,j) in IPmod._A) <= IPmod._cap+1)

    #Subtour elimination constraints MTZ
    for i in N:
        for j in range(1,len(N)):
            if (i,j) in IPmod._A:
                IPmod.addConstrs(IPmod._u[i,k]+IPmod._x[i,j,k] <= IPmod._u[j,k] + (len(N)-1)*(1-IPmod._x[i,j,k]) for k in K)
    IPmod.addConstrs(IPmod._u[depot_loc,k] == 0 for k in K)

    for omega in IPmod._omega:
        for (i,j) in IPmod._A_unq: #A_unq will not take into consideration trips to depot 0 to achieve feasibility
            
            IPmod.addConstr(IPmod._a[j,omega] >= (IPmod._a[i,omega]+IPmod._s[i][omega]+t[i,j])*gp.quicksum(IPmod._x[i,j,k] for k in K))
        IPmod.addConstr(IPmod._a[depot_loc,omega] == 0)

        #checking if time goal was broken
        for i in N:
            IPmod.addConstr(IPmod._a[i,omega]-IPmod._bigM[omega]*IPmod._q[i,omega] <= IPmod._tau[i][omega])

    #Computing probability of failure
    # for i in N:
    #     IPmod.addConstr(IPmod._y[i] == (IPmod._l[i]*IPmod._a[i]+IPmod._b[i])*(1-IPmod._q[i])+IPmod._q[i])
    IPmod.Params.LogToConsole = 0
    IPmod.Params.LazyConstraints = 0
    IPmod.optimize()
    IPmod._x_sols = IPmod.getAttr('x',IPmod._x)

    return IPmod
