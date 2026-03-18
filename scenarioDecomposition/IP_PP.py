import gurobipy as gp
from gurobipy import GRB
from itertools import combinations

# Callback - use lazy constraints to eliminate sub-tours
def subtourelim(model, where):
    if where == GRB.Callback.MIPSOL:
        # make a list of edges selected in the solution
        vals = model.cbGetSolution(model._vars)
        selected = gp.tuplelist((i, j) for i, j in model._vars.keys()
                                if vals[i, j] > 0.5)
        # find the shortest cycle in the selected edge list
        tour = subtour(selected,model._n)
        if len(tour) < model._n: #Activating subtour elimination if subtour was detected
            # add subtour elimination constr. for every pair of cities in tour
            model.cbLazy(gp.quicksum(model._vars[i, j]
                                    for i, j in combinations(tour, 2))
                        <= len(tour)-1)

    if model._fast == True:     
        if where == GRB.Callback.MIP:
            objbst = model.cbGet(GRB.Callback.MIP_OBJBST)
            if objbst < -0.001:
                #print('We already got a good column. Early stop for model')
                model.terminate()

# Given a tuplelist of edges, find the shortest subtour
def subtour(edges,n):
    unvisited = list(range(n))
    cycle = range(n+1)  # initial length has 1 more city
    while unvisited:  # true if list is non-empty
        thiscycle = []
        neighbors = unvisited
        while neighbors:
            current = neighbors[0]
            thiscycle.append(current)
            unvisited.remove(current)
            neighbors = [j for i, j in edges.select(current, '*')
                        if j in unvisited]
        if len(cycle) > len(thiscycle):
            cycle = thiscycle
    return cycle

#MIP Pricing Problem
class Init_IP_PP:
    
    def __init__(self,id=''):
        self.m = gp.Model(f'PricingProblem{id}')
    
    def load_data(self,depot_loc,cap,N,K,E,front_sets,back_sets,t,s,tau,w,bigM):
        self.m._depot_loc,self.m._cap,self.m._N = depot_loc,cap,N
        self.m._K,self.m._E,self.m._back_sets = K,E,back_sets
        self.m._front_sets,self.m._t = front_sets,t
        self.m._s,self.m._tau,self.m._w,self.m._bigM = s,tau,w,bigM

    def set_problem(self):
        self.m._y = self.m.addVars(self.m._E,vtype=GRB.BINARY,name='y') #Check for better routes
        # self.m._u = self.m.addVars(self.m._N,vtype=GRB.CONTINUOUS,name='u') #For subtour elimination
        self.m._E =list(set(self.m._E))

        self.m._E_unq = []
        for i in self.m._E:
            if (i[0] != i[1]):
                if i[1] == self.m._depot_loc:
                    continue
                else:
                    self.m._E_unq.append(i)

        self.m._a = self.m.addVars(self.m._E,vtype=GRB.CONTINUOUS,name='a') #Arrival time variable
        self.m._o = self.m.addVars(self.m._N,vtype=GRB.CONTINUOUS,name='o') #finish time variable
        self.m._q = self.m.addVars(self.m._N,vtype=GRB.BINARY,name='q') #Late arrival indicator
        # self.m._z = self.m.addVars(self.m._N,vtype=GRB.CONTINUOUS,name='y') #Total penalty variable
        self.m._v = self.m.addVars(self.m._N,vtype=GRB.BINARY,name='v') #Route node assignment variable
        self.m._g = self.m.addVars(self.m._N,vtype=GRB.CONTINUOUS,name='g') #LAteness variable
        self.m._e = self.m.addVars(self.m._N,vtype=GRB.CONTINUOUS,name='e') #Earliness variable

        #Disjunction variables
        self.m._Dis1 = self.m.addVars(self.m._N,vtype=GRB.BINARY) #DISJUNCTION FOR g being positive
        self.m._Dis2 = self.m.addVars(self.m._N,vtype=GRB.BINARY) #Disjunction for e being positive

        #Starting block of constraints
        self.m.addConstr(gp.quicksum(self.m._y[self.m._depot_loc,j] for (i,j) in self.m._E if i == self.m._depot_loc)==1,name='Depot_exit')
        self.m.addConstr(gp.quicksum(self.m._y[j,self.m._depot_loc] for (j,i) in self.m._E if i == self.m._depot_loc)==1,name='Depot_enter')
        self.m.addConstrs((gp.quicksum(self.m._y[i,j] for (ii,j) in self.m._E if ii == i) - gp.quicksum(self.m._y[j,i] for (j,ii) in self.m._E if ii == i) == 0 for i in self.m._N),name='FlowBalance')
        self.m.addConstr((gp.quicksum(self.m._y[i,j] for (i,j) in self.m._E) <= self.m._cap+1),name='RouteCapacity')

        #Scheduling constraints
        for i in self.m._N:
            self.m.addConstr(gp.quicksum(self.m._y[j,i] for j in self.m._back_sets[i]) == self.m._v[i])
            self.m.addConstr(self.m._v[i] >= self.m._q[i])


        for (i,j) in self.m._E: #A_unq will not take into consideration trips to depot 0 to achieve feasibility
            self.m.addConstr(self.m._a[i,j] == ((self.m._o[i]+self.m._t[i,j])*self.m._y[i,j]))

        # #Finishing time of inspection on node j (only for any j different that depot)
        for j in self.m._N:
            if j != self.m._depot_loc:
                self.m.addConstr(self.m._o[j] == self.m._s[j]+gp.quicksum(self.m._a[i,j] for i in self.m._back_sets[j]))
        # #Initial time for depot is 0
        self.m.addConstr(self.m._o[self.m._depot_loc]==0)

        for j in self.m._N:
            if j != self.m._depot_loc:
                self.m.addConstr(gp.quicksum(self.m._a[i,j] for i in self.m._back_sets[j]) - self.m._g[j] + self.m._e[j] == self.m._tau[j])

                #Disjunction to avoid g and e active at the same time
                self.m.addConstr(self.m._g[j] <= self.m._bigM*(1-self.m._Dis1[j]))
                self.m.addConstr(-self.m._bigM*(1-self.m._Dis1[j]) <= self.m._g[j] )
                self.m.addConstr(self.m._e[j] <= self.m._bigM*(1-self.m._Dis2[j]))
                self.m.addConstr( -self.m._bigM*(1-self.m._Dis2[j]) <= self.m._e[j])
                self.m.addConstr(self.m._Dis1[j]+self.m._Dis2[j]==1)

                self.m.addConstr(gp.quicksum(self.m._a[i,j] for i in self.m._back_sets[j]) - self.m._bigM*self.m._q[j] <= self.m._tau[j])
                self.m.addConstr(self.m._g[j] >= self.m._q[j])

        self.m.update()
        

    def set_objective(self,pi):
        self.m.setObjective((gp.quicksum((self.m._t[i,j]-pi[j])*self.m._y[i,j] for (i,j)  in self.m._E) )+(gp.quicksum((self.m._w[i]-pi[i])*self.m._q[i]*self.m._v[i] for i in self.m._N)), GRB.MINIMIZE)

    def solve_PP(self,silent=True,fast=True,time_limit=10):
        self.m._fast = fast
        if silent==True:
            self.m.setParam('OutputFlag',0)
        else:
            self.m.setParam('OutputFlag',1)
        self.m.setParam('TimeLimit', time_limit)
        self.m._n = len(self.m._N)
        self.m._vars = self.m._y
        self.m.Params.lazyConstraints = 1
        self.m.optimize(subtourelim)
    
    def get_path(self):
        better_route = {}
        for (i,j),val in self.m._y.items():
            if val.X > 0.5:
                better_route[i,j] = val.X

        #Registering new route and adding it to possible routes
        solution_arcs = list(better_route.keys())
        curr_node = 0
        path = [curr_node] #Adding new route
        while solution_arcs:
            for (i,j) in solution_arcs:
                if i == curr_node:
                    path.append(j)
                    curr_node = j
                    solution_arcs.remove((i,j))

        return path
