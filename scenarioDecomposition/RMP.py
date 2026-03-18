import numpy as np
import pandas as pd
import gurobipy as gp
from gurobipy import GRB
from scenarioDecomposition.tabu_search import get_best_solution
from helperFunctions import objectiveComputation
from scenarioDecomposition.initial_feasible_solution import initial_feasible,infeasibility_check

#Master Problem
class InitRMP:
    def __init__(self,id=''):
        self.m = gp.Model(f'RestrictedMasterProblem{id}')

    def load_data(self,N,K,E,front_sets,t,s,tau,w,drone_num,depot_loc,node_num,cap): #initializing the data into the gurobi object for later use
        self.m._N,self.m._K,self.m._E,self.m._front_sets = N,K,E,front_sets #Loading sets
        self.m._t,self.m._drone_num,self.m._depot_loc = t,drone_num,depot_loc
        self.m._node_num,self.m._cap = node_num,cap
        self.m._s,self.m._tau,self.m._w = s,tau,w

    def generate_seed_routes(self,num_random_routes):
        # Generate greedy, random, and reversed routes for diversity
        N = self.m._N
        K = self.m._K
        depot = self.m._depot_loc
        for i in range(num_random_routes):
            # Greedy: sort by distance from depot
            if i % 3 == 0:
                arr = np.array(sorted(range(1, len(N)), key=lambda x: self.m._t[depot, x]))
            # Reversed greedy
            elif i % 3 == 1:
                arr = np.array(sorted(range(1, len(N)), key=lambda x: -self.m._t[depot, x]))
            # Random
            else:
                arr = np.array(range(1, len(N)))
                np.random.shuffle(arr)
            newarr = np.array_split(arr, self.m._drone_num)
            for k in K:
                route = list(newarr[k])
                route = [depot] + route + [depot]
                pi = {}
                cost = objectiveComputation.path_cost(route, self.m._t, self.m._s, pi, self.m._tau, self.m._w, depot, N)
                self.add_column(cost, route)
        
    def set_RMP(self,relaxed=True):
        if relaxed == True:
            self.m._var_type = GRB.CONTINUOUS
        else:
            self.m._var_type = GRB.BINARY
        #Setting elementary routes (one for each drone) (we just assign greedily to use all capacity for each drone)
        remaining_nodes = list(self.m._N[1:])
        self.m._A = np.zeros((0,0))
        self.m._r = {} #routes variables
        self.m._r_routes = {} #dictionary with specific route
        self.m._r_costs = {}
        self.m._solution_counter = 0

        initial_solutions = initial_feasible(self.m._N,self.m._front_sets,self.m._E,self.m._depot_loc,self.m._cap,self.m._drone_num)
        self.m._infeasible = infeasibility_check(initial_solutions,self.m._N,self.m._depot_loc)
        if self.m._infeasible == 1:
            return
        else:
            pi = {}
            for idx,route in initial_solutions.items():
                cost = objectiveComputation.path_cost(route,self.m._t,self.m._s,pi,self.m._tau,self.m._w,self.m._depot_loc,self.m._N)
                self.add_column(cost,route)

        self.m.update()

        #Tabu-searched columns
        self.m._best_solution = get_best_solution(self.m._r_routes,self.m._t,self.m._s,self.m._tau,self.m._w,self.m._N,self.m._cap,50,self.m._depot_loc,self.m._E)
        for path in self.m._best_solution.values():
            pi = {}
            C = objectiveComputation.path_cost(path,self.m._t,self.m._s,pi,self.m._tau,self.m._w,self.m._depot_loc,self.m._N)
            self.add_column(C,path,'Tabu_route')
        #Newly added columns based on the insights about criticality and first routes
        #Identifying critical nodes
        
        
        elim_index = []
        left_nodes = list(range(len(self.m._tau)))
        paths = {}
        for i in self.m._K:
            if len(left_nodes) > 0: #We continue
                paths[i] = [0]
                left_nodes.remove(0)
                elim_index.append(0)
                cap = self.m._cap
                complete = False
                while cap > 0:
                    if len(left_nodes) > 0:
                        dists =  np.squeeze(np.asarray(self.m._t[paths[i][-1]]))
                        dists = np.array(dists)[left_nodes]
                        ta = np.array(list(self.m._tau.values()))[left_nodes]
                        d_t = pd.DataFrame({'D':dists,'Tau':ta},index=left_nodes)
                        d_t['D'] = -d_t['D']
                        d_t.sort_values(by=['Tau','D'],inplace=True)
                        if d_t.index[0] == 0: #We evaluate if we continue or not
                            if (len(d_t) == 1) | (cap < self.m._cap/3):
                                paths[i].append(d_t.index[0])
                                elim_index.append(d_t.index[0])
                                cap = 0
                                complete = True
                            else: #We should still continue
                                # print(d_t)
                                paths[i].append(d_t.index[1])
                                elim_index.append(d_t.index[1])
                                
                        else:
                            paths[i].append(d_t.index[0])
                            elim_index.append(d_t.index[0])
                        l_n = []
                        for l in left_nodes:
                            if l not in elim_index:
                                l_n.append(l)
                        left_nodes = l_n
                        cap -= 1
                    else:
                        complete = True
                        paths[i].append(0)
                        cap = 0
                if complete == False:
                    paths[i].append(0)
                elim_index.remove(0)
                left_nodes.append(0)
        for path_id,path in paths.items():
            pi = {}
            C = objectiveComputation.path_cost(path,self.m._t,self.m._s,pi,self.m._tau,self.m._w,self.m._depot_loc,self.m._N)
            self.add_column(C,path,f'Insight_route_{path_id}')


    def add_column(self,cost,path,ctype='found_route'):
        #Registering new route and adding it to possible routes
        self.m._r_costs[self.m._solution_counter] = cost #updating costs
        self.m._r_routes[self.m._solution_counter] = path

        #Updating columns
        visited_nodes = list(filter(lambda a: a != 0, self.m._r_routes[self.m._solution_counter])) #Removing depot nodes in path
        new_column = np.zeros(len(self.m._N)-1) #all but depot
        #Adding the visiting nodes
        for i in range(len(self.m._N)-1): #positions enough for matrix A
            if i+1 in visited_nodes: #Counting for the removal of 0 ###If we deal with names instead of integers, will require changes
                new_column[i] = 1
        #adding the number of drones constraint
        #new_column[len(N)-1] = 1
        if len(self.m._A) == 0: #Newborn A, we have to fill it
            self.m._A = np.matrix(new_column).T
            self.m._r[self.m._solution_counter] = self.m.addVar(vtype=self.m._var_type,name=f'initial_route_{self.m._solution_counter}')
            for i in range(0,len(self.m._N)-1):
                self.m.addConstr(gp.LinExpr(self.m._A[i,:],[self.m._r[self.m._solution_counter]]) == 1,name=f'Visit_node_{i+1}') #Node visiting constraint
            
            self.m.setObjective(gp.LinExpr([self.m._r_costs[self.m._solution_counter]],[self.m._r[self.m._solution_counter]]),GRB.MINIMIZE)
        else:
            self.m._A = np.c_[ self.m._A, new_column ] #
            allconstraints=self.m.getConstrs()
            new_col = gp.Column(new_column,allconstraints)
            self.m._r[self.m._solution_counter] = self.m.addVar(obj=self.m._r_costs[self.m._solution_counter],vtype=self.m._var_type,column=new_col,name=f'{ctype}_{self.m._solution_counter}')
        self.m._solution_counter += 1
        self.m.update()
    
    def parse_solution(self):
        self.m._RMP_sols = {}
        drone_counter = 0
        #Parsing solutions
        for i,val in self.m._r.items():
            if val.X > 0.5:
                for pos,orig in enumerate(self.m._r_routes[i][:-1]):
                    dest = self.m._r_routes[i][pos+1]
                    self.m._RMP_sols[(orig,dest,drone_counter)] = 1
            drone_counter += 1
        
        return self.m._RMP_sols
    
    def get_paths(self):
        self.m._selected_paths = {}
        for idx,val in self.m._r.items():
            if val.X > 0.5:
                self.m._selected_paths[idx] = self.m._r_routes[idx]

        return self.m._selected_paths

    def change_var_type(self,vtype='int'):
        if vtype == 'int':
            self.m._var_type = GRB.BINARY #Change all new inserted variables
            for var in self.m.getVars():#change all existing vars
                var.vtype=GRB.BINARY
        if vtype == 'cont':
            self.m._var_type = GRB.CONTINUOUS # change future variables
            for var in self.m.getVars(): #change all existing vars
                var.vtype=GRB.CONTINUOUS

    def solve_RMP(self,silent=True, time_limit=10):
        if silent == True:
            self.m.setParam('OutputFlag', 0)
        else:
            self.m.setParam('OutputFlag', 1)
        self.m.setParam('TimeLimit', time_limit)
        self.m.optimize()

    def get_duals(self):
        #Gathering the duals
        pi = {}
        for i in range(0,len(self.m._N)):
            if i == self.m._depot_loc:
                pi[i] = 0
            else:
                pi[i] = self.m.getConstrByName(f'Visit_node_{i}').Pi
        return pi