import numpy as np
import gurobipy as gp
from gurobipy import GRB
from scenarioDecomposition.random_coloring import random_color
from scenarioDecomposition.RMP import InitRMP
from scenarioDecomposition.IP_PP import Init_IP_PP
from helperFunctions import objectiveComputation

def CG_RC(num_random_routes,max_iter,internal_iters,N,K,E,front_sets,back_sets,t,drone_num,depot_loc,node_num,cap,s,tau,w,bigM,forbidden_arc_sets=None):
    #Setting RMP
    rmp = InitRMP()
    rmp.load_data(N,K,E,front_sets,t,s,tau,w,drone_num,depot_loc,node_num,cap)
    rmp.set_RMP(relaxed=True) #initializing a relaxed restricted Master problem
    if rmp.m._infeasible == 1:
        print('Warning: INFEASIBILITY suspected by feasible solution heuristic!!')
        return rmp
    rmp.generate_seed_routes(num_random_routes)

    rmp.solve_RMP(time_limit=10)

    ## Setting safe-vault Pricing problem
    PP = Init_IP_PP()
    PP.load_data(depot_loc,cap,N,K,E,front_sets,back_sets,t,s,tau,w,bigM)
    PP.set_problem()
    if forbidden_arc_sets is None:
        forbidden_arc_sets = set()
    #print('setting up IP PP safevault')

    iters = 0
    while iters < max_iter:
        #Gathering the duals
        desired_routes = 1
        pi = rmp.get_duals()

        T,P = random_color(max_iter,internal_iters,desired_routes,cap,N,depot_loc,front_sets,E,t,pi,s,tau,w,strong_rules=False,target=0,forbidden_arc_sets=forbidden_arc_sets)
        # if len(P) == 0: #Last try
        #     T,P = random_color(max_iter,internal_iters,desired_routes,cap,N,depot_loc,front_sets,E,t,pi,s,tau,l,b,w,alpha,strong_rules=False,target=0)
        if len(P) > 0:
            #print('WE found a random colored column!')
            for ((n,V),C),candidate_path in zip(T,P):
                rmp.add_column(C,candidate_path,'RC_route')
                #print('Adding column')
                iters += 1
                #print(iter)
            rmp.solve_RMP(time_limit=10)

        else : #Safevault using IP PP
            #print('We employ the IP safevault!!')
            # Add no-good cuts for forbidden arc sets
            for forbidden_arcs in forbidden_arc_sets:
                PP.m.addConstr(gp.quicksum(1-PP.m._y[i,j] for (i,j) in forbidden_arcs if (i,j) in PP.m._y) >= 1, name=f'nogood_{hash(forbidden_arcs)%1000000}')
            PP.set_objective(pi)
            PP.solve_PP(fast=True)
            if PP.m.ObjVal < -0.001:
                #print(f'We found an IP generated column')
                T = []
                P = []
                path = PP.get_path()
                P.append(path)
                #print(f'IP found reduced cost of {cost}')
                pi = {}
                cost = objectiveComputation.path_cost(path,t,s,pi,tau,w,depot_loc,N)
                #print(f'adding IP column with cost {cost}')
                #print(path)
                T.append(((None,None),cost))
                for ((n,V),C),candidate_path in zip(T,P):
                    rmp.add_column(C,candidate_path,'IP_route')
                    #print('Adding column')
                    iters += 1
                    #print(iter)
                rmp.solve_RMP(time_limit=10)
            else:
                break
    return rmp