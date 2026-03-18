from scenarioDecomposition.RMP import InitRMP
from scenarioDecomposition.random_coloring import random_color
from scenarioDecomposition.CG_RC import CG_RC
from scenarioDecomposition.IP_PP import Init_IP_PP
from helperFunctions import objectiveComputation
import gurobipy as gp
import numpy as np

def column_enumeration(num_random_routes,max_iter,internal_iters,N,K,E,front_sets,back_sets,t,drone_num,depot_loc,node_num,cap,s,tau,w,bigM,silent=True,S_x={}):
    # Build set of forbidden arc sets from S_x (full arc set for each solution)
    forbidden_arc_sets = set()
    for _, (x_sol, route_list) in S_x.items():
        arcs = set()
        for route in route_list:
            for i in range(len(route)-1):
                arcs.add((route[i], route[i+1]))
        if arcs:
            forbidden_arc_sets.add(frozenset(arcs))

    RMP = CG_RC(num_random_routes,max_iter,internal_iters,N,K,E,front_sets,back_sets,t,drone_num,depot_loc,node_num,cap,s,tau,w,bigM,forbidden_arc_sets=forbidden_arc_sets)
    RMP.solve_RMP(True, time_limit=10)
    LB = RMP.m.ObjVal
    pi = RMP.get_duals() #getting duals for setp three
    if silent == False:
        print('Initializing step 1: Solving root node LP Relaxation with column generation')
        print(f'LB:{LB}')
        print('--------------------------')
    #Step two: Solve RMP to get
    
    RMP.change_var_type('int') #Changing type to get upper bound
    RMP.solve_RMP(silent=True, time_limit=10)
    UB1 = RMP.m.ObjVal
    if silent == False:
        print('Initializing step 2: Solving integer relaxed RMP to get Upperbound')
        print(f'UB1:{UB1}')
        print('--------------------------')
    gap = ((UB1-LB)/UB1)*100
    #Step three: Set target gap and enumerate any missing solutions with strenghten random_coloring
    target = UB1-LB
    if target > 0.001:
        T,P = random_color(max_iter,internal_iters,1,cap,N,depot_loc,front_sets,E,t,pi,s,tau,w,strong_rules=True,target=target,forbidden_arc_sets=forbidden_arc_sets)
        cont = True
        if len(P) > 0:
            for ((n,V),C),candidate_path in zip(T,P):
                RMP.add_column(C,candidate_path)
                #print(iter)
            RMP.solve_RMP()
            #print('We can improve!')
            UB2 = RMP.m.ObjVal
            if UB2 >= UB1:
                #print('Random coloring failed to find improving columns')
                cont = True
            else:
                cont = False

        if cont == True: #last try with IP enumeration
            ## Setting safe-vault Pricing problem
            PP = Init_IP_PP()
            PP.load_data(depot_loc,cap,N,K,E,front_sets,back_sets,t,s,tau,w,bigM)
            PP.set_problem()
            # Add no-good cuts for forbidden arc sets
            for forbidden_arcs in forbidden_arc_sets:
                PP.m.addConstr(gp.quicksum(1-PP.m._y[i,j] for (i,j) in forbidden_arcs if (i,j) in PP.m._y) >= 1, name=f'nogood_{hash(forbidden_arcs)%1000000}')
            P = []
            C = []
            finished = False
            while finished == False:
                PP.set_objective(pi)
                PP.solve_PP(fast=False)
                
                if PP.m.ObjVal < target:
                    path = PP.get_path()
                    P.append(path)
                    cost = objectiveComputation.path_cost(path,t,s,{},tau,w,depot_loc,N)
                    C.append(cost)
                    #Building no-good cuts
                    cuts_y = []
                    cuts_y_r = []
                    for (i,j),val in PP.m._y.items():
                        if val.X > 0.5:
                            cuts_y.append((i,j))
                            cuts_y_r.append((j,i))
                    #Adding no-good cuts
                    PP.m.addConstr(((gp.quicksum(1-PP.m._y[i_j] for i_j in cuts_y) >= 1)),name=f'y_IntegerCuts')
                    PP.m.addConstr(((gp.quicksum(1-PP.m._y[j_i] for j_i in cuts_y_r) >= 1)),name=f'y_IntegerCuts')
                    #     subproblem.m.addConstr(((gp.quicksum(1-subproblem.m._g[i_k] for i_k in subproblem.m_cuts_g_1[i])+gp.quicksum(subproblem.m._g[i_k] for i_k in subproblem.m_cuts_g_0[i]) >= 1)),name=f'g_IntegerCuts_{i}')
                    PP.m.update()
                    PP.solve_PP(fast=False)
                    #print(PP.m.ObjVal)
                else:
                    finished = True
            for cost,path in zip(C,P):
                RMP.add_column(cost,path,'enumerated_route')
            RMP.solve_RMP(True, time_limit=10)

    n_gap = ((RMP.m.ObjVal-LB)/RMP.m.ObjVal)*100
    if silent == False:
        print(f'Initializing step 3: Obtaining any additional columns based on our gap {gap}%')
        print('--------------------------')
        print(f'Finished algorithm with optimal object {RMP.m.ObjVal} and a final gap {n_gap}%')
    
    return RMP