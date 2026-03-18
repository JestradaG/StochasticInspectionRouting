import random
import numpy as np
from helperFunctions import objectiveComputation
import concurrent.futures

def del_list_indexes(l, id_to_del):
    somelist = [i for j, i in enumerate(l) if j not in set(id_to_del)]
    return somelist


def random_color(iter_max,internal_iters,desired_routes,cap,N,depot_loc,front_sets,E,t,pi,s,tau,w,strong_rules=False,target=0,forbidden_arc_sets=None, parallel=False):
    #print('Usando este script')
    T = []
    P = []
    if forbidden_arc_sets is None:
        forbidden_arc_sets = set()
    master_iterations = 0
    
    if forbidden_arc_sets is None:
        forbidden_arc_sets = set()
    def single_coloring_run(_):
        T = []
        P = []
        #Color coding function
        coloring = {}
        available_nodes = list(range(1, len(N)))
        random.shuffle(available_nodes)
        color_ids = list(range(cap+1)) * ((len(available_nodes) // (cap+1)) + 1)
        random.shuffle(color_ids)
        for idx, node in enumerate(available_nodes):
            coloring[node] = color_ids[idx % len(color_ids)]
        for i in N:
            if i not in coloring.keys():
                coloring[i] = np.random.randint(0, cap)
        LAMBDA = {}
        PI = {}
        LAMBDA[depot_loc] = [[[0,np.zeros(cap+1)],0]]
        PI[depot_loc] = [[0]]
        for i in range(1,len(N)):
            LAMBDA[i] = []
            PI[i] = []
        S = [depot_loc]
        initial_count = 1
        iteration = 0
        found_new_route = False
        while S:
            i = S[0]
            if (i == depot_loc)&(initial_count == 0):
                for ((n,V),C),valid_path in zip(LAMBDA[i],PI[i]):
                    if n == cap+1:
                        if valid_path not in P:
                            arcs = set((valid_path[k], valid_path[k+1]) for k in range(len(valid_path)-1))
                            if any(arcs == forbidden for forbidden in forbidden_arc_sets):
                                continue
                            if C < target:
                                pi_f = {}
                                real_cost = objectiveComputation.path_cost(valid_path,t,s,pi_f,tau,w,depot_loc,N)
                                T.append([[n,V],real_cost])
                                P.append(valid_path)
                                found_new_route = True
            else:
                for j in front_sets[i]:
                    if (i,j) in E:
                        for ((n,V),C),current_path in zip(LAMBDA[i].copy(),PI[i].copy()):
                            if V[coloring[j]] == 0:
                                lam = []
                                n_ext = n+1
                                V_ext = V.copy()
                                V_ext[coloring[j]] = 1
                                lam.append([n_ext,V_ext])
                                comparison_path = current_path.copy()
                                comparison_path.append(j)
                                C_ext = objectiveComputation.path_cost(comparison_path,t,s,pi,tau,w,depot_loc,N)
                                lam.append(C_ext)
                                equal = 0
                                dominated = 0
                                dominated_paths = []
                                if len(LAMBDA[j]) > 0:
                                    iter_ = 0
                                    while (dominated == 0)&(iter_==0):
                                        for idx,((n_j,V_j),C_j) in enumerate(LAMBDA[j]):
                                            comparison_path = current_path.copy()
                                            comparison_path.append(j)
                                            if comparison_path == PI[j][idx]:
                                                dominated += 1
                                                break
                                            if strong_rules == False:
                                                if C_j <= C_ext:
                                                    dominated += 1
                                                    break
                                                if n_j <= n_ext:
                                                    dominated += 1
                                                    break
                                                for v,v_ext in zip(V,V_ext):
                                                    if v <= v_ext:
                                                        dominated += 1
                                                        break
                                            elif strong_rules == True:
                                                if C_j <= C_ext:
                                                    dominated += 1
                                                    break
                                                if n_j == n_ext:
                                                    dominated += 1
                                                    break
                                                for v,v_ext in zip(V,V_ext):
                                                    if v == v_ext:
                                                        dominated += 1
                                                        break
                                            if dominated > 0:
                                                break
                                            else:
                                                dominated_paths.append(idx)
                                        iter_ += 1
                                if dominated == 0:
                                    del_list_indexes(LAMBDA[j],dominated_paths)
                                    del_list_indexes(PI[j],dominated_paths)
                                    candidate_path = current_path.copy()
                                    candidate_path.append(j)
                                    PI[j].append(candidate_path)
                                    LAMBDA[j].append(lam)
                                    S.append(j)
                if initial_count == 1:
                    initial_count -= 1
                    LAMBDA[depot_loc].pop(0)
                    PI[depot_loc].pop(0)
            S.remove(i)
            S = list(set(S))
            if strong_rules == False:
                if iteration >= internal_iters:
                    break
            iteration += 1
        return T, P, found_new_route

    T_all = []
    P_all = []
    found_any = False
    max_failed_master = 10
    failed_master = 0
    master_iterations = 0
    max_total_attempts = 10
    total_attempts = 0
    if parallel:
        with concurrent.futures.ThreadPoolExecutor() as executor:
            while True:
                futures = [executor.submit(single_coloring_run, _) for _ in range(iter_max)]
                for future in concurrent.futures.as_completed(futures):
                    T, P, found_new_route = future.result()
                    T_all.extend(T)
                    P_all.extend(P)
                    if found_new_route:
                        found_any = True
                total_attempts += iter_max
                master_iterations += 1
                if not found_any:
                    failed_master += 1
                else:
                    failed_master = 0
                if failed_master >= max_failed_master:
                    break
                if total_attempts >= max_total_attempts:
                    break
                if strong_rules == False:
                    if len(T_all) >= desired_routes:
                        break
    else:
        while True:
            for _ in range(iter_max):
                T, P, found_new_route = single_coloring_run(_)
                T_all.extend(T)
                P_all.extend(P)
                if found_new_route:
                    found_any = True
            total_attempts += iter_max
            master_iterations += 1
            if not found_any:
                failed_master += 1
            else:
                failed_master = 0
            if failed_master >= max_failed_master:
                break
            if total_attempts >= max_total_attempts:
                break
            if strong_rules == False:
                if len(T_all) >= desired_routes:
                    break
    return T_all, P_all