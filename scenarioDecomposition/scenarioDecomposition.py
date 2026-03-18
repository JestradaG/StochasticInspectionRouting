import time
import numpy as np

from scenarioDecomposition import Column_enum
from helperFunctions import objectiveComputation,utils
import math
import concurrent.futures


def solve(num_random_routes,num_scenarios,max_iters,internal_iters,N,A,front_sets,back_sets,t,drone_num,depot_loc,node_num,cap,s,s_realizations_r,tau_realizations_r,w,bigM):
    # Early stopping and adaptive parameters
    
    UpperBound = np.inf
    LowerBound = -np.inf
    S_x = {} #set of explored solutions for variable x
    x_star = {} #best feasible solution x as of now
    anticipatory_penalties = {}
    for i in range(num_scenarios):
        if i == 0:
            anticipatory_penalties[i] = -(num_scenarios-1)
        else:
            anticipatory_penalties[i] = 1
    cuts_x_1 = {}
    cuts_x_0 = {}
    master_iter = 0
    tolerance = 0.05
    init = time.time()
    path_pool = []
    max_time = 15#60 + 2 * node_num + 5 * num_scenarios  # seconds, adaptive to instance size
    max_master_iters = 5#min(20, max(5, node_num // 2))
    max_inner_iters = 5#min(50, max(10, node_num * 2))
    max_no_improve = 5#max(5, node_num // 2)
    no_improve_count = 0
    best_obj = np.inf
    scenario_batch = min(num_scenarios, 2)
    scenario_increment = max(1, num_scenarios // 5)
    route_gen_limit = 5#min(100, math.factorial(node_num) // (drone_num+1))
    print(f"[DEBUG] max_time={max_time}, max_master_iters={max_master_iters}, max_inner_iters={max_inner_iters}, max_no_improve={max_no_improve}, route_gen_limit={route_gen_limit}")
    while (master_iter < max_master_iters):
        print(f"[DEBUG] Master iteration {master_iter}, UpperBound={UpperBound}, LowerBound={LowerBound}")
        if (time.time() - init) > max_time:
            print(f"[DEBUG] Early stopping: max time {max_time}s reached after {master_iter} master iterations.")
            break
        v_i = {}
        x_i = {}
        included_cuts = list(cuts_x_1.keys())
        scenarios_to_solve = min(num_scenarios, scenario_batch)

        def solve_scenario(scenario):
            print(f"[DEBUG]  Solving scenario {scenario} (batch size {scenarios_to_solve})")
            enumerated_exp = Column_enum.column_enumeration(
                min(num_random_routes, route_gen_limit),
                min(max_inner_iters, max_iters),
                min(max_inner_iters, internal_iters),
                N, range(drone_num), A, front_sets, back_sets, t, drone_num, depot_loc, node_num, cap,
                s_realizations_r[scenario], tau_realizations_r[scenario], w, bigM[scenario], S_x)
            solutions = {}
            solutions['x'] = enumerated_exp.parse_solution()
            solutions['v'] = enumerated_exp.m.ObjVal/num_scenarios
            return (scenario, solutions)

        with concurrent.futures.ThreadPoolExecutor() as executor:
            future_to_scenario = {executor.submit(solve_scenario, scenario): scenario for scenario in range(scenarios_to_solve)}
            for future in concurrent.futures.as_completed(future_to_scenario):
                scenario, solutions = future.result()
                x_i[master_iter, scenario] = (solutions['x'], utils.extract_routes(solutions['x']))
                v_i[scenario] = solutions['v']
        print(f"[DEBUG]  Number of candidate solutions in x_i: {len(x_i)}")
        LowerBound = sum(v_i.values())
        S_x = S_x|x_i
        print(f"[DEBUG]  Total number of solutions in S_x: {len(S_x)}")
        print(f"[DEBUG]  LowerBound (indicator): {LowerBound}")

        # Upperbounding
        improved = False
        found_feasible = False
        for _,(x,route) in S_x.items():
            u = objectiveComputation.expected_objective(False,x,drone_num,num_scenarios,s_realizations_r,tau_realizations_r,t,s,w,depot_loc,N)
            if UpperBound > u:
                UpperBound = u
                x_star = x
                improved = True
                found_feasible = True
        if not found_feasible:
            print("[DEBUG]  No feasible solution found in this iteration.")
        if improved:
            no_improve_count = 0
            best_obj = UpperBound
        else:
            no_improve_count += 1
        # If no improvement for several iterations, stop
        if no_improve_count >= max_no_improve:
            print(f"[DEBUG] Early stopping: no improvement in {max_no_improve} iterations after {master_iter} master iterations.")
            break
        # If time allows and improvement, increase scenario batch
        if improved and scenarios_to_solve < num_scenarios and (time.time() - init) < max_time * 0.8:
            scenario_batch = min(num_scenarios, scenario_batch + scenario_increment)
        end = time.time()
        master_iter += 1

    print(f"[DEBUG] Finished after {master_iter} master iterations. Final UpperBound: {UpperBound}, LowerBound (indicator): {LowerBound}")
    if not x_star:
        print("[DEBUG] No feasible solution returned by scenario decomposition.")
    return x_star