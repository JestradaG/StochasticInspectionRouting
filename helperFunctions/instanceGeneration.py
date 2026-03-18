from scipy import stats
import random
import numpy as np
import pandas as pd

from helperFunctions import utils

def sample_norm(mean,sd,n_samples):
    samples = stats.norm.rvs(scale=sd,loc=mean,size=n_samples)
    return samples

def generate_instance(node_num,condense_parameter_out,condense_parameter_in,initial_sample_arcs):

    N = range(node_num)
    data_dict = []
    for i in N:
        x = np.round(random.random()*200,0)
        y = np.round(random.random()*200,0)
        if i == 0:
            si = 0
            li = 0
            bi = 0
            taui = 0
            wi = 0 
            pi = 0
        else:
            si = int(random.random()*10)+1
            bi = np.round(random.random(),2)*np.round(random.random(),2)*np.round(random.random(),2)
            taui = np.round(random.random()*100,0)*int(random.random()*10)+int(random.random()*20)
            li = (1-bi)/(taui)
            wi = int((random .random()+1)*300)
            pi = int(wi*random.random()*4+1)
        data_dict.append({'x':x,'y':y,'si':si,'li':li,'bi':bi,'taui':taui,'wi':wi,'pi':pi})
    data = pd.DataFrame(data_dict)
    A_arr=np.array(data[['x','y']])

    arcs = []
    for i in N:
        for j in N:
            if i!=j:
                arcs.append((i,j))
    rnd_arcs = np.random.choice(range(len(arcs)),int(len(arcs)*initial_sample_arcs),replace=False)
    in_arcs = list(N)
    out_arcs = list(N)
    sel_arcs = []
    for i in rnd_arcs:
        a = arcs[i]
        p1 = a[0]
        if p1 in out_arcs:
            out_arcs.remove(p1)
        p2 = a[1]
        if p2 in in_arcs:
            in_arcs.remove(p2)
        sel_arcs.append((p1,p2))
    for o_a in out_arcs:
        candidates = list(N)
        candidates.remove(o_a)
        in_node = np.random.choice(candidates,1)[0]
        sel_arcs.append([o_a,in_node])
    for i_a in in_arcs:
        candidates = list(N)
        candidates.remove(i_a)
        out_node = np.random.choice(candidates,1)[0]
        sel_arcs.append([out_node,i_a])
    adj_mat = np.zeros((node_num,node_num))
    for i in N:
        adj_mat[i,i] = 1
    for a in sel_arcs:
        p1 = a[0]
        p2 = a[1]
        adj_mat[p1,p2] = 1
    adj_mat = np.linalg.matrix_power(adj_mat,node_num)
    for i in N:
        counter = 0
        for j in N:
            if adj_mat[i,j] == 0:
                if counter == 0:
                    sel_arcs.append((i,j))
                    counter += 1
    condense_mat = {}
    for n in N:
        condense_mat[n] = {}
        condense_mat[n]['out'] = 0
        condense_mat[n]['in'] = 0
    for i in sel_arcs:
        condense_mat[i[0]]['out'] += 1
        condense_mat[i[1]]['in'] += 1
    for n in N:
        if condense_mat[n]['out'] < condense_parameter_out:
            for i in range(condense_parameter_out-condense_mat[n]['out']):
                candidates = list(N)
                candidates.remove(n)
                in_node = np.random.choice(candidates,1)[0]
                sel_arcs.append([n,in_node])
                condense_mat[n]['out'] += 1
                condense_mat[in_node]['in'] += 1
        if condense_mat[n]['in'] < condense_parameter_in:
            for i in range(condense_parameter_in-condense_mat[n]['in']):
                candidates = list(N)
                candidates.remove(n)
                out_node = np.random.choice(candidates,1)[0]
                sel_arcs.append([out_node,n])
                condense_mat[out_node]['out'] += 1
                condense_mat[n]['in'] += 1
    adj_mat = np.zeros((node_num,node_num))
    for a in sel_arcs:
        p1 = a[0]
        p2 = a[1]
        x1 = data['x'].iloc[p1]
        y1 = data['y'].iloc[p1]
        x2 = data['x'].iloc[p2]
        y2 = data['y'].iloc[p2]
        adj_mat[p1][p2] = 1
        dx = x2-x1
        dy = y2-y1
        for i in range(len(data)):
            x = data['x'].loc[i]
            y = data['y'].loc[i]
    A = [] #feasible arcs
    front_sets = {} #Front sets for every node
    back_sets = {} #Back sets for every node
    for i in N:
        front_sets[i] = []
        back_sets[i] = []
    for i in N:
        for j in N:
            if np.matrix(adj_mat)[i,j] > 0:
                A.append((i,j))
                front_sets[i].append(j)
                back_sets[j].append(i)
    A = list(set(A))
    c = pd.DataFrame(utils.EDM(A_arr,A_arr))
    t = np.matrix(c)

    return data,N,A,back_sets,front_sets,t

def generate_IEEE(bus_num,depot_loc,drone_num):
    arcs = pd.read_excel(f'PowerGrid/{bus_num}_branch_data.xlsx')
    arcs['r'] = arcs['r']*100
    arcs['fbus'] = arcs['fbus'] - 1
    arcs['tbus'] = arcs['tbus'] - 1
    dir_graph = arcs[['fbus','tbus']]
    data = pd.read_excel(f'PowerGrid/{bus_num}_bus_data.xlsx')
    data['Risk'] = (data['type']+data['Pd'])*100
    w = dict(data['Risk'])
    for node in range(bus_num):
        if node == depot_loc:
            w[node] = 0
        else:
            w[node] = int(w[node])

    A = []
    for (i,j) in dir_graph.values:
        A.append((i,j))
        A.append((j,i))
    A = list(set(A))
    A_orig = A.copy()
    A_unq = []
    for i in A:
        if (i[0] != i[1]):
            if i[1] == depot_loc:
                continue
            else:
                A_unq.append(i)
    N = range(bus_num)
    front_sets = {} #Front sets for every node
    back_sets = {} #Back sets for every node
    for i in N:
        front_sets[i] = []
        back_sets[i] = []
    for (i,j) in A:
        front_sets[i].append(j)
        back_sets[j].append(i)
    d = pd.DataFrame(0, index=list(N), columns = list(N)).add(arcs.pivot_table(values='r', index="fbus", columns='tbus',fill_value=0, aggfunc=sum),fill_value=0)
    for i,j in dir_graph.values:
        d[i][j] = d[j][i]
    
    #Using shortest path computation to add cost of returning to depot (using the current arcs)
    #We do not assume the cost of new arcs, but compute the shortest path from current node to depot
    graph_data = []
    for (i,j) in dir_graph.values:
        graph_data.append({'Origin':i,'Destination':j,'Cost':d[i][j]})
        graph_data.append({'Origin':j,'Destination':i,'Cost':d[j][i]})
    graph_data = pd.DataFrame(graph_data)
    for i in N:
        if i != depot_loc:
            dijkst = utils.Dijkstra_sp(data=graph_data,start_node=i,report_all_nodes=True,end_node=depot_loc)
            d[i][depot_loc] = dijkst[depot_loc]['d']
            if d[depot_loc][i] == 0: #adding the arc from any node to the depot and back (for feasibility)
                d[depot_loc][i] = dijkst[i]['d']
                d[i][depot_loc] = dijkst[i]['d']
                A.append((depot_loc,i))
                A.append((i,depot_loc))
                front_sets[depot_loc].append(i)
                front_sets[i].append(depot_loc)
                front_sets[depot_loc] = list(set(front_sets[depot_loc]))
                front_sets[i] = list(set(front_sets[i]))
                back_sets[i].append(depot_loc)
                back_sets[i] = list(set(back_sets[i]))
                back_sets[depot_loc].append(i)
                back_sets[depot_loc] = list(set(back_sets[depot_loc]))

    A = list(set(A))
    K = range(drone_num)

    N = range(bus_num)
    s,tau = {},{}
    for i in N:
        if i == depot_loc:
            s[i] = 0
            tau[i] = 0
        else:
            s[i] = int(random.random()*10)+1
            tau[i] = np.round(random.random()*200,0)*int(random.random()*100)

    

    return N,K,A,back_sets,front_sets,np.array(d),s,tau,w

def generate_uncertainty(s,tau,deviation,num_scenarios,out_sample=1):
    #We generate uncertainty for parameters s,tau 
    #(we do not use l and b anymore, since they are only for the linear approximation of degradation)
    s_realizations = {}
    s_realizations_r = {}

    tau_realizations = {}
    tau_realizations_r = {}

    for i,mean in s.items():
        s_realizations[i] = {}
        tau_realizations[i] = {}
        for omega in range(num_scenarios):
            mean_s = s[i]*out_sample
            sd_s = mean_s*deviation
            
            s_realizations[i][omega] = np.ceil(sample_norm(mean_s,sd_s,1)[0])
            
            mean_tau = tau[i]*out_sample
            sd_tau = mean_tau*deviation

            tau_realizations[i][omega] = np.ceil(sample_norm(mean_tau,sd_tau,1)[0])
    for omega in range(num_scenarios):
        s_realizations_r[omega] = {}
        tau_realizations_r[omega] = {}

        for i,mean in s.items():
            s_realizations_r[omega][i] = s_realizations[i][omega]
            tau_realizations_r[omega][i] = tau_realizations[i][omega]

    return s_realizations,s_realizations_r,tau_realizations,tau_realizations_r