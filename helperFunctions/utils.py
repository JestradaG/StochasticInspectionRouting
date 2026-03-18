import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict

#Shortest path computation of returning path to depot (only distance between arcs in graph)
def Dijkstra_sp(data:pd.DataFrame,start_node,report_all_nodes:bool,end_node=''):

    nodes = list(pd.concat((data['Origin'],data['Destination']),ignore_index=1).unique())
    S = {} #S set
    S_bar = {} #S bar set

    #Setting positive infinite distances for all nodes
    for i in nodes:
        S_bar[i] = {'d':np.inf,'pred':np.nan} 

    #Defining goals
    start_node, end_node = start_node,end_node

    #Setting up incumbent solution
    S_bar[start_node]['d'] = 0
    S_bar[start_node]['pred'] = np.nan
    while len(S) < len(nodes):
        #print('______________Iteration begins_________________')
        #Selecting lowest distance node to continue algorithm
        min_d = min([S_bar[i]['d'] for i in S_bar.keys()])
        min_k = [key for key in S_bar.keys() if S_bar[key]['d'] == min_d][0]
        #Adding node to Set S and removing in from S_bar
        S[min_k] = S_bar[min_k]
        S_bar.pop(min_k)
        sink_nodes = list(data[data['Origin']==min_k]['Destination'])
        for j in sink_nodes:
            if j not in S.keys():
                #calculating cost
                ckj = data[(data['Origin']==min_k)&(data['Destination']==j)]['Cost'].iloc[0]
                if S_bar[j]['d'] > S[min_k]['d'] + ckj:
                    S_bar[j]['d'] = S[min_k]['d'] + ckj
                    S_bar[j]['pred'] = min_k

                    #print(f'Setting new best distance for {j} = {S_bar[j]["d"]} and predecessor {S_bar[j]["pred"]}  ')
    if report_all_nodes:
        return S
    elif not report_all_nodes:
        print('--------Printing optimal route----------')
        opt_path = []
        curr_node = end_node
        opt_path.append(curr_node)
        while curr_node != start_node:
            curr_node = S[curr_node]['pred']
            opt_path.append(curr_node)
        opt_path.reverse()
        for i in range(len(opt_path)-1):
            print(f'From {opt_path[i]} -> {opt_path[i+1]}')
        print(f'Optimal cost of {S[end_node]["d"]}')
        return 
    
#Euclidean distance matrix
def EDM(A,B): 
    p1 = np.sum(A**2, axis =1)[:,np.newaxis]
    p2 = np.sum(B**2, axis=1)
    p3 = -2 * np.dot(A,B.T)
    return np.round(np.sqrt(p1+p2+p3),2)

def extract_routes(solution):
    # Group arcs by vehicle
    vehicle_arcs = defaultdict(list)
    for (i, j, k), v in solution.items():
        if v == 1:
            vehicle_arcs[k].append((i, j))
    routes = []
    for arcs in vehicle_arcs.values():
        # Build route starting from depot (0)
        route = [0]
        current = 0
        arc_dict = {i: j for i, j in arcs}
        while True:
            next_node = arc_dict.get(current)
            if next_node is None or next_node == 0:
                break
            route.append(next_node)
            current = next_node
        route.append(0)  # End at depot
        routes.append(route)
    return routes

def plot_solution(data,x_sols):
    data.plot.scatter(x='x',y='y')
    colors = ['blue','red','green','black','orange','brown']
    color_map = {}
    drone_counter = 0
    for (i,j,k),active in x_sols.items():
        if active > 0.5:
            if k not in color_map.keys():
                 color_map[k] = drone_counter
                 drone_counter+=1
            x1 = data['x'].iloc[i]
            y1 = data['y'].iloc[i]
            x2 = data['x'].iloc[j]
            y2 = data['y'].iloc[j]
            dx = x2-x1
            dy = y2-y1
            plt.arrow(x1, y1, dx , dy,alpha=0.9,head_width =2,length_includes_head=True,color=colors[color_map[k]])
    for i in range(len(data)):
            x = data['x'].loc[i]
            y = data['y'].loc[i]
            plt.text(x-0.5 , y , i, fontsize=15)