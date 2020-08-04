import matplotlib.pyplot as plt
import networkx as nx
import math
import rospkg
import rospy
import sys
import tf
import numpy as np
import sys
import yaml 
import csv

def import_yaml():
    rospack = rospkg.RosPack()
    with open(rospack.get_path('rotors_gazebo')+"/goals/goals.yaml", 'r') as stream:
        try:
            goal_locations=yaml.safe_load(stream)
            #print(bay_lib)
        except yaml.YAMLError as exc:
            print(exc)
            print("Check Global_planner.yaml in config folder")
        return goal_locations
      
def import_file(file_name):
        file =[]
        with open(file_name) as tsvfile:
          reader = csv.reader(tsvfile, delimiter=',')
          for row in reader:
              file.append(row)
        # delete first 4 lines for data and last line
        del file[0:4]
        # Convert list to matrix
        return file
rospack = rospkg.RosPack()

inputnodes=import_file(rospack.get_path('rotors_gazebo')+"/goals/user_goals.csv")
print(inputnodes)

for i in range(len(inputnodes)):
      inputnodes[i]=int(inputnodes[i][0])
print(inputnodes)
nodelist=[]

value={}
nodes=[]
goal_locations=import_yaml()

for i in range(len(inputnodes)):
  nodes.append(goal_locations['drone_goals']['G'+str(inputnodes[i])])                  
print(nodes)
# nodes=[[0,0],[4,6],[7,0],[-3,-7],[-8,0],[-4,10],[6,13],[12,6],[8,-6],[-12,-10],[-14,-2],
#      [-12,6],[16,14],[20,6],[14,-8],[0,-17],[-10,-18],[-20,-6],[-25,5],[-20,15],
#      [-10,20],[10,20],[20,25],[25,20],[30,10],[35,5],[18,2],[24,-4],
#      [40,-10],[18,-12],[35,-20],[12,-16],[25,-22],[20,-30],[5,-30],
#      [-10,-30],[-23,-25],[-25,-15],[-35,-30],[-45,-20],[-35,-10],[-40,-10],[-35,25],[-22,21]]




for i in range(len(nodes)):
  value[i+1]=nodes[i]
#print(value)
for i in inputnodes:
  nodelist.append(value[i])
N=len(nodelist)
#print(N)
def get_key(val,value=value): 
    for key, value in value.items(): 
         if val == value: 
             return key

G = nx.Graph()

for i in range(len(nodelist)):
    for j in range(len(nodelist)):
        if i !=j:
          G.add_edge(get_key(nodelist[i]), get_key(nodelist[j]), weight=round(math.sqrt((nodelist[i][0]-nodelist[j][0])**2+(nodelist[i][1]-nodelist[j][1])**2),2))

elarge = [(u, v) for (u, v, d) in G.edges(data=True)]
elarge1 = [[u, v,d['weight']] for (u, v, d) in G.edges(data=True)]
final_graph=(sorted(elarge1,key=lambda x: x[-1]))

x1,x2 =final_graph[0][0],final_graph[0][1]
new_list=[]
new_list.append(x1)
new_list.append(x2)
g={}
g.update( {x1 : set([x2])} )
K = nx.Graph()
K.add_edge(x1, x2,weight=1)
for  i in range(len(nodelist)-2):
  s=[[u, v,d['weight']] for (u, v, d) in G.edges(data=True) if u in new_list and (v not in new_list) or v in new_list and (u not in new_list)]
  s=(sorted(s,key=lambda x: x[-1]))
  if s[0][1]in new_list:
    new_list.append(s[0][0])
  else:
    new_list.append(s[0][1])
  K.add_edge(s[0][0], s[0][1],weight=1)
new_graph = [(u, v) for (u, v, d) in K.edges(data=True)]
#print(s)

def ifnochild(node,visited,stack,m):
  k=[]
  for i in elarge1:
    if i[0]==node and i[1] not in visited:
      k.append([i,i[1]])
    if i[1]==node and i[0]not in visited:
      k.append([i,i[0]])
  if k !=[]:
    k=sorted(k,key=lambda x: x[0][-1])
    if k[0][-1] in stack:
      stack.pop(stack.index(k[0][-1]))
    visited.add(k[0][-1])
    m.append(k[0][-1])
    if (g[k[0][-1]]-visited):
         stack.extend(g[k[0][-1]]-visited)
    else:
         ifnochild(k[0][-1],visited,stack,m)

def totalcost(f_o):
  i=0
  res=0
  while i+1<len(f_o):
    for j in elarge1:
      
      if( f_o[i]==j[0] and f_o[i+1]==j[1] )or (f_o[i]==j[1] and f_o[i+1]==j[0] ):
        res+=j[-1]
        break
    i+=1
  return res

for u,v in new_graph:
  if u not in g:
    g.update( {u : set([v])} )
  else:
    g[u].add(v)
for u,v in new_graph:
  if v not in g:
    g.update( {v: set([u])} )
  else:
    g[v].add(u)
    
#print(g)
def dfs(graph, start):
    m=[]
    visited, stack = set(), [start]
    while stack:
        vertex = stack.pop()
        if vertex not in m:
          m.append(vertex)
        #print(stack)
        if vertex in graph and graph[vertex]-visited:
          if vertex not in visited:
              visited.add(vertex)
              stack.extend(graph[vertex] - visited)
        else:
          visited.add(vertex)
          ifnochild(vertex,visited,stack,m)
    
    return m

m=dfs(g,1)
cost=totalcost(m)
costk=cost/4

i=0
inbet=[]
final=[]
res=0
l=0
#inbet.append(m[i])
while i+1<len(m):
  #print(m[i])
  for j in elarge1:
    
    if( m[i]==j[0] and m[i+1]==j[1] )or (m[i]==j[1] and m[i+1]==j[0] ):
      inbet.append(m[i])
      res+=j[-1]
      break
  if res>=costk:
    inbet.append(m[i+1])
    final.append(inbet)
    #print(res)
    l+=res
    #print(inbet)
    inbet=[]
    res=0
    i+=1
  i+=1
inbet.append(m[-1])
final.append(inbet)
#print(res)
#print(inbet)
l+=res
#print(l)
for i in range(1,len(final)):
  final[i].insert(0,m[0])
  #print(totalcost(final[i]))

newpath=[[],[],[],[]]
for i in range(10):
  i_0=totalcost(final[0])
  i_1=totalcost(final[1])
  i_2=totalcost(final[2])
  i_3=totalcost(final[3])
  l_cost=[i_0,i_1,i_2,i_3]
  #print("old",max(i_0,i_1,i_2,i_3))
  oldm=max(i_0,i_1,i_2,i_3)
  minpos = l_cost.index(min(l_cost)) 
  maxpos = l_cost.index(max(l_cost))  
  #print(maxpos,minpos)
  topop=final[maxpos].pop()
  final[minpos].insert(1,topop)
  i_0=totalcost(final[0])
  i_1=totalcost(final[1])
  i_2=totalcost(final[2])
  i_3=totalcost(final[3])
  #print("new",max(i_0,i_1,i_2,i_3))
  newm=max(i_0,i_1,i_2,i_3)
  if newm<oldm:
    newpath[0]=final[0]
    newpath[1]=final[1]
    newpath[2]=final[2]
    newpath[3]=final[3]
  else:
    break
  
print(newpath)
robgoals=newpath
robgoals[0].append(1)
robgoals[1].append(1)
robgoals[2].append(1)
robgoals[3].append(1)