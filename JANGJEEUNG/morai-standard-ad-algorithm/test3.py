#%%
# 모든 노드와 링크를 캔버스에 나타내기

import numpy as np
import matplotlib.pyplot as plt
import json

link_set = None
node_set = None
with open("mgeo/lib/mgeo_data/R_KR_PG_K-City/link_set.json", "r",encoding="utf-8" ) as f:
    link_set = json.load(f)

with open("mgeo/lib/mgeo_data/R_KR_PG_K-City/node_set.json", "r",encoding="utf-8" ) as f:
    node_set = json.load(f)

# link_set의 포인트는 작은 흰점
# node_set의 포인트는 큰 노란점

point_link_set = []

for i in range(len(link_set)):
    for j in range(len(link_set[i])):
        for k in range(len(link_set[i]["points"])):
            point_link_set.append(link_set[i]["points"][k][:2])

point_node_set = []

for i in range(len(node_set)):
    for j in range(len(node_set[i])):
            point_node_set.append(node_set[i]["point"][:2])

np_link = np.array(point_link_set)
np_node = np.array(point_node_set)
    
plt.scatter(*np_link.T)
plt.scatter(*np_node.T)
plt.show()
