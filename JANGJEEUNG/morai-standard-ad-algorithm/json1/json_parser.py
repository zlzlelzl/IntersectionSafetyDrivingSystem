#%%
import numpy as np
import matplotlib.pyplot as plt
import pickle
import json
import os

#%%
with open("./lane_boundary_set.json") as f:
    json_object = json.load(f)

# %%
json_object