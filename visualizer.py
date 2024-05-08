# 1. ENVIROMENT SETUP
import open3d as o3d
import numpy as np
import os
import sys
import pandas as pd
import matplotlib.pyplot as plt

# 2. POINT CLOUD DATA PREPARATION
pcd = o3d.io.read_point_cloud("2.ply")
o3d.visualization.draw_geometries([pcd])
