# 1. ENVIROMENT SETUP
import open3d as o3d
import numpy as np
import os
import sys
import pandas as pd
import matplotlib.pyplot as plt

# 2. POINT CLOUD DATA PREPARATION
pcd = o3d.io.read_point_cloud("1.ply")

# 3. DATA PRE-PROCESSING
pcd_centre=pcd.get_center()
pcd.translate(-pcd_centre)

# 3.1 STATISTICAL OUTLIER FILTER
nn=16
std_multiplier=10
filtered_pcd=pcd.remove_statistical_outlier(nn,std_multiplier)
outliers = pcd.select_by_index(filtered_pcd[1],invert=True)
outliers.paint_uniform_color([1,0,0])
filtered_pcd=filtered_pcd[0]
o3d.visualization.draw_geometries([filtered_pcd, outliers])

# 3.2 VOXEL DOWNSAMPLING
voxel_size = 45                              # PARAM TO SETUP
pcd_downsampled = filtered_pcd.voxel_down_sample(voxel_size=voxel_size)

# PRINT CLOUD POINTS PRE-POST DOWNSAMPLING
print("pcd",pcd)                            # PRE
print("pcd_downsampled",pcd_downsampled)    # POST

# 3.3 ESTIMATING NORMALS
nn_distance=np.mean(pcd.compute_nearest_neighbor_distance())
radius_normals = nn_distance * 4
pcd_downsampled.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normals,max_nn=16), fast_normal_computation=True)
pcd_downsampled.paint_uniform_color([0.6,0.6,0.6])
o3d.visualization.draw_geometries([pcd])
o3d.visualization.draw_geometries([pcd_downsampled,outliers])


# 4 RANSAC PLANAR SEGMENTATION
pt_to_plane_dist = 0.7   # SETUP TO THE WORKSPACE BASE
plane_model, inliers = pcd.segment_plane(distance_threshold=pt_to_plane_dist, ransac_n=3, num_iterations=1000)
[a,b,c,d]=plane_model
inlier_cloud = pcd.select_by_index(inliers)
planeseg = pd.DataFrame(np.asarray(inlier_cloud.points), columns=['x', 'y', 'z'])
rest = pd.DataFrame(np.asarray(pcd_downsampled.points), columns=['x', 'y', 'z'])
rest = rest[(rest['z'] >= 10+max(planeseg['z']))]
outlier_cloud = o3d.geometry.PointCloud()
outlier_cloud.points = o3d.utility.Vector3dVector(rest.values)
inlier_cloud.paint_uniform_color([1.0,0,0])
outlier_cloud.paint_uniform_color([0.6,0.6,0.6])
#o3d.visualization.draw_geometries([inlier_cloud,outlier_cloud])
#o3d.visualization.draw_geometries([outlier_cloud])

# 5. DBSCAN
labels = np.array(outlier_cloud.cluster_dbscan(eps=105, min_points=10))
max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")
clusters = []
for i in range(max_label + 1):
    cluster = o3d.geometry.PointCloud()
    cluster.points = o3d.utility.Vector3dVector(np.asarray(outlier_cloud.points)[labels == i])
    clusters.append(cluster)
colors = plt.get_cmap("tab10")(labels/(max_label if max_label > 0 else 1))
colors[labels<0]=0
for i, cluster in enumerate(clusters):
    cluster.colors = o3d.utility.Vector3dVector(colors[labels == i, :3])
o3d.visualization.draw_geometries([inlier_cloud]+clusters)

# 6 PC FUNCTION
def contactPoints(obb):
    # 6.0 SETUP ENVIROMENT
    pp1 = np.array(obb.get_box_points())
    center = obb.get_center()
    x_components = np.array([v[0] for v in pp1])
    y_components = np.array([v[1] for v in pp1])
    z_components = np.array([v[2] for v in pp1])

    # 6.1 INIT
    pc = []
    left_1 = []
    right_1 = []
    front_1 = []
    backward_1 = []

    # 6.2 DETERMINE LEFT/RIGHT POINTS
    for i, x in enumerate(x_components):
        if x > center[0]:
            right_1.append(pp1[i])
        else:
            left_1.append(pp1[i])
    centroid_right = sum(right_1) / len(right_1)
    centroid_left = sum(left_1) / len(left_1)

    # 6.3 DETERMINE FRONT/BACK POINTS
    for i, y in enumerate(y_components):
        if y > center[1]:
            front_1.append(pp1[i])
        else:
            backward_1.append(pp1[i])

    # 6.4 CENTROID CALCULATION
    centroid_right = sum(right_1) / len(right_1)
    centroid_left = sum(left_1) / len(left_1)
    centroid_front = sum(front_1) / len(front_1)
    centroid_backward = sum(backward_1) / len(backward_1)

    # 6.5 INSERT POINT CONTACT INTO POINT CLOUD
    pcPoints = o3d.geometry.PointCloud()
    pcPoints.points = o3d.utility.Vector3dVector([centroid_right, centroid_left, centroid_front, centroid_backward])
    return pcPoints

# 7. FOR EACH CLUSTER DETERMINE BOUNDING BOX AND POINT CONTACTS
obbs=[]
hulls=[]
pcs=[]
for i in range(max_label + 1):
    hull, _ = clusters[i].compute_convex_hull()
    hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
    hull_ls.paint_uniform_color([1.0,0.0,0.0])
    hulls.append(hull_ls)
    obb = clusters[i].get_oriented_bounding_box()
    obb.color = (0, 1, 0)
    obbs.append(obb)
    pc = contactPoints(obb)
    pc.paint_uniform_color([0.0,0.0,1.0])
    pcs.append(pc)

# 8. VISUALIZE RESULTS
o3d.visualization.draw_geometries([pcd]+obbs+pcs)
