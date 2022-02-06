import open3d as o3d
import numpy as np
import matplotlib.cm as plt

# load ply
print("1 - Load a ply point cloud and visualize it")
pcd = o3d.io.read_point_cloud("/Users/veronika/Desktop/input_data.ply")
o3d.visualization.draw_geometries([pcd])
print("Done 1")

# downsample
print("2 - Downsample the point cloud with a voxel of 0.01")
downpcd = pcd.voxel_down_sample(voxel_size=0.02)
o3d.visualization.draw_geometries([downpcd])
print("Done 2")

# compute normals
print("3 - Recompute the normal of the downsampled point cloud")
downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
o3d.visualization.draw_geometries([downpcd])
print("Done 3")

# orient normals
print("4 - Orient computed normals")
downpcd.orient_normals_towards_camera_location(camera_location=np.array([0., 0., 0.]))
o3d.visualization.draw_geometries([downpcd])
print("Done 4")

# reconstruct via Poisson
print("5 - Reconstruct the point cloud via Poisson algorythm")
with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(downpcd, depth=9)
o3d.visualization.draw_geometries([mesh])
print("Done 5")

# visualize low density areas
print("6 - Visualize mesh density")
densities = np.asarray(densities)
density_colors = plt.get_cmap('plasma')((densities - densities.min()) / (densities.max() - densities.min()))
density_colors = density_colors[:, :3]
density_mesh = o3d.geometry.TriangleMesh()
density_mesh.vertices = mesh.vertices
density_mesh.triangles = mesh.triangles
density_mesh.triangle_normals = mesh.triangle_normals
density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)
o3d.visualization.draw_geometries([density_mesh])
print("Done 6")

# remove vertices with low density
print("7 - Remove low density vertices")
vertices_to_remove = densities < np.quantile(densities, 0.01)
mesh.remove_vertices_by_mask(vertices_to_remove)
o3d.visualization.draw_geometries([mesh])
print("Done 7")

# save
o3d.io.write_triangle_mesh("/Users/veronika/Desktop/poisson_output_mesh.ply", mesh)

#------------
#print("5 - Reconstruct the point cloud via BPA algorythm")
#distances = downpcd.compute_nearest_neighbor_distance()
#avg_dist = np.mean(distances)
#radius = 2 * avg_dist
#
#bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(downpcd,
#                                                                           o3d.utility.DoubleVector([radius, 
#                                                                                                     radius * 2]))
#dec_mesh = bpa_mesh.simplify_quadric_decimation(100000)
#dec_mesh.remove_degenerate_triangles()
#dec_mesh.remove_duplicated_triangles()
#dec_mesh.remove_duplicated_vertices()
#dec_mesh.remove_non_manifold_edges()
#
#o3d.visualization.draw_geometries([dec_mesh])
