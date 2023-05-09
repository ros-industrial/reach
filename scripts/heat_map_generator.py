import argparse
import numpy as np
import open3d as o3d
import os.path
from reach import ReachDatabase, load, computeHeatMapColors, normalizeScores
from scipy.interpolate import RBFInterpolator


def main():
    parser = argparse.ArgumentParser(description="Generate a reachability heatmap from a preexisting reach database.")
    parser.add_argument(type=str, dest="db_file", help="Filepath of the reach database")
    parser.add_argument(type=str, dest="mesh_file", help='Filepath of the part mesh')
    parser.add_argument("-k", "--kernel", type=str, default="thin_plate_spline", help="Kernel for RBF interpolation")
    parser.add_argument("-e", "--epsilon", type=float, default=None, help="Shape parameter for RBF interpolation")
    parser.add_argument("-s", "--smoothing", type=float, default=0.0, help="Smoothing parameter for RBF interpolation")
    parser.add_argument("-o", "--output-mesh", type=str, default=None, help="Filepath for output heatmap")
    parser.add_argument("-n", "--number-subdivisions", type=int, default=2,
                        help="Order of subdivision. Each triangle is divided once for n iterations")
    parser.add_argument("-fcr", "--full-color-range", action='store_true', default=False,
                        help="Display scores using the full color range rather than only scaling scores by the max")
    args = parser.parse_args()

    # Load database
    if not os.path.exists(args.db_file):
        raise FileExistsError(f'File \'{args.db_file}\' does not exist')
    db = load(args.db_file)

    # Use the last set of results in the database
    res = db.results[-1]

    # Loop over records in database to extract point position and scores into Numpy array
    positions = np.array([r.goal()[0:3, 3] for r in res])
    scores = normalizeScores(res, args.full_color_range)

    # Calculate the RBF
    rbf = RBFInterpolator(y=positions, d=scores, kernel=args.kernel, epsilon=args.epsilon,
                          smoothing=args.smoothing)

    # Load the mesh and subdivide it
    mesh = o3d.io.read_triangle_mesh(args.mesh_file).subdivide_midpoint(args.number_subdivisions)

    # Extract the vertices of the sub-sampled mesh as a numpy array and calculate the interpolated score for each
    dims = rbf.y.shape[1]
    vert_scores = rbf(np.asarray(mesh.vertices)[:, :dims])

    # Clip the scores on [0, 1]
    vert_scores = np.clip(vert_scores, a_min=0.0, a_max=1.0)

    # Colorize the mesh vertices
    mesh.vertex_colors = o3d.utility.Vector3dVector(computeHeatMapColors(vert_scores.tolist()))

    # Visualize the output
    o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=False)

    if args.output_mesh is not None:
        o3d.io.write_triangle_mesh(args.output_mesh, mesh)


if __name__ == '__main__':
    main()
