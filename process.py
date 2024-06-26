import numpy as np
from tools import Mesho3d
import open3d as o3d
import random
from scipy.spatial.transform import Rotation as R


random.seed(0)
np.random.seed(0)
o3d.utility.random.seed(0)


def create_plane(a, b, c, d):
    # Define a grid of points on the plane
    xx, yy = np.meshgrid(np.arange(-3, 3, 0.1), np.arange(-3, 3, 0.1))
    zz = (-a * xx - b * yy - d) / c

    # Create vertices
    vertices = np.column_stack([xx.flatten(), yy.flatten(), zz.flatten()])

    # Create triangles
    triangles = []
    rows, cols = xx.shape
    for i in range(rows - 1):
        for j in range(cols - 1):
            triangles.append([i * cols + j, i * cols + j + 1, (i + 1) * cols + j])
            triangles.append([i * cols + j + 1, (i + 1) * cols + j + 1, (i + 1) * cols + j])

    # Create mesh
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(triangles)
    return mesh


def get_target_normal(plane_name):
    if plane_name == 'x':
        return np.array([1.0, 0, 0], dtype=np.float32)
    elif plane_name == 'y':
        return np.array([0, 1.0, 0], dtype=np.float32)
    elif plane_name == 'z':
        return np.array([0, 0, 1.0], dtype=np.float32)
    else:
        raise Exception("plane name not known!")


def get_norm(v):
    return np.sqrt(np.sum(v**2))


def get_to_origin(pts_plane):
    return np.mean(pts_plane, axis=0)*-1


def get_rot_matrix(v1, v2):
    # make them unit vectors
    v1 = v1/get_norm(v1)
    v2 = v2/get_norm(v2)

    rot_axis = np.cross(v1, v2)
    rot_axis = rot_axis/get_norm(rot_axis)

    rot_angle = np.arccos(np.dot(v1, v2))

    u_x = np.array([[0, -rot_axis[2], rot_axis[1]],
                    [rot_axis[2], 0, -rot_axis[0]],
                    [-rot_axis[1], rot_axis[0], 0]])
    
    # rodrigues rotation formula
    R = np.cos(rot_angle) * np.eye(3) + np.sin(rot_angle) * u_x + (1 - np.cos(rot_angle)) * np.outer(rot_axis, rot_axis)
    return R


def find_plane(pts, thresh=0.01, maxIteration=1000):
        n_points = pts.shape[0]
        best_eq = []
        best_inliers = []

        for it in range(maxIteration):

            # samples 3 random points
            idx = random.sample(range(0, n_points), 3)
            samp_pts = pts[idx]

            v1, v2 = samp_pts[1, :] - samp_pts[0, :], samp_pts[2, :] - samp_pts[0, :]
            normal_ = np.cross(v1, v2)
            normal_ = normal_ / get_norm(normal_)

            k = -np.sum(np.multiply(normal_, samp_pts[1, :]))
            plane_eq = np.array([normal_[0], normal_[1], normal_[2], k])

            # distance from a point to a plane
            pt_id_inliers = []
            dists = np.sum(pts*plane_eq[:3], axis=1)+plane_eq[3]

            # find inliers based on threshold
            pt_id_inliers = np.where(np.abs(dists) <= thresh)[0]
            if len(pt_id_inliers) > len(best_inliers):
                best_eq = list(plane_eq)
                best_inliers = list(pt_id_inliers)

        return best_eq, best_inliers


def reorient(args, geo):
    #### Want to down sample point clouds here?

    # Get random rotation
    random_rotation = R.random().as_matrix()
    geo.pc.points = o3d.utility.Vector3dVector(np.dot(random_rotation, np.array(geo.pc.points).T).T)

    pts = np.array(geo.pc.points)
    print(f"\nNumber of points in the given point cloud: {len(pts)}")

    # open3d plane segmentation
    if args.find_plane_method == 'open3d':
        plane_model, inliers = geo.pc.segment_plane(distance_threshold=0.01,
                                                ransac_n=3,
                                                num_iterations=1000)
    elif args.find_plane_method == 'ransac':
        # ransac plane segmentation
        plane_model, inliers = find_plane(pts)
    else:
        raise Exception("method not known!")

    [a, b, c, d] = plane_model
    plane_model = np.array(plane_model)
    print(f"\nPlane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    pts_plane = pts[inliers]
    print(f"Number of planar points in the given point cloud: {len(pts_plane)}")

    # draw the plane
    # plane = create_plane(a, b, c, d)
    # Mesho3d(v=np.array(plane.vertices), f=None).write_obj("./plane.obj")

    # move pts to origin
    to_origin = get_to_origin(pts_plane)
    aug_pts_plane = pts_plane + to_origin
    aug_pts = pts + to_origin

    # get rotation matrix
    tar_normal = get_target_normal(args.select_plane)
    rot_mat = get_rot_matrix(plane_model[:3], tar_normal)
    
    # augment using rotation matrix
    aug_pts_plane = np.dot(rot_mat, aug_pts_plane.T).T
    aug_pts = np.dot(rot_mat, aug_pts.T).T

    pc_name = args.pc_path.split('/')[-1][:-4]
    Mesho3d(v=pts_plane, f=None).write_obj(f"./{pc_name}_pts_plane.obj")
    Mesho3d(v=aug_pts_plane, f=None).write_obj(f"./{pc_name}_aug_pts_plane.obj")
    Mesho3d(v=pts, f=None).write_obj(f"./{pc_name}_pts.obj")
    Mesho3d(v=aug_pts, f=None).write_obj(f"./{pc_name}_aug_pts.obj")


def change_representation(args, geo):

    pts = np.array(geo.pc.points)
    print(f"\nNumber of points in the given point cloud: {len(pts)}")

    voxel_down_pc = geo.pc.voxel_down_sample(voxel_size=0.02)
    ds_pts = np.array(voxel_down_pc.points)
    print(f"\nNumber of points in the down sampled point cloud: {len(ds_pts)}")

    voxel_down_pc, ind = voxel_down_pc.remove_radius_outlier(nb_points=32, radius=0.05)
    ds_pts_removal = np.array(voxel_down_pc.points)
    print(f"\nNumber of points after outlier removal: {len(ds_pts_removal)}")

    voxel_down_pc.estimate_normals()
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(voxel_down_pc, depth=10)
    
    pc_name = args.pc_path.split('/')[-1][:-4]
    Mesho3d(v=np.array(mesh.vertices), f=np.array(mesh.triangles)).write_obj(f"./{pc_name}_recon.obj")
    Mesho3d(v=pts, f=None).write_obj(f"./{pc_name}_pts.obj")
    Mesho3d(v=ds_pts, f=None).write_obj(f"./{pc_name}_ds_pts.obj")
    Mesho3d(v=ds_pts_removal, f=None).write_obj(f"./{pc_name}_ds_pts_removal.obj")


def main(args):

    geo = Mesho3d(filename=args.pc_path)

    if args.operation == 'orient':
        reorient(args, geo)
    elif args.operation == 'surface':
        change_representation(args, geo)
    else:
        raise Exception("operation not known!")
    


if __name__ == '__main__':

    import argparse
    parser = argparse.ArgumentParser(description='info')
    
    parser.add_argument('--pc_path',
                        default='./pcs/shoe_pc.ply',
                        type=str,
                        help="Give the path to the point cloud needed to be processed")

    parser.add_argument('--operation',
                        default='orient',
                        type=str,
                        help="What operation to perform? (orient/surface)")
    
    parser.add_argument('--find_plane_method',
                        default='open3d',
                        type=str,
                        help="which method to use to find floor plane equation? (ransac/open3d)")

    parser.add_argument('--select_plane',
                        default='x',
                        type=str,
                        help="Plane to which floor should be aligned, (eg. default is x=0)")
    

    args = parser.parse_args()

    main(args)