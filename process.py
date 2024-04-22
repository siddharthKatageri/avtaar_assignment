import numpy as np
from tools import Mesho3d
import open3d as o3d
import random

random.seed(0)
np.random.seed(0)
o3d.utility.random.seed(0)


def create_plane(a, b, c, d):
    """
    Create a mesh representing a plane defined by the equation ax + by + cz + d = 0.
    """
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


def reorient(args, geo):
    #### Want to down sample point clouds here?

    pts = np.array(geo.pc.points)
    print(f"\nNumber of points in the given point cloud: {len(pts)}")

    # open3d plane segmentation
    plane_model, inliers = geo.pc.segment_plane(distance_threshold=0.01,
                                            ransac_n=3,
                                            num_iterations=1000)

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

    Mesho3d(v=pts_plane, f=None).write_obj("./pts_plane.obj")
    Mesho3d(v=aug_pts_plane, f=None).write_obj("./aug_pts_plane.obj")
    Mesho3d(v=pts, f=None).write_obj("./pts.obj")
    Mesho3d(v=aug_pts, f=None).write_obj("./aug_pts.obj")


def change_representation(args, geo):
    pass


def main(args):

    geo = Mesho3d(filename=args.pc_path)

    if args.operation == 'orient':
        reorient(args, geo)
    elif args.operation == 'represent':
        change_representation(args, geo)
    else:
        raise Exception("operation not known!")
    




    exit()



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
                        help="What operation to perform? (orient/represent)")

    parser.add_argument('--select_plane',
                        default='x',
                        type=str,
                        help="Plane to which floor should be aligned, (eg. default is x=0)")
    

    args = parser.parse_args()

    main(args)