import json
import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares, minimize
from scipy.interpolate import make_interp_spline
import open3d as o3d



#################################################
##########          json2mask          ##########
#################################################
def polygons_json2mask(json_path, save_path):
    # load img and json
    data = json.load(open(json_path,encoding='gbk'))

    img_h = 1080
    img_w = 1440
    img = np.zeros([img_h,img_w])

    color_bg = (0)
    color_seam = (255)
    points_bg = [(0, 0), (0, img_h), (img_w, img_h), (img_w, 0)]
    img = cv2.fillPoly(img, [np.array(points_bg)], color_bg)

    # draw roi
    for i in range(len(data['shapes'])):
        # data['shapes']
        points = data['shapes'][i]['points']

        img = cv2.fillPoly(img, [np.array(points, dtype=int)], color_seam)
    cv2.imwrite(save_path, img)


def remove_duplicates_2d(arr):
    seen = set()
    result = []
    
    for item in arr:
        tuple_item = tuple(item)
        if tuple_item not in seen:
            seen.add(tuple_item)
            result.append(item)
    
    return result


def smooth_curve(points, interval = None, num_samples = None, fit_points_num = 10, func_degree=3):  
    
    points = remove_duplicates_2d(points)
    points = np.array(points)
    total_length = 0  
    for i in range(points.shape[0]-1):  
        total_length += np.linalg.norm(points[i+1]-points[i])  
      
    if num_samples == None:
        num_samples = int(total_length // interval)  
      
    distances = np.zeros(points.shape[0])  
    for i in range(1, points.shape[0]):  
        distances[i] = distances[i-1] + np.linalg.norm(points[i]-points[i-1])  
      
    interp_func = make_interp_spline(distances, points, k=func_degree)  
      
    sampled_distances = np.linspace(0, distances[-1], num_samples)  
    sampled_points = np.zeros((num_samples, 2))  

    half_fit_points_num = fit_points_num//2
    for i in range(num_samples):  
        start_idx = max(i-half_fit_points_num, 0)  
        end_idx = min(i+half_fit_points_num, num_samples-1)  
        sub_sampled_distances = sampled_distances[start_idx:end_idx+1]  
        sub_sampled_points = interp_func(sub_sampled_distances)  
        # sampled_points[i] = sub_sampled_points[len(sub_sampled_points)//2]
        sampled_points[i] = sub_sampled_points[i-start_idx]

    return sampled_points


def lines_json2mask(json_path, thickness, save_path=None):
    # load img and json
    data = json.load(open(json_path,encoding='gbk'))

    img_h = 1080
    img_w = 1440
    img = np.zeros([img_h,img_w])

    color_bg = (0)
    color_seam = (255)
    points_bg = [(0, 0), (0, img_h), (img_w, img_h), (img_w, 0)]
    img = cv2.fillPoly(img, [np.array(points_bg)], color_bg)

    # draw roi
    for i in range(len(data['shapes'])):
        # data['shapes']
        points = data['shapes'][i]['points']
        if len(points)>2:
            points = smooth_curve(points, interval = None, num_samples = 100, fit_points_num = 5, func_degree=3)

        cv2.polylines(img, [np.array(points, dtype=int)], isClosed=False, color=color_seam, thickness=thickness)
    
    if save_path is not None: cv2.imwrite(save_path, img)
    return img



# #################################################
# ##########          visualize          ##########
# #################################################

def data2pcd(data):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data[:,:3])
    if np.any(data[:,3:]):
        pcd.colors = o3d.utility.Vector3dVector(data[:,3:6])
    return pcd


def get_arrow(begin=[0,0,0],vec_Arr=[0,0,1],ratio = 2):
    def get_cross_prod_mat(pVec_Arr):
        # pVec_Arr shape (3)
        qCross_prod_mat = np.array([
            [0, -pVec_Arr[2], pVec_Arr[1]],
            [pVec_Arr[2], 0, -pVec_Arr[0]],
            [-pVec_Arr[1], pVec_Arr[0], 0],
        ])
        return qCross_prod_mat
    
    def caculate_align_mat(pVec_Arr):
        scale = np.linalg.norm(pVec_Arr)
        pVec_Arr = pVec_Arr / scale
        # must ensure pVec_Arr is also a unit vec.
        z_unit_Arr = np.array([0, 0, 1])
        z_mat = get_cross_prod_mat(z_unit_Arr)
    
        z_c_vec = np.matmul(z_mat, pVec_Arr)
        z_c_vec_mat = get_cross_prod_mat(z_c_vec)
    
        if np.dot(z_unit_Arr, pVec_Arr) == -1:
            qTrans_Mat = -np.eye(3, 3)
        elif np.dot(z_unit_Arr, pVec_Arr) == 1:
            qTrans_Mat = np.eye(3, 3)
        else:
            qTrans_Mat = np.eye(3, 3) + z_c_vec_mat + np.matmul(z_c_vec_mat,
                                                                z_c_vec_mat) / (1 + np.dot(z_unit_Arr, pVec_Arr))

        qTrans_Mat *= scale
        return qTrans_Mat
    begin = begin

    
    mesh_arrow = o3d.geometry.TriangleMesh.create_arrow(
        cone_height=0.0002 * ratio ,
        cone_radius=0.0001 * ratio,
        cylinder_height=0.0005 * ratio,
        cylinder_radius=0.00005 * ratio
    )
    mesh_arrow.paint_uniform_color([0, 1, 0])
    mesh_arrow.compute_vertex_normals()

    rot_mat = caculate_align_mat(vec_Arr)
    mesh_arrow.rotate(rot_mat, center=np.array([0, 0, 0]))
    mesh_arrow.translate(np.array(begin))  # 0.5*(np.array(end) - np.array(begin))
    return mesh_arrow


def vis_points_direction(sampled_points, sampled_directions):
    sampled_pcd = []
    pcd = data2pcd(sampled_points)
    for i in range(len(sampled_points)):
        coord = get_arrow(begin=sampled_points[i],vec_Arr=sampled_directions[i],ratio = 2)
        sampled_pcd.append(coord)
    sampled_pcd.append(pcd)
    o3d.visualization.draw_geometries(sampled_pcd)

##########################################################
##########          trajectory_process          ##########
##########################################################

def cal_tangent_vector_4order(points):
    x = points[:,0]
    y = points[:,1]
    z = points[:,2]
    tangent_vectors = []
    for i in range(len(x)):
        if i == 0:
            tangent_vector = np.array([x[i+1] - x[i], y[i+1] - y[i], z[i+1] - z[i]])
        elif i == 1:
            tangent_vector = np.array([x[i+1] - x[i-1], y[i+1] - y[i-1], z[i+1] - z[i-1]])
        elif i == len(x) - 2:
            tangent_vector = np.array([x[i-1] - x[i+1], y[i-1] - y[i+1], z[i-1] - z[i+1]])
        elif i == len(x) - 1:
            tangent_vector = np.array([x[i] - x[i-1], y[i] - y[i-1], z[i] - z[i-1]])
        else:
            tangent_vector = np.array([x[i-2] - 8 * x[i-1] + 8 * x[i+1] - x[i+2], y[i-2] - 8 * y[i-1] + 8 * y[i+1] - y[i+2],
                                       z[i-2] - 8 * z[i-1] + 8 * z[i+1] - z[i+2]])
        tangent_vector /= np.linalg.norm(tangent_vector)
        tangent_vectors.append(tangent_vector)
    tangent_vectors = np.array(tangent_vectors)
    
    return tangent_vectors



def smooth_curve_4order(points, interval = None, num_samples = None, fit_points_num = 10, func_degree=3):  
    total_length = 0  
    for i in range(points.shape[0]-1):  
        total_length += np.linalg.norm(points[i+1]-points[i])  
      
    if num_samples == None:
        num_samples = int(total_length // interval)
      
    distances = np.zeros(points.shape[0])  
    for i in range(1, points.shape[0]):  
        distances[i] = distances[i-1] + np.linalg.norm(points[i]-points[i-1])  
      
    interp_func = make_interp_spline(distances, points, k=func_degree)  
      
    sampled_distances = np.linspace(0, distances[-1], num_samples)  
    sampled_points = np.zeros((num_samples, 3))  
    sampled_directions = np.zeros((num_samples, 3))  

    half_fit_points_num = fit_points_num//2
    for i in range(num_samples):  
        start_idx = max(i-half_fit_points_num, 0)  
        end_idx = min(i+half_fit_points_num, num_samples-1)  
        sub_sampled_distances = sampled_distances[start_idx:end_idx+1]  
        sub_sampled_points = interp_func(sub_sampled_distances)  
        sampled_points[i] = sub_sampled_points[i-start_idx]

    sampled_directions = cal_tangent_vector_4order(sampled_points)

      
    return sampled_points, sampled_directions


# trajectory: NX6
def interpolate_curve(trajectory, num_samples=200, fit_points_num = 20, func_degree=2, vis_flag = False):
    sampled_points, sampled_directions = smooth_curve_4order(trajectory, num_samples=num_samples, fit_points_num = fit_points_num, func_degree=func_degree)
    if vis_flag:
        vis_points_direction(sampled_points, sampled_directions)
    return sampled_points, sampled_directions

def fit_straight_equation(points, num_samples = 200, vis_flag=False):
    centroid = np.mean(points, axis=0)

    points_adjusted = points - centroid

    def residuals(params, points):
        point = points[0]
        vec = points[1:] - point
        normal_vec = vec - np.dot(vec, params)[:, np.newaxis] * np.array(params)
        return np.sqrt((normal_vec ** 2).sum(axis=1))

    initial_guess = [1, 0, 0]

    result = least_squares(residuals, initial_guess, args=(points_adjusted,))

    direction_vector = result.x

    def project_point2line(point, centroid, direction_vector):
        d = direction_vector
        P = centroid
        A = point
        AP = P - A
        PB = (np.dot(AP, d) / np.dot(d, d)) * d
        B = P - PB
        return B


    start_point = project_point2line(points[0], centroid, direction_vector)
    end_point = project_point2line(points[-1], centroid, direction_vector)


    # Plot the fitted line
    t = np.linspace(0, 1, num_samples) * (end_point[0]-start_point[0]) / direction_vector[0]
    x = start_point[0] + direction_vector[0] * t
    y = start_point[1] + direction_vector[1] * t
    z = start_point[2] + direction_vector[2] * t

    sampled_points = np.vstack((x,y,z)).T

    if vis_flag:
        # Create a 3D figure
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        # Plot the 3D points
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='r', marker='o')

        ax.plot(x, y, z, 'b-')

        # Set labels and title
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Fitted 3D Line')

        # Show the plot
        plt.show()

    equation_data = [start_point, end_point, direction_vector]
    return sampled_points, equation_data


def project_point_to_plane(plane_equation, point_cloud, vis_flag = False):
    A, B, C, D = plane_equation

    projected_x = point_cloud[:, 0] - (A * point_cloud[:, 0] + B * point_cloud[:, 1] + C * point_cloud[:, 2] + D) * A / (A**2 + B**2 + C**2)
    projected_y = point_cloud[:, 1] - (A * point_cloud[:, 0] + B * point_cloud[:, 1] + C * point_cloud[:, 2] + D) * B / (A**2 + B**2 + C**2)
    projected_z = point_cloud[:, 2] - (A * point_cloud[:, 0] + B * point_cloud[:, 1] + C * point_cloud[:, 2] + D) * C / (A**2 + B**2 + C**2)

    projected_point_cloud = np.column_stack((projected_x, projected_y, projected_z))
    if vis_flag:
        pcd = data2pcd(projected_point_cloud)
        pcd.paint_uniform_color([1,0,0])
        o3d.visualization.draw_geometries([pcd])

        # point_visualization(pcd)
    
    return projected_point_cloud


def fit_circle(points,r, num_samples = 50, vis_flag=False):
    origin_pcd = data2pcd(points)

    plane_equation, inliers = origin_pcd.segment_plane(distance_threshold=0.0005,
                                            ransac_n=5,
                                            num_iterations=1000)
    # print(plane_equation)
    projected_points = project_point_to_plane(plane_equation, points, vis_flag = False)

    def rotate_vector(vector, axis, theta):
        # 将向量和旋转轴转换为NumPy数组
        vector = np.array(vector)
        vector /= np.linalg.norm(vector)
        axis = np.array(axis)
        
        axis = axis / np.linalg.norm(axis)
        
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        rotation_matrix = np.array([[cos_theta + axis[0]**2 * (1 - cos_theta),
                                    axis[0] * axis[1] * (1 - cos_theta) - axis[2] * sin_theta,
                                    axis[0] * axis[2] * (1 - cos_theta) + axis[1] * sin_theta],
                                    [axis[1] * axis[0] * (1 - cos_theta) + axis[2] * sin_theta,
                                    cos_theta + axis[1]**2 * (1 - cos_theta),
                                    axis[1] * axis[2] * (1 - cos_theta) - axis[0] * sin_theta],
                                    [axis[2] * axis[0] * (1 - cos_theta) - axis[1] * sin_theta,
                                    axis[2] * axis[1] * (1 - cos_theta) + axis[0] * sin_theta,
                                    cos_theta + axis[2]**2 * (1 - cos_theta)]])
        
        rotated_vector = np.dot(rotation_matrix, vector)
        
        return rotated_vector



    def objective(params,r, points, plane_equation):
        [A, B, C, D] = plane_equation
        cx, cy = params
        cz = (A*cx + B*cy + D) / (-C)
        distances = np.sqrt(np.sum((points - np.array([cx, cy, cz]))**2, axis=1)) - r
        return np.sum(distances**2)

    def fit_circle_3d(points,r, num_points_on_circle, plane_equation):
        init_params = [0, 0]
        
        result = minimize(objective, init_params, args=(r,points, plane_equation))
        cx, cy = result.x
        [A, B, C, D] = plane_equation
        cz = (A*cx + B*cy + D) / (-C)

        center_point = [cx, cy, cz]

        
        theta = np.linspace(0, 2*np.pi, num_points_on_circle)
        circle_points = np.zeros((num_points_on_circle, 3))

        vector = [1, 1, -(A+B)/C]

        for i in range(num_points_on_circle):
            circle_points[i] = np.array([cx,cy,cz])+rotate_vector(vector, [A,B,C], theta[i])*r

        equation_data = [center_point, plane_equation, r]
        return circle_points, equation_data

    
    circle_points, equation_data = fit_circle_3d(projected_points,r, num_samples, plane_equation)


    if vis_flag:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='b', label='Original Points')
        ax.scatter(circle_points[:, 0], circle_points[:, 1], circle_points[:, 2], c='r', label='Circle Points')
        ax.scatter(projected_points[:, 0], projected_points[:, 1], projected_points[:, 2], c='g', label='Projected Points')
        ax.legend()
        plt.show()
    
    return circle_points, equation_data
