import os
from dataset_preprocess_utils import *


def get_real_trajectory(type, type_idx, part_idx, num_samples = 200, base_path = "RGBD_Coating_Dataset/", vis_flag=False):
    dirname = ''.join([base_path, type, '/', type, '_', str(type_idx+1), '/'])
    tra_file = f'{dirname}trajectory/{part_idx}.npz'
    trajectory = np.load(tra_file)['xyzqua_base'][:,:]

    if type == 'curve':
        sampled_points, sampled_directions = interpolate_curve(trajectory[:,:3], 
                                num_samples=num_samples, fit_points_num = 20, func_degree=2, vis_flag = vis_flag)
        return sampled_points, sampled_directions
    elif type == 'straight':
        sampled_points, equation_data = fit_straight_equation(trajectory[:,:3], 
                                                                num_samples = num_samples, vis_flag=vis_flag)
        return sampled_points, equation_data

    elif type == 'circle':
        r_list = [0.043, 0.0553, 0.0497, 0.063]
        sampled_points, equation_data = fit_circle(trajectory[:,:3], r_list[type_idx], 
                                                    num_samples = num_samples, vis_flag=vis_flag)
        return sampled_points, equation_data



if __name__ == '__main__':

    base_path = "./"
    types = {'circle':4, 'curve':5, 'straight':4}
    
    get_real_trajectory(type="circle", type_idx=0, part_idx=1, num_samples = 200, base_path = base_path, vis_flag=True)
    
