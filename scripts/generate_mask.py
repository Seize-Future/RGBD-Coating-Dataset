import os
from dataset_preprocess_utils import *


def generate_mask(base_path, types, thickness):
    for type in list(types.keys()):
        for type_idx in range(types[type]):
            dirname = ''.join([base_path, type, '/', type, '_', str(type_idx+1), '/'])
            mask_dir = dirname + 'mask'
            if not os.path.exists(mask_dir):
                os.mkdir(mask_dir)

            json_dir = dirname + 'json'
            if not os.path.exists(json_dir):
                os.mkdir(json_dir)


            all_files = os.listdir(json_dir)
            json_files = list(filter(lambda x: '.json' in x, all_files))


            for i in range(len(json_files)):
                json_path = json_dir + '/' + json_files[i]
                print(json_path)
                save_path = mask_dir + '/' + json_files[i].replace('.json', '.png')
                # polygons_json2mask(json_path, save_path)
                lines_json2mask(json_path, thickness = thickness, save_path = save_path)
                print(json_path, '->', save_path)


if __name__ == '__main__':

    base_path = "./"
    types = {'circle':4, 'curve':5, 'straight':4}
    
    generate_mask(base_path, types, 20)
    


