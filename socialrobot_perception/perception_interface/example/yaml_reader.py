#!/usr/bin/env python
import rospkg


config_path = rospkg.RosPack().get_path('perception_interface') + '/config/'

def read_dataset():
    dataset = []
    fname = config_path + 'dataset.txt'
    f = open(fname, "r")
    data = f.readlines()
    
    for x in data:
        dataset.append(x[:-1])
    return dataset

def read_camera_matrix():
    import yaml
    
    fname = config_path + "camera_matrix.yaml"
    with open(fname) as f:
        data = yaml.load(f)

    return data['camera_matrix']

dataset = read_dataset()
mat = read_camera_matrix()

print(dataset)
print(mat)

import tf
trans = tf.transformations.translation_from_matrix(mat) 
rot = tf.transformations.quaternion_from_matrix(mat)
print(trans)
print(rot)