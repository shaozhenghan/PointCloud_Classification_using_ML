# -*- coding: utf-8 -*-

########################################################
### estimate mutual information (dependency) between ###
### feature vectors with different search radius for ###
### local feature estimation and target              ###
########################################################

import numpy as np 
from sklearn import preprocessing
from sklearn.feature_selection import mutual_info_classif
import os
import matplotlib.pyplot as plt

base_path = "/media/shao/TOSHIBA EXT/data_object_velodyne/feature_matrix_with_label/train"

# calculate and save statistic info about every label #
def get_samples_num ():
    read_path = os.path.join(base_path, "data")
    filelist = os.listdir(read_path)
    for file in filelist:
        read_file = os.path.join(read_path, file)
        if os.path.isdir(read_file):
            continue
        dataset = np.loadtxt(read_file)
        target = dataset[:, -1]
        sumples_num = np.zeros([1, 5], dtype=float)
        for i in range(0, 5):
            idx = np.argwhere(target == i)
            idx = np.array(idx)
            for _ in idx:
                sumples_num[0, i] += 1
        write_path = os.path.join(base_path, "samples_number")
        write_file = os.path.join(write_path, file)
        np.savetxt(os.path.splitext(write_file)[0]+'_samples_num.txt', sumples_num)
            


# estimate mutual info and save #
def estimate_mutual_info ():
    read_path = os.path.join(base_path, "data")
    filelist = os.listdir(read_path)
    for file in filelist:
        read_file = os.path.join(read_path, file)
        if os.path.isdir(read_file):
            continue
        dataset = np.loadtxt(read_file)
        target = dataset[:, -1]
        # feature scaling
        scaler = preprocessing.StandardScaler(copy=False)
        scaler.fit_transform(dataset[:, :-1])
        # mutual_info of local features
        data_local = dataset[:, 10:-1]
        mi = mutual_info_classif(data_local, target, 'auto', copy='true', n_neighbors=3)
        write_path = os.path.join(base_path, "mutual_info_local")
        write_file = os.path.join(write_path, file)
        np.savetxt(os.path.splitext(write_file)[0]+'_m.txt', mi)
        # mutual_info of global features
        data_global = dataset[:, 0:10]
        mi = mutual_info_classif(data_global, target, 'auto', copy='true', n_neighbors=3)
        write_path = os.path.join(base_path, "mutual_info_global")
        write_file = os.path.join(write_path, file)
        np.savetxt(os.path.splitext(write_file)[0]+'_m.txt', mi)




# visualize relationship between mutual_info of local features and search radius using bar graph #
def visualize_mutual_info_local ():
    read_path = os.path.join(base_path, "mutual_info_local")
    filelist = os.listdir(read_path)
    plt.figure(1)
    feature_name = ['lalonde1','lalonde2','lalonde3','meanIG']
    color = ['#0072BC', '#ED1C24', '#0C1234', '#7200AD', '#0702CD', '#DE08FF', '#0088AC', '#AF2C24']
    x = list(range(len(feature_name)))
    total_width = 0.8
    num_bar = len(filelist)
    width = total_width / num_bar 
    for j in range(0, 8):
        read_file = os.path.join(read_path, filelist[j])
        if os.path.isdir(read_file):
            continue
        mutual_info = np.loadtxt(read_file)
        np.squeeze(mutual_info)
        data = mutual_info[0:4]
        plt.bar(x, data, width=width, color=color[j], 
                label=os.path.splitext(filelist[j])[0], 
                tick_label = feature_name, edgecolor='white')
        for i in range(len(x)):
            x[i] = x[i] + width
    
    plt.rcParams['font.size'] = 18
    plt.rcParams['figure.figsize'] = (1, 1)
    plt.xlabel('Merkmale') 
    plt.ylabel('Gegenseitige Information')  
    plt.legend(loc='upper right')
    plt.title('Beziehung zwischen gegenseitiger Info und Suchradius')  
    # plt.savefig('mutual_info_and_search_radius') # savefig must be called before plt.show()
    plt.show()



# visualize mutual_info of global features using bar graph #
def visualize_mutual_info_global ():
    read_path = os.path.join(base_path, "mutual_info_global")
    filelist = os.listdir(read_path)
    plt.figure(2)
    feature_name = ['Laenge','Breite','Hoehe','Imax', 'Imean', 'Ivar', 'MI', 'e1', 'e2', 'e3']
    read_file = os.path.join(read_path, filelist[3])
    if os.path.isdir(read_file):
        return
    mutual_info = np.loadtxt(read_file)
    np.squeeze(mutual_info)
    plt.bar(range(len(mutual_info)), mutual_info, color='blue', 
            width=0.4, tick_label = feature_name, edgecolor='white')
    
    plt.rcParams['font.size'] = 18
    plt.rcParams['figure.figsize'] = (1, 1)
    plt.xlabel('Merkmale') 
    plt.ylabel('Gegenseitige Information')  
    plt.legend(loc='upper right')
    plt.title('Gegenseitige Info globaler Merkmale')
    # plt.savefig('mutual_info_and_search_radius') # savefig must be called before plt.show()
    plt.show()



# visualize relationship between mutual_info of FPFH and search radius #
def visualize_mutual_info_FPFH ():
    read_path = os.path.join(base_path, "mutual_info_local")
    filelist = os.listdir(read_path)
    plt.figure(3) 
    for file in filelist:
        read_file = os.path.join(read_path, file)
        if os.path.isdir(read_file):
            continue
        mutual_info = np.loadtxt(read_file)
        np.squeeze(mutual_info)
        data = mutual_info[4:]
        plt.plot(range(0, 33), data, label=os.path.splitext(file)[0])
    
    plt.rcParams['font.size'] = 18
    plt.rcParams['figure.figsize'] = (1, 1)
    plt.xlabel('FPFH') 
    plt.ylabel('Gegenseitige Information')  
    plt.legend(loc='upper right')
    plt.title('Beziehung zwischen gegenseitiger Info und Suchradius')  
    # plt.savefig('mutual_info_and_search_radius') # savefig must be called before plt.show()
    plt.show()



estimate_mutual_info()
get_samples_num()
visualize_mutual_info_local()
visualize_mutual_info_global()
visualize_mutual_info_FPFH()
