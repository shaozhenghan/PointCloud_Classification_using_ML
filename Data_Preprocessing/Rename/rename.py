# -*- coding: utf-8 -*-
import os

def rename():
    i=0
    # 下面两行根据需要改
    object_class = "van"
    path="/media/shao/TOSHIBA EXT/data_object_velodyne/testing/test_data_original_1/van"

    # 该文件夹下所有的文件（包括文件夹）
    filelist = os.listdir(path)                
    # print(filelist)
    # 遍历所有文件
    for files in filelist:     
        if object_class in filelist[0]:
            break
        # 原来的文件路径
        Olddir=os.path.join(path,files) 
        # 如果是文件夹则跳过                         
        if os.path.isdir(Olddir):                 
            continue
        # 旧文件名
        filename=os.path.splitext(files)[0]      
        # 文件扩展名，需要保留
        filetype=os.path.splitext(files)[1]   
        # 新的文件路径; object_class+str(i)+filetype 为新文件名
        Newdir=os.path.join(path, object_class+str(i)+filetype)  
        # 重命名
        os.rename(Olddir,Newdir)                   
        i=i+1

rename()
