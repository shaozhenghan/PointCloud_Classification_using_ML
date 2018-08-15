# -*- coding: utf-8 -*-
###################################
# Get all files' name in a folder #
###################################

import os

# use Cython so that C++ can call Python function

cdef public get_file_name(path):
    filelist = os.listdir(path)
    return filelist