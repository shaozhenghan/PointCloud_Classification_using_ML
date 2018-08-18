# encoding=utf-8

#######################
# train random forest #
#######################

from sklearn.ensemble import RandomForestClassifier
import numpy as np 

# load data for training
feature_matrix = np.loadtxt('/media/shao/TOSHIBA EXT/data_object_velodyne/feature_matrix_with_label/train/data/r_0.16.txt')
print('the shape of the loaded feature matrix is ', feature_matrix.shape)
data = feature_matrix[:, :-1]
target = feature_matrix[:, -1]

# set random forest params
rfc = RandomForestClassifier(   max_depth=10, 
                                random_state=0, 
                                n_estimators=10,
                                max_features=30, 
                                oob_score=True,
                                n_jobs=-1,
                                bootstrap=True,
                                class_weight='balanced'
                            )

# train random forest 
rfc.fit(data, target)

# print important info
print(rfc.feature_importances_)
print(rfc.oob_score_)

# save the trained model
from sklearn.externals import joblib
joblib.dump(rfc, 'rf.pkl')