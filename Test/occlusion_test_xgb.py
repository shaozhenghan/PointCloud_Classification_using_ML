# encoding=utf-8

#############################
# occlusion_test_xgboost.py #
#############################

from sklearn.externals import joblib
import numpy as np  
from sklearn import metrics
import os
import sys

# get model file path
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)
MODEL_DIR = ROOT_DIR + '/Training/xgb.pkl'

# load data for testing
feature_matrix = np.loadtxt('/media/shao/TOSHIBA EXT/data_object_velodyne/Daten/test/robust_test/occlusion/op_90.txt')
data = feature_matrix[:, :-1]
target = feature_matrix[:, -1]

# load the trained model
xgbc = joblib.load(MODEL_DIR)

# prediction / test
y_pred = xgbc.predict(data)
score = metrics.accuracy_score(target, y_pred)
print('accuracy score = ', score)
conf_matrix = metrics.confusion_matrix(target, y_pred, [0,1,2,3,4])
print('confusion matrix = ')
print(conf_matrix)
recall = metrics.recall_score(target, y_pred, average='weighted')
print('recall score = ', recall)
precision = metrics.precision_score(target, y_pred, average='weighted')
print('precision score = ', precision)
f1 = metrics.f1_score(target, y_pred, average='weighted')
print('f1 score = ', f1)
prob = xgbc.predict_proba(data)
log_loss = metrics.log_loss(target, prob, labels=np.array([0,1,2,3,4]))
print('log loss = ', log_loss)