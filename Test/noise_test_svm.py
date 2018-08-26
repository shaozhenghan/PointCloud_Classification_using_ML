# encoding=utf-8

#####################
# basic_test_svm.py #
#####################

from sklearn import preprocessing
from sklearn.externals import joblib
import numpy as np  
from sklearn import metrics
import os
import sys

# get model file path
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)
MODEL_DIR = ROOT_DIR + '/Training/svm.pkl'

# load data for testing
feature_matrix = np.loadtxt('/media/shao/TOSHIBA EXT/data_object_velodyne/Daten/test/robust_test/noise/noi_0.09.txt')
print('the shape of the loaded feature matrix is ', feature_matrix.shape)

# feature scaling and normalizing
np.random.shuffle(feature_matrix)
data = feature_matrix[:, :-1]
target = feature_matrix[:, -1]
# scaler = preprocessing.MaxAbsScaler(copy=False)
scaler = preprocessing.StandardScaler(copy=False)
scaler.fit_transform(data)
normalizer = preprocessing.Normalizer(norm='l2', copy=False)
normalizer.fit_transform(data)

# load the trained model
clf = joblib.load(MODEL_DIR)

# prediction / test
y_pred = clf.predict(data)
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
print('f1 score = ', precision)
y_pred = clf.decision_function(data)
hinge_loss = metrics.hinge_loss(target, y_pred, labels=np.array([0,1,2,3,4]))
print('hinge loss = ', hinge_loss)