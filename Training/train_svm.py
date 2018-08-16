# encoding=utf-8

#############
# train svm #
#############

import numpy as np 
from sklearn import svm
from sklearn import preprocessing
from sklearn.model_selection import GridSearchCV

# load data for training
feature_matrix = np.loadtxt('/media/shao/TOSHIBA EXT/data_object_velodyne/feature_matrix_with_label/train/data/r_0.16.txt')
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

# train svm using grid search
params = {'kernel':['rbf', 'linear'], 'C':[0.001, 0.01, 0.1, 1], 'gamma':[0.001, 0.01, 0.1, 1]}
# params = {'kernel':['rbf'], 'C':[1], 'gamma':[1]}
svc = svm.SVC()
clf = GridSearchCV(
    svc, params, scoring='accuracy', 
    n_jobs=5, cv=5, return_train_score=False)
clf.fit(data, target)

# print important info
print('clf.cv_results_', clf.cv_results_)
print('clf.best_params_', clf.best_params_)
print('clf.best_estimator_', clf.best_estimator_)
print('clf.grid_scores_', clf.grid_scores_) 
print('best score', clf.grid_scores_[clf.best_index_])

# save the trained model
from sklearn.externals import joblib
joblib.dump(clf, 'svm.pkl')