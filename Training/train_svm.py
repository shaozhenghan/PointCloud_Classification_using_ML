# encoding=utf-8

#############
# train svm #
#############

import numpy as np 
from sklearn import svm
from sklearn import preprocessing
from sklearn.model_selection import GridSearchCV
from sklearn.metrics import fbeta_score, make_scorer

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
fone_scorer = make_scorer(fbeta_score, beta=1, average='weighted')
# params = {  'kernel':['rbf', 'linear'], 
#             'C':[0.5, 1.0, 1.3, 1.5, 2.0, 2.1, 2.2, 2.3, 2.4], 
#             'gamma':[0.5, 1.0, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0]
#             }
params = {'kernel':['rbf'], 'C':[2.3], 'gamma':[1.4]}
svc = svm.SVC()
clf = GridSearchCV(
    svc, params, scoring=fone_scorer,
    n_jobs=5, cv=5, return_train_score=False, iid=True)
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