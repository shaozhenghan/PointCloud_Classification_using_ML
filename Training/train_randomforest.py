# encoding=utf-8

#######################
# train random forest #
#######################

from sklearn.ensemble import RandomForestClassifier
import numpy as np 
from sklearn.model_selection import GridSearchCV
from sklearn.metrics import fbeta_score, make_scorer

# load data for training
feature_matrix = np.loadtxt('/media/shao/TOSHIBA EXT/data_object_velodyne/feature_matrix_with_label/train/data/r_0.16.txt')
print('the shape of the loaded feature matrix is ', feature_matrix.shape)
data = feature_matrix[:, :-1]
target = feature_matrix[:, -1]

# grid search for tuning hyperparameters

# coarse tune
# params = {
#             'max_depth':[6, 8, 10, 12, 15, 18, 20],
#             'n_estimators':[10, 20, 30, 40, 50, 60, 70],
#             'max_features':[15, 20, 25, 30, 35, 40]
#         }

# fine tune
params = {
            'max_depth':[10, 11, 12, 13, 14, 15], # optimal 12
            'n_estimators':[54, 55, 56, 57, 58],  # optimal 56
            'max_features':[18, 19, 20, 21, 22]   # optimal 20
        }
rfc = RandomForestClassifier(   
                                # max_depth=10, 
                                random_state=0, 
                                # n_estimators=10,
                                # max_features=30, 
                                oob_score=True,
                                bootstrap=True,
                                class_weight='balanced'
                            )
fone_scorer = make_scorer(fbeta_score, beta=1, average='weighted')
clf = GridSearchCV (
                        rfc,
                        params,
                        scoring=fone_scorer,
                        n_jobs=4,
                        cv=5,
                        iid=True,
                        refit=True
                    )
clf.fit(data, target)

# print important info
# print(rfc.feature_importances_)
# print(rfc.oob_score_)
print('clf.cv_results_', clf.cv_results_)
print('clf.best_params_', clf.best_params_)
print('clf.best_estimator_', clf.best_estimator_)
print('clf.grid_scores_', clf.grid_scores_) 
print('best score', clf.grid_scores_[clf.best_index_])

# save the trained model
from sklearn.externals import joblib
joblib.dump(clf, 'rf.pkl')