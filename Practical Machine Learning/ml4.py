import pandas as pd
import quandl, math
import numpy as np
from sklearn import preprocessing, model_selection, svm
from sklearn.linear_model import LinearRegression

df = quandl.get('WIKI/GOOGL')

# pare dataframe down to only the information we care about
df = df[['Adj. Open','Adj. High','Adj. Low','Adj. Close','Adj. Volume']]

# create high/low percent column
df['HL_PCT'] = (df['Adj. High'] - df['Adj. Close']) / df['Adj. Close'] * 100

# create percent change column
df['PCT_change'] = (df['Adj. Close'] - df['Adj. Open']) / df['Adj. Open'] * 100

# pare dataframe down again to only the information we really care about
df = df[['Adj. Close','HL_PCT','PCT_change','Adj. Volume']]

# create new variable to define what we will be predicting
forecast_col = 'Adj. Close'

# fill any missing data with -99,999 so it can be read by machine learning algorithm but will be treated as an outlier so as not to impact outcome
df.fillna(-999999, inplace=True)

# create new variable to define percentage of the dataframe to predict out into the future (1% here)
forecast_out = int(math.ceil(0.01*len(df)))

print(forecast_out)

# create label column and shift forecast column up number of columns specified by forecast_out
df['label'] = df[forecast_col].shift(-forecast_out)

# drop null rows
df.dropna(inplace=True)

# define numpy array containing all features (all columns except 'label')
X = np.array(df.drop(['label'],1))

# define numpy array containing all labels (only 'label')
#y = np.array(df['label'])

# scale feature data to look like standard, normally distributed data: Gaussian with zero mean and unit variance
X = preprocessing.scale(X)

# define numpy array containing all labels (only 'label')
y = np.array(df['label'])

# takes feature and label data, shuffles it up, then returns test_size percent of it (20% in this case) as test data and the rest as training data, split into four seperate arrays
X_train, X_test, y_train, y_test = model_selection.train_test_split(X, y, test_size=0.2)

# set classifier type, try testing with svm.SVR or svm.SVR(kernel='poly'), or speed up training by adding argument n_jobs=(some integer number of threads to run or -1 to run all possible threads)
clf = LinearRegression(n_jobs=-1)

# train classifier using training data arrays created earlier
clf.fit(X_train, y_train)

# score accuracy of the classifier using the test data arrays created earlier and store result in new variable
accuracy = clf.score(X_test, y_test)

print(accuracy)
