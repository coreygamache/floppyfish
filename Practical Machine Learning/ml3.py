import pandas as pd
import quandl
import math

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

# create label column and shift forecast column up number of columns specified by forecast_out
df['label'] = df[forecast_col].shift(-forecast_out)

print(df.head())
