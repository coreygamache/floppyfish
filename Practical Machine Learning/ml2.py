import pandas as pd
import quandl

df = quandl.get('WIKI/GOOGL')

# pare dataframe down to only the information we care about
df = df[['Adj. Open','Adj. High','Adj. Low','Adj. Close','Adj. Volume']]

# create high/low percent column
df['HL_PCT'] = (df['Adj. High'] - df['Adj. Close']) / df['Adj. Close'] * 100

# create percent change column
df['PCT_change'] = (df['Adj. Close'] - df['Adj. Open']) / df['Adj. Open'] * 100

# pare dataframe down again to only the information we really care about
df = df[['Adj. Close','HL_PCT','PCT_change','Adj. Volume']]

print(df.head())
