import numpy as np
import pandas as pd
import os

collected_data = r"C:\Git\udacity\fcnd\estimation\config\log\Graph2.txt"
df = pd.read_csv(collected_data)
df.iloc[:, 1].std()