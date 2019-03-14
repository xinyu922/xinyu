import pandas as pd
import re

filename = 'echo_mb.txt'
outfile = 'movebase_time.csv'

with open(filename, 'r') as f:
    rows = f.readlines()

result = [(rows[3+i],rows[4+i],rows[12+i]) for i in range(0,len(rows),17)]   

def find_num(x):
    return re.findall('[\d.]+',x)[0]  
data = [list(map(find_num,(sec,nsec,status))) for sec,nsec,status in result]
# print(data)

data = [(sec + '.' + nsec,status) for sec,nsec,status in data]
df = pd.DataFrame(data, columns = ['time','status'])

df.time = df.time.apply(float)
df.status = df.status.apply(int)

df.to_csv(outfile, index=False)