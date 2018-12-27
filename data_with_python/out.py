import pandas as pd
import re

filename = 'output_z.txt'
outfile = 'out.csv'

with open(filename, 'r') as f:
    rows = f.readlines()

result = [(rows[3+i],rows[4+i],rows[10+i],rows[11+i],rows[12+i]) for i in range(0,len(rows),21)]   

def find_num(x):
    return re.findall('[\d.]+',x)[0]  
data = [list(map(find_num,(sec,nsec,x,y,z))) for sec,nsec,x,y,z in result]

data = [(sec + '.' + nsec,x,y,z) for sec,nsec,x,y,z in data]
df = pd.DataFrame(data, columns = ['time', 'x', 'y', 'z'])

df.time = df.time.apply(float)
df.x = df.x.apply(float)
df.y = df.y.apply(float)
df.z = df.z.apply(float)

df.to_csv(outfile, index=False)
