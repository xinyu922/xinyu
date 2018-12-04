
# coding: utf-8

import pandas as pd
import re
import sys


# user-defined
filename = '024_01_201810121457.txt'
outfile = '024_01_201810121457.csv'


condition = ['WY-1+6-3D','WY-IP7-3D']

with open(filename, 'r') as f:
    raw_rows = f.readlines()

first = raw_rows.index('\n')
second = raw_rows[first+1:].index('\n')

distance = second + 1
strip_tail = (len(raw_rows)-1)%distance
rows = raw_rows[2:-strip_tail]

temp = rows[:distance]

# end_index = None
# for i,e in enumerate(temp):
#     if e.startswith('Ron-output file'):
#         end_index = i
#         break
# # value_num = end_index - 1
# value_num = 4
value_num = distance - 1


result = [e for i,e in enumerate(rows) if (i%distance <= value_num) and (i%distance > 0)]
split_result = [re.split(r'[,()]',row)[:10] for row in result]
add_zero_result = [row[:5]  + row[6:] for row in split_result]

title = ['time', 'name'] + list('xyzabcd')
df = pd.DataFrame(add_zero_result, columns = title)

df_out = df[df.name.isin(condition)]   
df_out.to_csv(outfile, index=False)

