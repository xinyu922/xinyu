import pandas as pd
import re
from tf.transformations import euler_from_quaternion, quaternion_from_euler

filename = 'waypoint.txt'
outfile = 'waypoint_ang.csv'

with open(filename, 'r') as f:
    rows = f.readlines()
f_out = open(outfile, 'w')

result = [(rows[9+i],rows[10+i],rows[11+i],rows[13+i],rows[14+i],rows[15+i],rows[16+i]) for i in range(0,len(rows),19)]   

def find_num(x):
    return re.findall('[-\d.]+',x)[0]  
data = [list(map(find_num,(x,y,z,q1,q2,q3,q4))) for x,y,z,q1,q2,q3,q4 in result]

data = [(x,y,z,q1,q2,q3,q4) for x,y,z,q1,q2,q3,q4 in data]
df = pd.DataFrame(data, columns = ['x', 'y', 'z', 'q1', 'q2', 'q3', 'q4'])

df.x = df.x.apply(float)
df.y = df.y.apply(float)
df.z = df.z.apply(float)
df.q1 = df.q1.apply(float)
df.q2 = df.q2.apply(float)
df.q3 = df.q3.apply(float)
df.q4 = df.q4.apply(float)

# df.to_csv(outfile, index=False)

f_out.write('x,y,z,yaw,roll,pitch\n')
# print(df)

for i in range(len(df.x)):
    tempdata=[df.q1[i], df.q2[i], df.q3[i], df.q4[i]]
    (roll, pitch, yaw) = euler_from_quaternion(tempdata)
    f_out.write('%.12f, %.12f, %.12f, %.12f, %.12f, %.12f\n' %
                        (
                         df.x[i],
                         df.y[i],
                         df.z[i],
                         yaw,
                         roll,
                         pitch
                         )
            )

