import pandas as pd
import re
from tf.transformations import euler_from_quaternion, quaternion_from_euler

filename = 'waypoint.txt'
outfile = 'waypoint.csv'

f = open(outfile, 'w')

csv_data=pd.read_csv(open(filename))
n = 0
f.write('x,y,z,yaw,roll,pitch\n')
for i in range(len(csv_data.x)):
    tempdata=[csv_data.q1[i],csv_data.q2[i],csv_data.q3[i],csv_data.q4[i]]
    (roll, pitch, yaw) = euler_from_quaternion (tempdata)
    f.write('%.12f, %.12f, %.12f, %.12f, %.12f, %.12f\n' %
                        (
                         csv_data.x[i],
                         csv_data.y[i],
                         csv_data.z[i],
                         yaw,
                         roll,
                         pitch
                         )
            )
    n += 1
print('change ' + str(n) + ' Quaternions to angle: ' + outfile)

f.close()


    


# with open(filename, 'r') as f:
#     rows = f.readlines()

# result = rows[1:]
# print(result)

# print(len(result))
# num_list=result.split(',')
# split_result = [re.split(r'[,]',row)[:10] for row in result]
# print(num_list)

# for i in len(result):
#     orientation_list = [result[i].]
#     (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
