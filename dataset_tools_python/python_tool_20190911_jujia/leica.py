import pandas as pd

pd.set_option('display.float_format', lambda x: '%.3f' % x)

labels = ['name', 'x', 'y', 'h', 'data', 'time']
csv_data = pd.read_csv("20191112-001.txt", header=None, skiprows=[0, 1, 2, 3], names=labels)
csv_data['#timestamp'] = csv_data['data'] + ' ' + csv_data['time']
order = ['#timestamp', 'x', 'y', 'h']
csv_data = csv_data[order]
csv_data['#timestamp'] = pd.to_datetime(csv_data['#timestamp'])
csv_data['#timestamp'] = [pd.Timestamp.timestamp(csv_data['#timestamp'][i]) for i in range(len(csv_data['#timestamp']))]

csv_data.to_csv('leica.csv', index=False, float_format='%.4f')

