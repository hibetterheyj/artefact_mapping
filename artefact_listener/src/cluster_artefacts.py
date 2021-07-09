"""
eps = 1.0
min_samplesint = 1 (seems invalid in latest version)
in sklearn: 0.24.2
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from sklearn.cluster import dbscan

from queue import Queue

show_detected_instance = False

# data csv
# person | stop | backpack | umbrella | bottle | clock |
class_label = 'stop'
experiment = 'final_challenge'  # final_challenge | zurich_garage
instances_df = pd.read_csv('./' + experiment + '_' + class_label + '.csv')
x = instances_df['X'].values.reshape((-1, 1))
y = instances_df['Y'].values.reshape((-1, 1))
z = instances_df['Z'].values.reshape((-1, 1))
instance_xy = np.hstack((x, y))
# print(instance_xy)

# Scikit-Learn DBSCAN
# preds = dbscan(blobs, x, 5)[1]
preds = dbscan(instance_xy, 1.0, 1)[1]
# print(len(preds))
print(instance_xy)
print(preds)
cluster_num = max(preds).item() + 1

# start processing
new_list = []
csv_name = ['X', 'Y', 'Z', 'Class']
for i in range(cluster_num):
    idxs = np.where(preds == i)
    cluster_mem = idxs[0]
    print("Found closed detections {} from No.{} cluster".format(cluster_mem, i))
    num_in_cluster = len(cluster_mem)
    if num_in_cluster > 1:
        idx = cluster_mem[0]
        x_update = x[idx][0]
        y_update = y[idx][0]
        z_update = z[idx][0]
        for i in range(1, num_in_cluster):
            x_update += x[cluster_mem[i]][0]
            y_update += y[cluster_mem[i]][0]
            z_update += z[cluster_mem[i]][0]
        x_update = x_update / num_in_cluster
        y_update = y_update / num_in_cluster
        z_update = z_update / num_in_cluster
    else:
        idx = cluster_mem[0]
        x_update = x[idx][0]
        y_update = y[idx][0]
        z_update = z[idx][0]
    update_artefact = [x_update, y_update, z_update, class_label]
    new_list.append(update_artefact)
new_pd = pd.DataFrame(
    columns=csv_name, data=new_list)
new_pd.to_csv(class_label + '_update.csv', encoding='utf-8', index=False)

#Plot
if show_detected_instance:
    plt.style.use("bmh")
    fig, ax = plt.subplots(1, 2, dpi=200)

    dbscan_blob = np.append(instance_xy, preds.reshape(-1, 1), axis=1)
    pd.DataFrame(dbscan_blob).plot(x=1, y=0, kind="scatter", c=2, colorbar=False,
                                ax=ax[1], title="Scikit-Learn DBSCAN (eps=0.8, min_points=1)", marker="+", colormap="tab20b")

    # Test Data
    pd.DataFrame(instance_xy).plot(x=1, y=0, kind="scatter", ax=ax[0], alpha=0.5, figsize=(
        15, 6), title="Test Data", marker="+", c="#e377c0")

    plt.show()



# reference
# https://towardsdatascience.com/dbscan-with-python-743162371dca
# https://github.com/mpHarm88/projects/blob/master/dbscan/notebooks/dbscan.ipynb
# https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html

