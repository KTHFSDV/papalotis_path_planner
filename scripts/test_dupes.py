import numpy as np
import pandas as pd


cones_left = np.array([[0, 0], [1, 1], [2, 2], [3, 3], [4, 4], [5, 5], [6, 6], [7, 7], [8, 8], [9, 9]])
cones_right = np.array([[0, 0], [1, 1], [2, 2], [3, 3]])

combined_cones = np.concatenate((cones_left, cones_right), axis=0)
if combined_cones.any():
    df = pd.DataFrame(combined_cones, columns=['x', 'y'])
            
    # Find duplicates
    duplicate_cones = df[df.duplicated()]
    
    # Convert duplicates back to numpy array
    duplicate_list = duplicate_cones.to_numpy()
    
    cones_left_df = pd.DataFrame(cones_left, columns=['x', 'y'])
    difference_df = pd.concat([duplicate_cones, cones_left_df]).drop_duplicates(keep=False).to_numpy()