import bvh
from scipy.spatial.transform import Rotation
import os
import numpy as np
import sys

FILE_PATH = os.path.dirname(os.path.realpath(__file__))

sys.path.append(FILE_PATH)

from repair_tpose import bake_tpose

pfnn_to_meters = 5.6444
if __name__ == "__main__":
  # base_dir = os.path.join(FILE_PATH, "data/pfnn_raw")
  # output_dir = os.path.join(FILE_PATH, "data/pfnn_processed")
  # os.makedirs(output_dir, exist_ok=True)
  # for file in os.listdir(base_dir):
  #   if file.endswith(".bvh"):
  #     data = bvh.load(os.path.join(base_dir, file))
  #     data['offsets'] = data['offsets'] * pfnn_to_meters * 0.01
  #     data['positions'] = data['positions'] * pfnn_to_meters * 0.01
  #     data['rotations'] = data['rotations'][::2]
  #     data['positions'] = data['positions'][::2]
  #     bvh.save(os.path.join(output_dir, file), data)
  
  
  bake_tpose(
    os.path.join(FILE_PATH, "data/pfnn_tpose.bvh"), 
    os.path.join(FILE_PATH, "data/pfnn_tpose_baked.bvh"))