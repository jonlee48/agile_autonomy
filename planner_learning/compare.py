#!/usr/bin/env python3

import numpy as np

out_50 = np.load("yaw_predict.npy")
out_51 = np.load("yaw_resume_ckpt.npy")

print("Close?")
print(np.allclose(out_50, out_51))