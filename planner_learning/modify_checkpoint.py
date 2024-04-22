#!/usr/bin/env python3

import argparse
import os
import sys
sys.path.append("./src/PlannerLearning/models")
import time
import numpy as np
import tensorflow as tf

from plan_learner import PlanLearner

from config.settings import create_settings


def main():
    parser = argparse.ArgumentParser(description='Train Planning Network')
    parser.add_argument('--settings_file',
                        help='Path to settings yaml', required=True)

    args = parser.parse_args()
    settings_filepath = args.settings_file
    settings = create_settings(settings_filepath, mode='train')

    # set modify_ckpt_output_layer=True to modify last layer dimension
    learner = PlanLearner(settings=settings, modify_ckpt_output_layer=False)

    # input size (batch size, modes, total imu + img dim)
    input_x = np.ones((8, 3, 60))
    out = learner.network._plan_branch(input_x)
    out_array = out.numpy()
    # original ckpt out dimension is (8, 3, 31)
    # out dimension with yaw is (8, 3, 41)
    print(out_array.shape)
    print(out_array)
    np.save("yaw_resume_ckpt", out_array)

    # save the new checkpoint
    ckpt_manager = tf.train.CheckpointManager(learner.ckpt, "models/", max_to_keep=20)
    save_path = ckpt_manager.save()
    print("Saved checkpoint for epoch {}: {}".format(int(learner.ckpt.step), save_path))

if __name__ == "__main__":
    main()
