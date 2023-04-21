#!/usr/bin/env python3

import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)

import argparse
import os
from pathlib import Path
import csv
import time

import logging
logging.basicConfig(level=logging.INFO)

from ruamel.yaml import YAML

from stable_baselines.ppo2 import PPO2

from multirotors_landing_rl.common.common_defs import TESTING_ENVS_IDS, ENVIRONMENTS_ID
from multirotors_landing_rl.envs import vec_env_landing as wrapper
from multirotors_landing_rl.testing.testing_utils import run_test

from mlgym import QuadrotorEnvVulkan


def parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-w', '--weight', type=str, required=True,
                        help='Trained weight path')
    parser.add_argument('--rollouts', type=int, default=15,
                        help="Number of rollouts at test time. Defaults to 15")
    parser.add_argument('--n-envs-training', type=int, default=2,
                        help="Number of environments the policy was trained on")
    return parser


def main():
    # Read arguments
    args = parser().parse_args()

    # Check if there are conflicting parameters
    if args.weight == '':
        logging.error("Please specify a valid path for the weights")
        exit(-1)
    else:
        args.weight = Path(args.weight).expanduser()

    # Read config file
    ml_path = os.environ["ML_PATH"]
    if ml_path == "":
        logging.fatal("ML_PATH has not been declared. Abort")
        exit(-1)

    env_name = "quad_landing_test"
    cfg_file = Path(ml_path).joinpath(f"multirotors_landing_lib/config/{env_name}.yaml")
    if not cfg_file.exists():
        logging.fatal(f"Configuration file {cfg_file} does not exist. Abort")
        exit(-1)

    env_cfg = YAML().load(open(cfg_file, 'r'))

    img_size = (env_cfg["training_env"]["img_width"],
                env_cfg["training_env"]["img_height"],
                env_cfg["training_env"]["img_channels"])

    # Set up the actual environment
    env = wrapper.VecQuadrotorEnvVulkan(QuadrotorEnvVulkan(env_name),
                image_size=img_size, verbose=env_cfg["common"]["verbose"])


    # Test the policy and the random actions in sequences
    timestamp = time.strftime("%Y-%m-%d-%H-%M-%S")
    output_folder = Path(os.getenv("ML_PATH")).joinpath(f"experiments/tests/{timestamp}")
    if not output_folder.exists():
        logging.warn(f"Creating output dir at {output_folder}")
        output_folder.mkdir(parents=True)

    model = PPO2.load(args.weight, env=env)
    success_policy = run_test(env, model, args.rollouts, output_folder)
    del model

    # Write summary file
    summary_writer = csv.writer(open(os.path.join(output_folder, 'results_summary.csv'), 'w+'))
    summary_writer.writerow(["Env", "Policy"])
    for env_id in TESTING_ENVS_IDS:
        env_model_name = ENVIRONMENTS_ID[env_id]
        summary_writer.writerow([env_model_name, success_policy[env_model_name]])


if __name__ == "__main__":
    main()
