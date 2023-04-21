#!/usr/bin/env python3

import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)

import argparse
import os
from pathlib import Path
import numpy as np

import logging
logging.basicConfig(level=logging.INFO)

from ruamel.yaml import YAML
import tensorflow as tf

from stable_baselines.ppo2 import PPO2
from stable_baselines.common.callbacks import CheckpointCallback
from stable_baselines import logger

from multirotors_landing_rl.envs import vec_env_landing as wrapper
from multirotors_landing_rl.policies.policies import CnnAsymmetricalPolicy
from multirotors_landing_rl.common.utils import ConfigurationSaver

from mlgym import QuadrotorEnvVulkan

from find_best_policy_weights import find_best_weigths


def parser():
    parser = argparse.ArgumentParser()

    parser.add_argument('--keep-train', action='store_true',
                        help="Whether to keep training the input model")
    parser.add_argument('--save_dir', type=str,
                        default=os.path.dirname(os.path.realpath(__file__)),
                        help="Directory where to save the checkpoints and "
                             "training metrics")
    parser.add_argument('--seed', type=int, default=0, help="Random seed")
    parser.add_argument('--decay-lr', action='store_true',
                        help="Whether to use decaying learning rate")
    parser.add_argument('--lr', type=float, default=1e-3, help="Learning Rate (only used to keep training model)")
    parser.add_argument('--training-steps', type=int, default=5000000, help="Number of training steps")
    parser.add_argument('-w', '--weight', type=str, default='',
                        help='trained weight path')
    
    return parser


def configure_random_seed(seed, env=None):
    if env is not None:
        env.seed(seed)
    np.random.seed(seed)
    tf.set_random_seed(seed)


def main():
    # Read arguments
    args = parser().parse_args()

    # Check if there are conflicting parameters
    if args.weight == '' and args.keep_train:
        logging.error("Please specify a valid path for the weights " \
            "to continue training")
        exit(-1)

    # Read config file
    ml_path = os.environ["ML_PATH"]
    if ml_path == "":
        logging.fatal("ML_PATH has not been declared. Abort")
        exit(-1)

    env_name = "quad_landing"
    cfg_file = Path(ml_path).joinpath(f"multirotors_landing_lib/config/{env_name}.yaml")
    if not cfg_file.exists():
        logging.fatal(f"Configuration file {cfg_file} does not exist. Abort")
        exit(-1)

    env_cfg = YAML().load(open(cfg_file, 'r'))

    # Set environment and corresponding policy
    img_size = (env_cfg["training_env"]["img_width"],
                env_cfg["training_env"]["img_height"],
                env_cfg["training_env"]["img_channels"])

    # Set up the actual environment
    env = wrapper.VecQuadrotorEnvVulkan(QuadrotorEnvVulkan(env_name),
                image_size=img_size, verbose=env_cfg["common"]["verbose"])

    # Set parameters
    # FIXME Add other policies
    nminibatches = env_cfg["env"]["num_envs"] * 4
    # if not args.lstm:
    #     nminibatches *= 4

    try:
        use_dropout = env_cfg["training_env"]["use_dropout"]
        dropout_rate = env_cfg["training_env"]["dropout_rate"]
        assert 0. <= dropout_rate and dropout_rate <= 1.
    except KeyError:
        logging.warn("Dropout parameters not specified - not using it")
        use_dropout = False
        dropout_rate = 0.

    # set random seed
    configure_random_seed(args.seed, env=env)

    # Select policy
    policy = CnnAsymmetricalPolicy
    policy_kwargs = {"use_dropout": use_dropout, "dropout_rate": dropout_rate}

    # Logger for configuration
    log_dir = Path(ml_path).joinpath("experiments")
    if not log_dir.exists():
        logging.warn(f"Creating logging dir at {log_dir}")
        log_dir.mkdir(parents=True)

    saver = ConfigurationSaver(log_dir=log_dir, env_name=env_name)
    saver.log_arguments_and_params(args, policy.__name__, [env_cfg])

    # Utility for training 
    def linear_schedule(initial_value, min_value=1e-5):
        """
        Linear learning rate schedule.
        :param initial_value: (float or str)
        :param min_value: (float)
        :return: (function)
        """
        if isinstance(initial_value, str):
            initial_value = float(initial_value)

        def func(progress):
            """
            Progress will decrease from 1 (beginning) to 0
            :param progress: (float)
            :return: (float)
            """
            return max(progress * initial_value, min_value)

        return func

    if not args.keep_train:
        # Set up model
        learning_rate_fc = linear_schedule(args.lr) if args.decay_lr else args.lr
        model = PPO2(
            tensorboard_log=saver.data_dir,
            policy=policy,  # check activation function
            policy_kwargs=policy_kwargs,
            env=env,
            lam=0.95,
            gamma=0.99,  # lower 0.9 ~ 0.99
            n_steps=2000,
            ent_coef=0.001,
            learning_rate=learning_rate_fc,
            vf_coef=0.5,
            max_grad_norm=0.5,
            nminibatches=nminibatches,
            noptepochs=4,
            cliprange=0.2,
            verbose=1,
        )
    else:
        model = PPO2.load(args.weight, tensorboard_log=saver.data_dir)
        # Note! In the keep training case, we fix the learning rate.
        # Otherwise we keep original hyperparameters
        if not args.decay_lr:
            model.learning_rate = args.lr
        model.set_env(env)

    # Callbacks
    callabacks = [CheckpointCallback(
        save_freq=15000, save_path=saver.data_dir, name_prefix='quad_land'
        )]
    
    # Logger
    logger.configure(folder=str(saver.data_dir))

    # PPO run
    try:
        model.learn(total_timesteps=args.training_steps, callback=callabacks)
        model.save(saver.data_dir.joinpath('last_step_model.zip'))
    except KeyboardInterrupt:
        logging.warn("Store the model after keyboard interrupt")
        model.save(saver.data_dir.joinpath('interrupt_model.zip'))

    # Closing
    logging.info("Experiment done")

    # Inform the user which weights it should use
    best_weights_path = find_best_weigths(str(saver.data_dir), verbose=False)
    logging.warn(f"Best policy weights: {best_weights_path}")


if __name__ == "__main__":
    main()
