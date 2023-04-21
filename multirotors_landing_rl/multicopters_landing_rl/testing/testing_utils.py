#!/usr/bin/env python3

from pathlib import Path
import csv
import time
import numpy as np

import logging
logging.basicConfig(level=logging.INFO)

from multirotors_landing_rl.common.common_defs import \
    TESTING_ENVS_IDS, ENVIRONMENTS_ID, QUAD_POSIT_SIZE, DONE_REASONS


TIME_SLEEP_BETWEEN_RUNS = 0.5


def compute_total_distance(positions: np.ndarray) -> float:
    tot_dist = 0.
    for i in range(3, positions.shape[0]):
        tot_dist += np.linalg.norm(positions[i, :] - positions[i-1, :])
    return tot_dist



def run_test(gym_env, gym_model, num_rollouts: int, output_folder: Path, 
                policy_name: str=None, lstm_policy: bool=False) -> dict:

    # Open logging file
    stat_file_name = f'results_policy_{policy_name}.csv' if policy_name is not None \
        else 'results_policy.csv'
    csv_file = open(output_folder.joinpath(stat_file_name), 'w+')
    file_writer = csv.writer(csv_file)
    file_writer.writerow(["Env", "Run", "Success", "Reason", "Reward", "Height",
                          "Travelled Distance", "Steps"])

    path_file_name = f'results_policy_{policy_name}_path.csv' if policy_name is not None \
        else 'results_policy_path.csv'
    path_csv_file = open(output_folder.joinpath(path_file_name), 'w+')
    path_file_writer = csv.writer(path_csv_file)
    path_file_writer.writerow(["Env", "Run"])

    # Output
    success_envs = {}
    # Iterate over the environments
    for env_id in range(len(TESTING_ENVS_IDS)):
        # Now communicate the model
        env_model_name = ENVIRONMENTS_ID[TESTING_ENVS_IDS[env_id]]
        gym_env.update_model_id([TESTING_ENVS_IDS[env_id]])
        time.sleep(2)
        obs = gym_env.reset()
        zero_completed_obs = obs

        success_cum, travelled_distance_cum, total_steps_cum = 0, 0, 0
        for rollout in range(num_rollouts):
            # Storages for positions
            done = False
            state = None
            quad_positions = []
            while not done:
                # Act and get env info (supporting both LSMT and not LSTM cases)
                if lstm_policy:
                    act, state = gym_model.predict(zero_completed_obs, state, deterministic=True)
                else:
                    act, _ = gym_model.predict(zero_completed_obs, deterministic=True)
                obs, _, done, infos = gym_env.step(np.array([act[0]], dtype= np.float32))
                zero_completed_obs[0, :, :, :] = obs
                # Store the info to plot (before act to avoid problems with reset)
                quad_state = gym_env.get_quad_state()
                quad_positions.append(quad_state[0, :QUAD_POSIT_SIZE].tolist())

            # Log result to file
            reason = DONE_REASONS[int(infos[0]['extra_info']['reason'])]
            quad_positions = np.array(quad_positions)
            height = quad_positions[-1, 2]
            travelled_distance = compute_total_distance(quad_positions)
            total_steps = quad_positions.shape[0]  # This is the total number of steps
            success = infos[0]['extra_info']['success']

            # Stats (travelled distance and time only for successful exps)
            success_cum += success
            if success == 1:
                travelled_distance_cum += travelled_distance
                total_steps_cum += total_steps
            #
            file_writer.writerow([env_model_name, rollout, success, reason, infos[0]['episode']['r'],
                                  height, travelled_distance, total_steps])
            csv_file.flush()
            #
            path_row = [env_model_name, rollout]
            for i in range(len(quad_positions)):
                path_row.append(quad_positions[i][0])
                path_row.append(quad_positions[i][1])
                path_row.append(quad_positions[i][2])
            path_file_writer.writerow(path_row)

            # Printout
            logging.info("Env {} @ rollout {} / {} -> {} (success? {})".format(
                env_model_name, rollout+1, num_rollouts, reason, "True" if success == 1.0 else "False"))
            #
            time.sleep(TIME_SLEEP_BETWEEN_RUNS)

        # Store output (distance and time averaged over the successful exps)
        success_envs[env_model_name] = {
            "Success": float(success_cum) / float(num_rollouts),
            "Distance": float(travelled_distance_cum) / float(success_cum) if success_cum != 0 else 0.0,
            "Steps": float(total_steps_cum) / float(success_cum) if success_cum != 0 else 0.0
        }

    logging.info("Test with trained policy done!")
    
    return success_envs