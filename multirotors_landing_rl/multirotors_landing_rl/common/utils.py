import datetime
from pathlib import Path
from typing import Dict, List
from pathlib import Path
from ruamel.yaml import YAML


class ConfigurationSaver:
    """Utility class to store configuration files
    """
    def __init__(self, log_dir: Path, env_name: str, is_test: bool=False):
        if not is_test:
            self._folder_name = datetime.datetime.now().strftime(
                "%Y-%m-%d-%H-%M-%S_" + env_name)
        else:
            self._folder_name = datetime.datetime.now().strftime(
                "test_%Y-%m-%d-%H-%M-%S_" + env_name)
        self._data_dir = log_dir.joinpath(self._folder_name)
        if not self._data_dir.exists():
            self._data_dir.mkdir(parents=True)

    def log_arguments_and_params(self, args: Dict, policy_name: str, configs: List[str]):
        param_file = open(self._data_dir.joinpath("parameters.txt"), mode='w')
        # Store arguments
        param_file.write("########## ARGUMENTS ##########\n")
        param_file.write("Policy:             {}\n".format(policy_name))
        param_file.write("Keep Train:         {}\n".format(args.keep_train))
        param_file.write("Save directory:     {}\n".format(args.save_dir))
        param_file.write("Seed:               {}\n".format(args.seed))
        param_file.write("Weight:             {}\n".format(args.weight))
        param_file.write("\n########## CONFIG FILEs ##########\n")
        for c in configs:
            YAML().dump(c, param_file)
            param_file.write("\n")
        param_file.close()

    @property
    def data_dir(self):
        return self._data_dir

    @property
    def folder_name(self):
        return self._folder_name
