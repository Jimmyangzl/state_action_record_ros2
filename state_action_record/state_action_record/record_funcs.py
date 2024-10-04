from ament_index_python.packages import get_package_share_directory
import os
import yaml
import argparse
import importlib
import numpy as np

pkg_name = 'state_action_record'

def get_parser():
    config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'config.yaml'
    )
    with open(config_path, 'r') as file:
        configs = yaml.safe_load(file)
    arg_names = list(configs.keys())
    parser = argparse.ArgumentParser(
                    prog=pkg_name,
                    description='Record data needed for robot LfD.')
    for arg_name in arg_names:
        parser.add_argument("--"+arg_name, help="(Option) Specify "+arg_name)
    # args = parser.parse_args()
    return parser

def get_config(config_name):
    try:
        config_path = os.path.join(
            get_package_share_directory(pkg_name),
            'config',
            config_name+'.yaml'
        )
        with open(config_path, 'r') as file:
            return yaml.safe_load(file)
    except:
        return None
    
def get_msg_type(msg_type):
    msg_name = msg_type.replace("/",".")
    msg_module_name, msg_type_name = msg_name.rsplit('.', 1)
    msg_module = importlib.import_module(msg_module_name)
    try:
        return getattr(msg_module, msg_type_name)
    except AttributeError:
        return None
