import yaml
import os

def load_command_mappings(file_path):
    """Load command mappings from a YAML file."""
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Command mappings file '{file_path}' not found.")
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)['commands']
