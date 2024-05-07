import os
from distutils.util import strtobool
from pathlib import Path

from dotenv import load_dotenv

import supervisely as sly

if sly.is_development():
    load_dotenv("local.env")
    load_dotenv(os.path.expanduser("~/supervisely.env"))

ABSOLUTE_PATH = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.dirname(ABSOLUTE_PATH)

app_data = Path(PARENT_DIR).joinpath("temp").as_posix()
sly.fs.mkdir(app_data, remove_content_if_exists=True)

api = sly.Api.from_env()

task_id = sly.env.task_id()
team_id = sly.env.team_id()
workspace_id = sly.env.workspace_id()
project_id = sly.env.project_id()
dataset_id = sly.env.dataset_id(raise_not_found=False)


sly_ann_mode = os.environ.get("modal.state.annMode", "sly") == "sly"
merge_mode = bool(strtobool(os.environ.get("modal.state.mergeMode", "false")))
fill_mode = bool(strtobool(os.environ.get("modal.state.fillPoints", "false")))
fill_rate = int(os.environ.get("modal.state.fillRate", 50))

if sly_ann_mode:
    merge_mode = False
    fill_mode = False
