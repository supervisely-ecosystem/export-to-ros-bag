<div align="center" markdown>

<!-- <img align="center" src="" width="250"> -->

# Export to ROS Bag

[![](https://img.shields.io/badge/supervisely-ecosystem-brightgreen)](https://ecosystem.supervise.ly/apps/supervisely-ecosystem/export-to-ros-bag)
[![](https://img.shields.io/badge/slack-chat-green.svg?logo=slack)](https://supervise.ly/slack)
![GitHub release (latest SemVer)](https://img.shields.io/github/v/release/supervisely-ecosystem/export-to-ros-bag)
[![views](https://app.supervise.ly/img/badges/views/supervisely-ecosystem/export-to-ros-bag.png)](https://supervise.ly)
[![runs](https://app.supervise.ly/img/badges/runs/supervisely-ecosystem/export-to-ros-bag.png)](https://supervise.ly)

</div>

## Overview

The **Export to ROS Bag** application is a tool that allows you to export pointclouds and pointcloud episodes from Supervisely projects to ROS bag files. It provides a seamless integration between Supervisely and ROS, enabling you to leverage the power of ROS for further analysis and processing of your annotated data.
The application preserves annotations (3D Cuboid geometry only).

### Key Features

- Export Supervisely projects or dataset (pointclouds or pointcloud episodes) to ROS bag files
- Pointclouds `.pcd` will be saved as `/PointCloud2` topic
- Annotations will be saved as `/SlyAnnotations` topic
- Backward compatibility with importing the same bag file into Supervisely

## Need Help?

If you encounter any issues or have questions regarding this application, don't hesitate to reach out to our support team in [Slack](https://supervisely.com/slack/).

## How to Run

To run The application, follow these steps:

Option 1. Supervisely Ecosystem:

1. Find the application in the Ecosystem

2. Choose the necessary project or dataset and press `Run` button

Option 2: Project/Dataset context menu:

1. Go to Pointcloud / Pointcloud episode project or dataset you want to export

2. Right-click on the project or dataset and choose `Download as` -> `Export to ROS Bag`

3. Press `Run` button in the modal window

## Output

The app creates a task in `workspace tasks` list. Once the app is finished, you will see a download link to resulting tar archive.
The archive will be saved to the `Files` in:

`Current Team` -> `Files` -> `/tmp/supervisely/export/Export Point Clouds to ROS Bag/<task_id>/<project_id>_<project_name>.tar`.
