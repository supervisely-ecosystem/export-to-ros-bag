<div align="center" markdown>

<img align="center" src="https://github.com/supervisely-ecosystem/export-to-ros-bag/releases/download/v0.0.1/export_ros_poster.png">

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

- Export Supervisely pointcloud or pointcloud episodes projects/datasets as ROS bag files
- Pointclouds `.pcd` will be saved as `/PointCloud2` topic
- Annotations will be saved as `/SlyAnnotations` topic
- 2 options for exporting annotations: `geometry_msgs/Vector3Stamped` (allows to import as annotations back to Supervisely) and `sensor_msgs/PointCloud2` (will be transformed to point clouds, can be merged with original point clouds in the new ROS bag file)
- Backward compatibility with importing the same bag file into Supervisely (demo video below)

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

## Export demo

<video preload="auto" autoplay muted loop>
    <source src="https://github.com/supervisely-ecosystem/export-to-ros-bag/releases/download/v0.0.1/export.mp4" type="video/mp4">
</video>

## Import demo

<video preload="auto" autoplay muted loop>
    <source src="https://github.com/supervisely-ecosystem/export-to-ros-bag/releases/download/v0.0.1/import.mp4" type="video/mp4">
</video>

## Output

The app creates a task in `workspace tasks` list. Once the app is finished, you will see a download link to resulting tar archive.
The archive will be saved to the `Files` in:

`Current Team` -> `Files` -> `/tmp/supervisely/export/Export Point Clouds to ROS Bag/<task_id>/<project_id>_<project_name>.tar`.
