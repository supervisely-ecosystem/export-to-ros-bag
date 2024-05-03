import re
import time
from pathlib import Path

import numpy as np
import rosbag
import rospy
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

import src.globals as g
import supervisely as sly
from supervisely.geometry.cuboid_3d import Cuboid3d


def download_project(
    api: sly.Api,
    project: sly.Project,
    app_data: Path,
    dataset_id: int = None,
):
    local_path = Path(app_data).joinpath(project.name)
    if not local_path.exists():
        local_path.mkdir(parents=True, exist_ok=True)
    local_path = local_path.as_posix()
    if dataset_id is not None:
        dataset_ids = [dataset_id]
        nested_datasets = api.dataset.get_list(project.id, parent_id=dataset_id)
        dataset_ids.extend([d.id for d in nested_datasets])
    else:
        dataset_ids = None

    sly.fs.mkdir(local_path, remove_content_if_exists=True)
    if project.type == str(sly.ProjectType.POINT_CLOUDS):
        sly.download_pointcloud_project(
            api, project.id, local_path, download_pointclouds_info=True, dataset_ids=dataset_ids
        )
        sly_project = sly.PointcloudProject(local_path, sly.OpenMode.READ)
    elif project.type == str(sly.ProjectType.POINT_CLOUD_EPISODES):
        sly.download_pointcloud_episode_project(
            api, project.id, local_path, download_pointclouds_info=True, dataset_ids=dataset_ids
        )
        sly_project = sly.PointcloudEpisodeProject(local_path, sly.OpenMode.READ)
    else:
        raise ValueError(f"Unsupported project type: {project.type}")

    return sly_project


def get_rostime(pcd_name: str = None):
    if pcd_name is not None:
        if pcd_name.endswith(".pcd"):
            pcd_name = Path(pcd_name).stem
        if re.match(r"\d+\.\d+", pcd_name):
            secs = int(pcd_name.split(".")[0])
            nsecs = int(pcd_name.split(".")[1])
            return rospy.rostime.Time(secs, nsecs)
    float_secs = time.time()
    secs = int(float_secs)
    nsecs = int((float_secs - secs) * 1000000000)
    return rospy.rostime.Time(secs, nsecs)


def process_pcd_figure(fig: sly.PointcloudFigure):
    label_points = []
    if fig.geometry.geometry_name() == Cuboid3d.geometry_name():
        cuboid = fig.geometry
        bbox_center = np.array([cuboid.position.x, cuboid.position.y, cuboid.position.z])
        bbox_size = np.array([cuboid.dimensions.x, cuboid.dimensions.y, cuboid.dimensions.z])
        bbox_rotation = [cuboid.rotation.x, cuboid.rotation.y, cuboid.rotation.z]

        half_dimensions = bbox_size / 2

        # Create a rotation matrix based on the rotation angles
        rotation_matrix = np.array(
            [
                [np.cos(bbox_rotation[2]), -np.sin(bbox_rotation[2]), 0],
                [np.sin(bbox_rotation[2]), np.cos(bbox_rotation[2]), 0],
                [0, 0, 1],
            ]
        )
        # Calculate the 8 corner points of the cuboid
        corner_points = []
        for i in [-1, 1]:
            for j in [-1, 1]:
                for k in [-1, 1]:
                    corner_point = (
                        bbox_center
                        + i * rotation_matrix.dot([half_dimensions[0], 0, 0])
                        + j * rotation_matrix.dot([0, half_dimensions[1], 0])
                        + k * rotation_matrix.dot([0, 0, half_dimensions[2]])
                    )
                    corner_points.append(corner_point)
        label_points.extend(corner_points)

        edge_points = []

        label_points.extend(edge_points)
    label_points = np.array(label_points)
    return label_points


def handle_exception(exc: Exception, api: sly.Api, task_id: int):
    from supervisely.io.exception_handlers import handle_exception as sly_handle_exception

    handled_exc = sly_handle_exception(exc)
    if handled_exc is not None:
        api.task.set_output_error(task_id, handled_exc.title, handled_exc.message)
        err_msg = handled_exc.get_message_for_exception()
        sly.logger.error(err_msg, exc_info=True)
    else:
        err_msg = repr(exc)
        if len(err_msg) > 255:
            err_msg = err_msg[:252] + "..."
        title = "Error occurred"
        api.task.set_output_error(task_id, title, err_msg)
        sly.logger.error(f"{repr(exc)}", exc_info=True)


def write_bag(bag_dir: Path, dataset_fs: sly.Dataset, items_points: list):
    bag_path = bag_dir.joinpath(f"{dataset_fs.name}.bag")
    with rosbag.Bag(bag_path, "w") as bag:
        for pcd_points, ann_points, rostime in items_points:
            bag.write("PointCloud2", pcd_points, t=rostime)
            bag.write("SlyAnnotations", ann_points, t=rostime)


def process_dataset(
    project: sly.Project,
    dataset_fs: sly.Dataset,
    meta: sly.ProjectMeta,
    items_points: list,
):
    items_names = dataset_fs.get_items_names()
    if project.type == str(sly.ProjectType.POINT_CLOUD_EPISODES):
        ann = sly.PointcloudEpisodeAnnotation.load_json_file(
            dataset_fs.get_ann_path(), meta
        )
    for item_name in items_names:
        pcd_path = dataset_fs.get_item_path(item_name)
        if project.type == str(sly.ProjectType.POINT_CLOUDS):
            ann_path = dataset_fs.get_ann_path(item_name)

        pcd_info = dataset_fs.get_item_info_path(item_name)
        pcd_info = sly.json.load_json_file(pcd_info)
        pcd_meta = pcd_info.get("meta", {})

        # read annotation
        label_points = []
        if project.type == str(sly.ProjectType.POINT_CLOUDS):
            ann = sly.PointcloudAnnotation.load_json_file(ann_path, meta)
            for label in ann.figures:
                label: sly.PointcloudFigure
                label_points.extend(process_pcd_figure(label))
        else:
            frame_index = pcd_meta.get("frame", 0)
            figures = ann.get_figures_on_frame(frame_index)
            for fig in figures:
                label_points.extend(process_pcd_figure(fig))

        # read point cloud
        pcd_points = sly.pointcloud.read(pcd_path)  # np.array

        # create PointCloud2 message
        header = Header()
        rostime = get_rostime(pcd_meta.get("time", item_name))
        header.stamp = rostime
        header.frame_id = pcd_meta.get("frame_id", "os1_lidar")

        pcd_points = pc2.create_cloud_xyz32(header, pcd_points)
        ann_points = pc2.create_cloud_xyz32(header, label_points)
        items_points.append((pcd_points, ann_points, rostime))
