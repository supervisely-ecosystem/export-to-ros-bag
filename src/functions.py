import re
import time
from pathlib import Path
from typing import Union

import numpy as np
import rosbag
import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Vector3, Vector3Stamped
from std_msgs.msg import Header
from tqdm import tqdm

import src.globals as g
import supervisely as sly
from supervisely.geometry.cuboid_3d import Cuboid3d


def get_progress(
    total: int,
    message: str = "Processing...",
    is_size: bool = False,
) -> tuple:
    if sly.is_production():
        progress = sly.Progress(message, total, is_size=is_size)
        progress_cb = progress.iters_done_report
    else:
        progress = tqdm(
            total=total, desc=message, unit="B" if is_size else "it", unit_scale=is_size
        )
        progress_cb = progress.update
    return progress, progress_cb


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
            api,
            project.id,
            local_path,
            download_pointclouds_info=True,
            dataset_ids=dataset_ids,
            log_progress=True,
        )
        sly_project = sly.PointcloudProject(local_path, sly.OpenMode.READ)
    elif project.type == str(sly.ProjectType.POINT_CLOUD_EPISODES):
        sly.download_pointcloud_episode_project(
            api,
            project.id,
            local_path,
            download_pointclouds_info=True,
            dataset_ids=dataset_ids,
            log_progress=True,
        )
        sly_project = sly.PointcloudEpisodeProject(local_path, sly.OpenMode.READ)
    else:
        raise ValueError(f"Unsupported project type: {project.type}")

    return sly_project


def get_rostime(pcd_name: str = None, increment: int = 0):
    if pcd_name is not None:
        if pcd_name.endswith(".pcd"):
            pcd_name = Path(pcd_name).stem
        if re.match(r"\d+\.\d+", pcd_name):
            secs = int(pcd_name.split(".")[0])
            nsecs = int(pcd_name.split(".")[1])
            return rospy.rostime.Time(secs, nsecs + increment * 10000000)
    float_secs = time.time()
    secs = int(float_secs)
    nsecs = int((float_secs - secs) * 1000000000)
    return rospy.rostime.Time(secs, nsecs)


def get_points_from_cuboid(all_coords: list, cuboid: Cuboid3d):
    bbox_center = np.array([cuboid.position.x, cuboid.position.y, cuboid.position.z])
    bbox_size = np.array([cuboid.dimensions.x, cuboid.dimensions.y, cuboid.dimensions.z])
    bbox_rotation = [cuboid.rotation.x, cuboid.rotation.y, cuboid.rotation.z]

    half_dimensions = bbox_size / 2
    rotation_matrix = np.array(
        [
            [np.cos(bbox_rotation[2]), -np.sin(bbox_rotation[2]), 0],
            [np.sin(bbox_rotation[2]), np.cos(bbox_rotation[2]), 0],
            [0, 0, 1],
        ]
    )
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
    all_coords.extend(corner_points)
    if g.fill_mode:
        edge_points = []
        for i in range(0, 7, 2):
            edge_points.extend(
                np.linspace(corner_points[i], corner_points[i + 1], num=g.fill_rate, endpoint=True)
            )
        for i in [0, 1, 4, 5]:
            edge_points.extend(
                np.linspace(corner_points[i], corner_points[i + 2], num=g.fill_rate, endpoint=True)
            )
        for i in range(4):
            edge_points.extend(
                np.linspace(corner_points[i], corner_points[i + 4], num=g.fill_rate, endpoint=True)
            )

        all_coords.extend(edge_points)


def process_pcd_figure_to_vector(fig: sly.PointcloudFigure, rostime: rospy.rostime.Time, increment):
    if fig.geometry.geometry_name() == Cuboid3d.geometry_name():
        cuboid = fig.geometry
        frame_id = f"{rostime.secs}.{rostime.nsecs}"
        center = Vector3Stamped(
            Header(frame_id=frame_id, stamp=get_rostime(frame_id, increment)),
            Vector3(cuboid.position.x, cuboid.position.y, cuboid.position.z),
        )
        size = Vector3Stamped(
            Header(frame_id=frame_id, stamp=get_rostime(frame_id, increment)),
            Vector3(cuboid.dimensions.x, cuboid.dimensions.y, cuboid.dimensions.z),
        )
        rotation = Vector3Stamped(
            Header(frame_id=frame_id, stamp=get_rostime(frame_id, increment)),
            Vector3(cuboid.rotation.x, cuboid.rotation.y, cuboid.rotation.z),
        )
        return center, size, rotation


def process_pcd_figure_to_cloud(all_coords: list, fig: sly.PointcloudFigure):
    if fig.geometry.geometry_name() == Cuboid3d.geometry_name():
        cuboid = fig.geometry
        center = np.array([cuboid.position.x, cuboid.position.y, cuboid.position.z])
        size = np.array([cuboid.dimensions.x, cuboid.dimensions.y, cuboid.dimensions.z])
        rotation = np.array([cuboid.rotation.x, cuboid.rotation.y, cuboid.rotation.z])

        all_coords.append(center)
        all_coords.append(size)
        all_coords.append(rotation)


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
    progress, progress_cb = get_progress(len(items_points), f"Writing rosbag {bag_path.name}...")
    with rosbag.Bag(bag_path, "w") as bag:
        for pcd_points, ann, rostime in items_points:
            bag.write("PointCloud2", pcd_points, t=rostime)
            if ann is not None:
                if g.sly_ann_mode:
                    for msg in ann:
                        bag.write("SlyAnnotations", msg, t=msg.header.stamp)
                else:
                    bag.write("SlyAnnotations", ann, t=rostime)

            progress_cb(1)
        if sly.is_development():
            progress.close()


def process_dataset(
    project: sly.Project,
    dataset_fs: Union[sly.PointcloudEpisodeDataset, sly.PointcloudEpisodeDataset],
    meta: sly.ProjectMeta,
    items_points: list,
):
    items_names = dataset_fs.get_items_names()
    progress, progress_cb = get_progress(
        len(items_names), f"Processing dataset {dataset_fs.name}..."
    )
    if project.type == str(sly.ProjectType.POINT_CLOUD_EPISODES):
        ann = sly.PointcloudEpisodeAnnotation.load_json_file(dataset_fs.get_ann_path(), meta)
    for item_name in items_names:
        i = 0
        pcd_path = dataset_fs.get_item_path(item_name)
        if project.type == str(sly.ProjectType.POINT_CLOUDS):
            ann_path = dataset_fs.get_ann_path(item_name)

        pcd_info = dataset_fs.get_item_info_path(item_name)
        pcd_info = sly.json.load_json_file(pcd_info)
        pcd_meta = pcd_info.get("meta", {})
        rostime = get_rostime(pcd_meta.get("time", item_name))

        # read annotation
        label_coords = []
        vector_msgs = []
        if project.type == str(sly.ProjectType.POINT_CLOUDS):
            ann = sly.PointcloudAnnotation.load_json_file(ann_path, meta)
            for label in ann.figures:
                if g.sly_ann_mode:
                    # process_pcd_figure_to_cloud(label_coords, label) # NOTE: temporary commented out
                    msg_1, msg_2, msg_3 = process_pcd_figure_to_vector(label, rostime, i)
                    vector_msgs.extend([msg_1, msg_2, msg_3])
                    i += 3
                else:
                    get_points_from_cuboid(label_coords, label.geometry)
        else:
            frame_index = dataset_fs.get_frame_idx(item_name)
            figures = ann.get_figures_on_frame(frame_index)
            for fig in figures:
                if g.sly_ann_mode:
                    #     process_pcd_figure_to_cloud(label_coords, fig) # NOTE: temporary commented out
                    msg_1, msg_2, msg_3 = process_pcd_figure_to_vector(fig, rostime, i)
                    vector_msgs.extend([msg_1, msg_2, msg_3])
                    i += 3
                else:
                    get_points_from_cuboid(label_coords, fig.geometry)

        if len(label_coords) > 0:
            label_coords = np.array(label_coords, dtype=np.float32)
        # read point cloud
        pcd_points = sly.pointcloud.read(pcd_path)  # np.array

        # create PointCloud2 message
        header = Header()
        header.stamp = rostime
        header.frame_id = pcd_meta.get("frame_id", "os1_lidar")

        if g.merge_mode:
            if len(label_coords) > 0:
                pcd_points = np.concatenate((pcd_points, label_coords), axis=0)
            ann_points = None
        else:
            ann_points = pc2.create_cloud_xyz32(
                header, label_coords
            )  # NOTE: temporary commented out

        pcd_points = pc2.create_cloud_xyz32(header, pcd_points)
        if g.sly_ann_mode:
            items_points.append((pcd_points, vector_msgs, rostime))
        else:
            items_points.append((pcd_points, ann_points, rostime))
        progress_cb(1)
    if sly.is_development():
        progress.close()
