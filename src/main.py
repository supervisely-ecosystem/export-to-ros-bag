from pathlib import Path

import src.functions as f
import src.globals as g
import supervisely as sly

class MyExport(sly.app.Export):
    def process(self, context: sly.app.Export.Context):
        project = g.api.project.get_info_by_id(id=context.project_id)

        sly_project = f.download_project(g.api, project, g.app_data, context.dataset_id)
        meta = sly_project.meta

        bag_dir = Path(g.app_data).joinpath("bags")
        bag_dir.mkdir(exist_ok=True)
        for dataset_fs in sly_project.datasets:
            items_points = []
            f.process_dataset(project, dataset_fs, meta, items_points)
            f.write_bag(bag_dir, dataset_fs, items_points)

        return bag_dir.as_posix()


def main():
    try:
        app = MyExport()
        app.run()
    except Exception as e:
        f.handle_exception(e, g.api, g.task_id)
    finally:
        if not sly.is_development():
            sly.fs.remove_dir(g.app_data)


if __name__ == "__main__":
    sly.main_wrapper("main", main, log_for_agent=False)
