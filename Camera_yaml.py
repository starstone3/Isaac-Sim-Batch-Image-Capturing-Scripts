from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({
    "headless": True,
    "renderer": "Iris",
    "carb_settings": {
        "persistentKernelCache": False 
    }
})
import datetime
import omni.kit.actions.core
from omni.isaac.core.utils.stage import open_stage
from omni.isaac.core import World
import omni.isaac.core.utils.numpy.rotations as rot_utils
import omni.isaac.core.utils.prims as prims_utils
from omni.isaac.sensor import Camera
import numpy as np
import omni.usd
from pxr import UsdLux, Gf
from PIL import Image
import os
import json
import time
import argparse
from pathlib import Path
import collections
import gc
import yaml

class CameraBatchProcessor:
    def __init__(self, config_file, base_usd_path, output_base_path, max_concurrent_cameras=50, start_index=0):
        self.config_file = config_file
        self.base_usd_path = Path(base_usd_path)
        self.output_base_path = Path(output_base_path)
        self.max_concurrent_cameras = max_concurrent_cameras
        self.start_index = start_index
        
        with open(config_file, 'r') as f:
            self.config = json.load(f)
        
        self.scenes = [
            scene for scene in self.config.get('scenes', []) 
            if scene.get('scene_id') is not None and scene.get('scene_id') >= self.start_index
        ]
        
        self.output_base_path.mkdir(parents=True, exist_ok=True)
        
        print(f"找到 {len(self.scenes)} 个待处理场景 (起始ID >= {self.start_index})")

    def create_camera(self, prim_path, point_info):
        """创建单个摄像头，并应用位姿"""
        position = np.array(point_info['position'])
        position[2] += 1.0 
        orientation = np.array(point_info['rotation'])
        
        camera = Camera(
            prim_path=prim_path,
            position=position,
            frequency=3,
            resolution=(640, 480),
            orientation=orientation,
        )
        camera.initialize()
        camera.add_motion_vectors_to_frame()
        camera.set_horizontal_aperture(7)
        camera.set_vertical_aperture(5)
        stage = omni.usd.get_context().get_stage()
        light_path = f"{prim_path}/light"
        light = UsdLux.DomeLight.Define(stage, light_path)
        
        light.CreateIntensityAttr(250000.0)
        
        light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
        return camera

    def _setup_new_world(self, scene_path):
        """加载或重新加载场景，并设置好环境"""
        omni.usd.get_context().close_stage()
        gc.collect()
        
        if not open_stage(usd_path=str(scene_path)):
            print(f"错误：无法打开场景文件 {scene_path}")
            return None, None
        
        world = World()
        world.reset()
        
        for _ in range(10):
            world.step(render=True)
            
        return world

    def process_scene(self, scene_info):
        """
        处理一个场景的所有拍照点。
        修改后的逻辑：只加载一次场景，创建一批摄像头，然后通过调整位置和朝向来拍摄所有照片。
        """
        scene_id = scene_info['scene_id']
        scene_name = scene_info['scene_name']
        
        print(f"\n开始处理场景: {scene_name} (ID: {scene_id})")
        
        scene_path = self.base_usd_path / scene_name / f"{scene_name}.usd"
        if not scene_path.exists():
            print(f"警告: 场景文件不存在 {str(scene_path)}，跳过此场景。")
            return None
    
        world = self._setup_new_world(scene_path)
        if world is None:
            print(f"错误：无法加载场景 {scene_name}")
            return None
    
        all_points_in_scene = []
        for sample in scene_info['samples']:
            trajectory_id = sample['trajectory_id']
            for point_data in sample['points']:
                point_data_with_context = point_data.copy()
                point_data_with_context['trajectory_id'] = trajectory_id
                all_points_in_scene.append(point_data_with_context)
    
        total_points = len(all_points_in_scene)
        num_batches = (total_points + self.max_concurrent_cameras - 1) // self.max_concurrent_cameras
        print(f"场景 {scene_name} 共有 {total_points} 个拍照点, 将使用 {min(self.max_concurrent_cameras, total_points)} 个摄像头分 {num_batches} 批拍摄")

        num_cameras = min(self.max_concurrent_cameras, total_points)
        cameras = []
        for i in range(num_cameras):
            prim_path = f"/World/camera_{i}"
            initial_point = all_points_in_scene[i] if i < len(all_points_in_scene) else all_points_in_scene[0]
            camera = self.create_camera(prim_path, initial_point)
            cameras.append(camera)
            print(f"    创建摄像头 {prim_path}")
        
        for _ in range(10):
            world.step(render=True)
    
        all_results = []
        
        for batch_idx in range(num_batches):
            start_time = time.time()
            chunk_start = batch_idx * self.max_concurrent_cameras
            chunk_end = min(chunk_start + self.max_concurrent_cameras, total_points)
            point_chunk = all_points_in_scene[chunk_start:chunk_end]
            
            print(f"  处理批次 {batch_idx + 1}/{num_batches} (点 {chunk_start+1} 到 {chunk_end})...")

            if batch_idx == 0:
                print("  注意：这是第一次处理批次，摄像头位置和方向将基于初始点进行设置。")
            else :
                for i, point_info in enumerate(point_chunk):
                    if i < len(cameras):
                        camera = cameras[i]
                        position = np.array(point_info['position'])
                        position[2] += 1.0 
                        orientation = np.array(point_info['rotation'])
                        
                        camera.set_world_pose(position=position, orientation=orientation)
                        print(f"    调整摄像头 {i} 到位置: {point_info['position']}, 方向: {point_info['rotation']}")
                

            for i, point_info in enumerate(point_chunk):
                if i < len(cameras):
                    camera = cameras[i]
                    rgb_img = camera.get_rgba()[:, :, :3]
                    
                    scene_id = scene_info['scene_id']
                    scene_name = scene_info['scene_name']
                    trajectory_id = point_info['trajectory_id']
                    point_id = point_info['point']
                    
                    image_id = f"scene_{scene_id}_traj_{trajectory_id}_point_{point_id}"

                    trajectory_output_dir = self.output_base_path / f"{scene_name}" / f"trajectory_{trajectory_id}"
                    trajectory_output_dir.mkdir(parents=True, exist_ok=True)
                    
                    output_path = trajectory_output_dir / f"{image_id}.png"
                    Image.fromarray(rgb_img).save(str(output_path))
                    
                    all_results.append({
                        'trajectory_id': trajectory_id,
                        'point_id': point_id,
                        'image_id': image_id,
                        'image_path': f"trajectory_{trajectory_id}/{image_id}.png",
                        'position': point_info['position'],
                        'rotation': point_info['rotation'],
                    })
            
            end_time = time.time()
            print(f"  批次 {batch_idx + 1} 处理完成，耗时 {end_time - start_time:.2f} 秒。")

        updated_scene_data = self.aggregate_and_save_results(scene_info, all_results)
    
        print(f"场景 {scene_name} 处理完成，共拍摄 {len(all_results)} 张照片。")
        prim_paths_to_delete = [camera.prim_path for camera in cameras]
        for path in prim_paths_to_delete:
            prims_utils.delete_prim(path)
        cameras.clear()
        World.clear_instance()
        return updated_scene_data

    def aggregate_and_save_results(self, scene_info, all_results):
        """将扁平化的结果重新按轨迹组织，保存文件，并返回最终的场景数据结构"""
        scene_id = scene_info['scene_id']
        scene_name = scene_info['scene_name']
        
        results_by_traj = collections.defaultdict(list)
        for res in all_results:
            results_by_traj[res['trajectory_id']].append(res)
            
        original_points_map = {}
        for sample in scene_info['samples']:
            traj_id = sample['trajectory_id']
            for p in sample['points']:
                p['camera_images'] = []
            original_points_map[traj_id] = {p['point']: p for p in sample['points']}

        for trajectory_id, results in results_by_traj.items():
            for result in results:
                point_id = result['point_id']
                original_points_map[trajectory_id][point_id]['camera_images'].append({
                    'image_id': result['image_id'],
                    'image_path': result['image_path'],
                })

            updated_points = list(original_points_map[trajectory_id].values())
            
            trajectory_info = {
                'trajectory_id': trajectory_id,
                'points': updated_points
            }

            for i, sample in enumerate(scene_info['samples']):
                if sample['trajectory_id'] == trajectory_id:
                    scene_info['samples'][i] = trajectory_info
                    break

            scene_output_dir = self.output_base_path / f"{scene_name}"
            trajectory_output_dir = scene_output_dir / f"trajectory_{trajectory_id}"
            if not trajectory_output_dir.exists():
                raise FileNotFoundError(f"输出目录 {trajectory_output_dir} 不存在，前面的照片没有保存对。")

            trajectory_info_path = trajectory_output_dir / 'trajectory_info.json'
            with open(trajectory_info_path, 'w') as f:
                json.dump(trajectory_info, f, indent=2)

        
        scene_info_result = {
            'scene_id': scene_id,
            'scene_name': scene_name,
            'samples': scene_info['samples']
        }
        scene_info_path = scene_output_dir / 'scene_info.json'
        with open(scene_info_path, 'w') as f:
            json.dump(scene_info_result, f, indent=2)

        return scene_info_result

    def save_updated_config(self):
        """将内存中完全更新后的 self.config 对象保存到新文件"""
        updated_config_path = self.output_base_path / 'config_with_images.json'
        print(f"\n正在保存包含所有图片信息的最终配置文件到: {updated_config_path}")
        with open(updated_config_path, 'w') as f:
            json.dump(self.config, f, indent=2)
        print("最终配置文件保存成功。")

    def process_all_scenes(self, target_scene_ids=None):
        """处理所有场景，并在结束后保存一个更新过的配置文件"""
        scenes_to_process = [
            s for s in self.scenes if not target_scene_ids or s['scene_id'] in target_scene_ids
        ]
        
        total_scenes = len(scenes_to_process)
        successful_scenes = 0
        
        for i, scene_info in enumerate(scenes_to_process):
            print(f"\n{'='*60}")
            print(f"处理总进度: {i+1}/{total_scenes} - Scene ID: {scene_info['scene_id']}")
            print(f"{'='*60}")
            
            updated_scene_data = self.process_scene(scene_info)
            
            if updated_scene_data:
                successful_scenes += 1
                for idx, original_scene in enumerate(self.config['scenes']):
                    if original_scene['scene_id'] == updated_scene_data['scene_id']:
                        self.config['scenes'][idx] = updated_scene_data
                        print(f"场景 {scene_info['scene_id']} 的数据已更新到主配置中。")
                        break
            
            gc.collect()
        
        print(f"\n处理完成! 成功处理 {successful_scenes}/{total_scenes} 个场景")
        
        self.save_updated_config()
        
        return successful_scenes == total_scenes


def main():
    parser = argparse.ArgumentParser(description='通过 YAML 配置文件批量处理摄像头拍照')
    
    parser.add_argument('--config', type=str, required=True,
                        help='指向 YAML 配置文件的路径')
    
    parser.add_argument('--scene_ids', type=int, nargs='*', 
                        help='(覆盖选项) 要处理的特定场景ID列表。如果提供，将忽略YAML中的设置。')
    parser.add_argument('--output_path', type=str, default=None,
                        help='(覆盖选项) 输出路径。如果提供，将忽略YAML中的设置。')

    args = parser.parse_args()

    try:
        with open(args.config, 'r') as f:
            config = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"错误: 配置文件 {args.config} 未找到。")
        simulation_app.close()
        return
    except yaml.YAMLError as e:
        print(f"错误: 解析 YAML 文件 {args.config} 时出错: {e}")
        simulation_app.close()
        return

    input_cfg = config.get('input', {})
    output_cfg = config.get('output', {})
    proc_cfg = config.get('processing', {})

    camera_json_file = input_cfg.get('camera_config_json', './Camera_scripts/smooth.json')
    base_usd_path = input_cfg.get('base_usd_path', './matterport_usd')
    
    output_path = args.output_path if args.output_path else output_cfg.get('base_path')
    if not output_path:
        now_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = f"./data/test_{now_str}"
        print(f"未指定输出路径，将使用自动生成的路径: {output_path}")

    max_cameras = proc_cfg.get('max_concurrent_cameras', 50)
    start_index = proc_cfg.get('start_scene_index', 0)
    
    target_scene_ids = args.scene_ids if args.scene_ids else proc_cfg.get('target_scene_ids')

    print("\n" + "="*50)
    print("配置加载成功:")
    print(f"  > 摄像头JSON: {camera_json_file}")
    print(f"  > USD 基础路径: {base_usd_path}")
    print(f"  > 输出路径: {output_path}")
    print(f"  > 单批最大摄像头数: {max_cameras}")
    print(f"  > 起始场景索引: {start_index}")
    if target_scene_ids:
        print(f"  > 目标场景ID: {target_scene_ids}")
    else:
        print("  > 目标场景ID: (全部)")
    print("="*50 + "\n")

    processor = CameraBatchProcessor(
        config_file=camera_json_file,
        base_usd_path=base_usd_path,
        output_base_path=output_path,
        max_concurrent_cameras=max_cameras,
        start_index=start_index
    )
    
    if not processor.scenes:
        print("错误: 配置文件中没有找到任何符合条件的场景配置")
        simulation_app.close()
        return
    
    success = processor.process_all_scenes(target_scene_ids)
    
    if success:
        print(f"\n✅ 成功完成所有指定场景的处理")
    else:
        print(f"\n⚠️ 部分或全部场景处理失败")
    
    simulation_app.close()


if __name__ == "__main__": 
    main()
