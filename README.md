# Isaac Sim Batch Image-Capturing Scripts

This project provides a Python script for batch capturing images from multiple camera poses within the NVIDIA Isaac Sim environment. It is designed to automate the process of data generation for computer vision tasks by systematically loading 3D scenes, positioning cameras according to a configuration file, and capturing images.


## Prerequisites

*   NVIDIA Omniverse Isaac Sim (In our project ,the VERSION of issac sim is **4.2.0-rc.18+release.16044.3b2ed111.gl**).

*   A Python environment capable of running Isaac Sim scripts. The script is intended to be run with Isaac Sim's Python executable (e.g., `python.sh` or `python.bat`).

## Input Data Format

The script requires an input JSON file (`--config_file`) that defines the scenes and camera viewpoints. The structure should follow the example of `sample.json`,which is also uploaded as part of this repository. This file contains a sample scene, with multiple camera viewpoints defined by their position and orientation.

### JSON Structure

```json
{
  "scenes": [
    {
      "scene_id": 0,
      "scene_name": "YourSceneName",
      "samples": [
        {
          "trajectory_id": "57",
          "points": [
            {
              "point": "0",
              "position": [16.48, -8.97, 0.12],
              "rotation": [0.707, 0.0, 0.0, 0.707],
              "camera_images": []
            },
            {
              "point": "1",
              "position": [16.52, -8.26, 0.12],
              "rotation": [0.737, 0.0, 0.0, 0.675],
              "camera_images": []
            }
          ]
        }
      ]
    }
  ]
}
```

*   `scene_name`: Corresponds to the folder name containing the `.usd` file. The script expects the USD file to be at `{base_usd_path}/{scene_name}/{scene_name}.usd`.

*   `position`: An `[x, y, z]` list for the camera's location. Notice that we increase the height of the camera by 1.0 unit to prevent it from being too close to the ground.

*   `rotation`: A `[w, x, y, z]` quaternion for the camera's orientation.

*   `camera_images`: An empty list that will be populated by the script in the output JSON.

## Usage

Execute the script using the Python interpreter provided with Isaac Sim from your terminal.

Below is an example command to run the script:

```bash
isaacsim/python.sh /path/to/your/repo/Camera_batch.py \
    --config_file ./sample.json \
    --base_usd_path /path/to/your/usd_scenes \
    --output_path /path/to/your/output_directory \
    --max_concurrent_cameras 50
```

### Command-Line Arguments

*   `--config_file`: (Required) Path to the main JSON configuration file.

*   `--base_usd_path`: (Required) Path to the base directory containing the USD scene folders.

*   `--output_path`: (Required) Path to the directory where all outputs will be saved.

*   `--scene_ids`: (Optional) A space-separated list of scene IDs to process. If not provided, all scenes in the config file are processed. Example: `--scene_ids 0 2 5`.

*   `--max_concurrent_cameras`: (Optional) The maximum number of cameras to use in a single batch. A lower number reduces VRAM usage. Default: `50`.

*   `--start_index`: (Optional) A scene ID to start processing from. Scenes with an ID lower than this will be skipped. Default: `0`.

## Output Structure

The script will generate the following structure in the specified `--output_path`:

```
/path/to/your/output_directory/
├── YourSceneName/
│   ├── scene_info.json
│   └── trajectory_57/
│       ├── scene_0_traj_57_point_0.png
│       ├── scene_0_traj_57_point_1.png
│       └── trajectory_info.json
│
├── AnotherSceneName/
│   ├── ...
│
└── config_with_images.json
```

*   `config_with_images.json`: A copy of the original input configuration file, but with the `camera_images` fields populated with the paths to the rendered images.

*   `scene_info.json`: Contains all the trajectory data for a specific scene.

*   `trajectory_info.json`: Contains the point data and image paths for a specific trajectory.


## Common Issues

1. `[Error] [carb.graphics-vulkan.plugin] GPU crash is detected`:Try to use fewer GPUs.In our practice,3 RTX 3090 GPUs can run the script simultaneously without crashing but 4 GPUs may cause a crash.


## License

This project is licensed under the MIT License. See the `LICENSE` file for details.