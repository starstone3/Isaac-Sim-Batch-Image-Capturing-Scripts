# Isaac Sim Batch Image-Capturing Scripts

This repository provides two Python scripts for batch capturing images from multiple camera poses within the NVIDIA Isaac Sim environment. Both scripts automate the process of data generation for computer vision tasks by systematically loading 3D scenes, positioning cameras according to a configuration file, and capturing images.

---

## Table of Contents

- [Prerequisites](#prerequisites)
- [Script Overview](#script-overview)
  - [1. Camera_batch.py (JSON-based)](#1-camerabatchpy-json-based)
  - [2. Camera_yaml.py (YAML-based)](#2-camerayamlpy-yaml-based)
- [Input Data Format](#input-data-format)
- [Usage](#usage)
- [Output Structure](#output-structure)
- [Troubleshooting](#troubleshooting)
- [License](#license)

---

## Prerequisites

- **NVIDIA Omniverse Isaac Sim** (Tested with version: `4.2.0-rc.18+release.16044.3b2ed111.gl`)
- Python environment capable of running Isaac Sim scripts (use Isaac Sim's `python.sh` or `python.bat`)
- Additional Python packages: `numpy`, `pillow`, `pyyaml` (for YAML version)

---

## Script Overview

### 1. `Camera_batch.py` (JSON-based)

- Uses a JSON configuration file to define scenes and camera viewpoints.
- Command-line arguments allow you to specify the config file, USD base path, output path, and other options.
- Suitable for workflows where all configuration is in a single JSON file.

### 2. `Camera_yaml.py` (YAML-based)

- Uses a YAML configuration file for more flexible and readable configuration.
- YAML config can reference a JSON camera config, USD base path, output path, and processing options.
- Command-line arguments can override YAML settings.
- Recommended for more complex or batch workflows.

---

## Input Data Format

### JSON Camera Configuration

Both scripts expect a JSON file describing scenes, trajectories, and camera points. Example (`sample.json`):

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
            }
          ]
        }
      ]
    }
  ]
}
```

### YAML Batch Configuration (for `Camera_yaml.py`)

Example:

```yaml
input:
  camera_config_json: ./sample.json
  base_usd_path: ./matterport_usd

output:
  base_path: null

processing:
  max_concurrent_cameras: 50
  start_scene_index: 0
  target_scene_ids: null
```

---

## Usage

### 1. Using `Camera_batch.py` (JSON-based)

```bash
isaacsim/python.sh Camera_batch.py \
    --config_file ./sample.json \
    --base_usd_path /path/to/your/usd_scenes \
    --output_path /path/to/your/output_directory \
    --max_concurrent_cameras 50
```

**Key Arguments:**

- `--config_file`: Path to the main JSON configuration file (required)
- `--base_usd_path`: Path to the base directory containing the USD scene folders (required)
- `--output_path`: Output directory (required)
- `--scene_ids`: (Optional) Space-separated list of scene IDs to process
- `--max_concurrent_cameras`: (Optional) Max number of cameras per batch (default: 50)
- `--start_index`: (Optional) Start processing from this scene ID (default: 0)

---

### 2. Using `Camera_yaml.py` (YAML-based)

```bash
isaacsim/python.sh Camera_yaml.py --config your_config.yaml
```

**Key Arguments:**

- `--config`: Path to the YAML configuration file (required)
- `--scene_ids`: (Optional) Override: list of scene IDs to process
- `--output_path`: (Optional) Override: output directory

---

## Output Structure

Both scripts generate the following output structure:

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

- `config_with_images.json`: The original config file, updated with image paths in `camera_images`.
- `scene_info.json`: All trajectory data for a specific scene.
- `trajectory_info.json`: Point data and image paths for a specific trajectory.

---

## Troubleshooting

1. `[Error] [carb.graphics-vulkan.plugin] GPU crash is detected`:  
   Try using fewer GPUs. In our practice, 3 RTX 3090 GPUs can run the script simultaneously without crashing, but 4 GPUs may cause a crash.

2. When running the scripts in a `screen` session or in the background, the process may hang or become unresponsive. To avoid this, try redirecting both stdout and stderr to a log file, for example:
   ```bash
   isaacsim/python.sh Camera_batch.py ... > output.log 2>&1
   ```

---

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.