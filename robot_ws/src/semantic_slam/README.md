# Semantic SLAM

### Communication
- Action

```
# RunSlam.action

# Goal
string output_uri       # e.g "file:///tmp/gs_map" (saving loc)
bool   save_on_stop     # true: will save the slam map when

---
# Result
bool   success
string map_uri          # saving URI（e.g., file:///tmp/gs_map_2025-10-04-15-30-00.npz）
uint32 num_splats

---
# Feedback
uint32 num_splats       # number of 3d gaussian
```

### Usage
```
# server 
ros2 run semantic_slam run_slam_server

(I build a fake incremental gs slam example)
Gaussian will be save in provided location as .npz file
Later other package (e.g.Object Query) can reach slam map by loading the npz files

# client
ros2 run semantic_slam run_slam_client
```