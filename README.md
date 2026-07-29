# GraspNet ROS2 Player

GraspNet データセットの `scene_id` を ROS2 で再生するための小さなプレイヤーです。

## Docker 起動

```bash
docker compose up --build
```

コンテナ内でビルドします。

```bash
source /opt/ros/humble/setup.bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Scene 再生

`scene_id` を指定して起動します。データは `/data/graspnet` にマウントされる前提です。

```bash
ros2 launch graspnet_ros2 play_scene.launch.py scene_id:=3 camera:=realsense ann_id:=0
```

上下が逆に見える場合は、まずこれを試してください。

```bash
ros2 launch graspnet_ros2 play_scene.launch.py scene_id:=3 camera:=realsense ann_id:=0 invert_camera_pose:=true
```

まだ平面が変なら、`use_table_frame:=true` のまま `invert_camera_pose` を切り替えてください。

Kinect 側のデータを使う場合は `camera:=kinect` にします。

```bash
ros2 launch graspnet_ros2 play_scene.launch.py scene_id:=3 camera:=kinect ann_id:=0
```

## パラメータ

- `root`: データルート。既定は `/data/graspnet`
- `split`: データ分割。既定は `train_1`
- `scene_id`: 再生したい scene 番号
- `camera`: `realsense` または `kinect`
- `frame_id`: 画像系の frame id
- `pointcloud_frame_id`: 点群の出力 frame id
- `use_camera_pose`: フレームごとの pose を使うかどうか
- `invert_camera_pose`: pose を逆行列にして使うかどうか
- `use_table_frame`: `cam0_wrt_table` を合成するかどうか
- `ann_id`: 1 枚だけ再生するフレーム番号
- `loop`: 最後のフレームまで行ったら先頭に戻るかどうか
- `hz`: 再生周波数
- `point_step`: PointCloud の間引き幅

`ann_id` は再生開始フレームです。scene 内のフレームを順番に流し、`loop:=true` なら最後まで行ったら最初に戻ります。

速度を変えるときは `hz` を指定します。

```bash
ros2 launch graspnet_ros2 play_scene.launch.py scene_id:=3 camera:=realsense ann_id:=0 hz:=2.0
```

## Topic

プレイヤーは以下を publish します。

- `/camera/camera/color/image_raw`
- `/camera/camera/aligned_depth_to_color/image_raw`
- `/camera/camera/color/camera_info`
- `/camera/camera/depth/color/points`
