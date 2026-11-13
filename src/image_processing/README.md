# image_processing

ROS 2 lifecycle component `image_undistort::ImageUndistort`. It undistorts camera images (pinhole or
fisheye model) using the intrinsics from the first `CameraInfo` message, then republishes them.

| Parameter | Default | Description |
|---|---|---|
| `input_image_topic` | `image` | Input image topic. |
| `camera_info_topic` | `camera_info` | Camera intrinsics, read once. |
| `output_image_topic` | `image_rect` | Undistorted output image. |
| `resize_output` | `false` | Halve the output resolution. |
| `display_latency` | `false` | Log header stamp → publish latency (throttled). |

## Latency

**Setup:** RTX 3060 Laptop / i7-11800H, Release build, 960x544 `rgba8` distorted input at ~13 Hz, `resize_output: false`,
composable container with intra-process comms.

| Stage | Value |
|---|---|
| Undistortion processing (`image_callback`) | ~10 ms |

Only the processing time inside the node is listed. The `display_latency` log measures from the
image header stamp, so it also includes upstream latency (driver / bag recording offset and
transport). It does not reflect this node's cost.
