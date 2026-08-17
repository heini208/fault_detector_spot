# Open3D geometry dependency

Probe reference-point projection and local surface-plane estimation use Open3D.

## Ubuntu 22.04 / ROS 2 Humble

Install the Ubuntu package before building or running the probe setup workflow:

```bash
sudo apt update
sudo apt install python3-open3d
```

Verify that the ROS Python interpreter can import it:

```bash
python3 -c 'import open3d; print(open3d.__version__)'
```

Ubuntu 22.04 provides Open3D 0.14.1. The implementation uses APIs available in that version:

- `open3d.geometry.PointCloud.create_from_depth_image`
- organized output with `project_valid_depth_only=False`
- `open3d.geometry.PointCloud.segment_plane`

`rosdep` does not currently define an Open3D Python key, so this dependency is intentionally documented as a manual system-package installation instead of being added as an unresolved `package.xml` dependency.

## What Open3D owns

The application keeps ROS message validation, RGB-to-registered-depth pixel mapping, probe-specific thresholds, normal direction conventions, and workflow state.

Open3D owns the generic 3D algorithms:

- calibrated depth-image back-projection into a point cloud
- RANSAC local plane segmentation

The resulting surface normal is still oriented toward the camera before the existing probe alignment logic receives it.
