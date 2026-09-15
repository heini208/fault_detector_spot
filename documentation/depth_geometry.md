# Depth geometry

Registered depth is projected into camera coordinates with NumPy pinhole equations.
The organized point cloud preserves pixel layout, invalid-depth masks, byte order,
and row padding for ROS `16UC1` and `32FC1` images. Processed camera intrinsics
come from `CameraInfo.p`, falling back to `CameraInfo.k` when P is empty.
Projection matrices with translation are rejected.

Surface planes use three-point RANSAC with a local fixed random seed, followed
by SVD refinement of the selected inliers. Existing inlier-count, inlier-ratio,
tangent-spread, and caller-specific fit-error checks remain in place. Surface
normals are oriented toward the observing camera.

There is one NumPy implementation and no additional point-cloud library to install.
