# SciPy rotation dependency

Surface-facing probe orientation and generic quaternion/rotation conversion use
`scipy.spatial.transform.Rotation`.

## Ubuntu 22.04 / ROS 2 Humble

Install the Ubuntu package:

```bash
sudo apt update
sudo apt install python3-scipy
```

Verify it with:

```bash
python3 -c 'from scipy.spatial.transform import Rotation; print(Rotation.identity().as_quat())'
```

The application keeps the probe-specific orientation policy: local +X points
toward the surface and the secondary axis remains as upright as the surface
normal permits. SciPy owns the generic quaternion, rotation-matrix, Euler-angle,
and vector-rotation operations.

The ROS rosdep database does not currently provide a `python3-scipy` key, so
this dependency is documented as a system-package installation instead of
adding an unresolved `package.xml` dependency.
