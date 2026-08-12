"""Tests for synchronized reference-view input validation."""

from array import array
from copy import deepcopy

import pytest
from fault_detector_msgs.msg import TagElement
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.model.models import (
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.setup.reference_view_validation import (
    ReferenceViewCaptureNotReady,
    validate_reference_view_inputs,
)


def make_image(encoding: str, bytes_per_pixel: int) -> Image:
    """Create a complete timestamped image."""
    image = Image()
    image.header.frame_id = "hand_color_image_sensor"
    image.header.stamp.sec = 10
    image.width = 4
    image.height = 3
    image.encoding = encoding
    image.step = image.width * bytes_per_pixel
    image.data = array("B", [0]) * (image.step * image.height)
    return image


def make_camera_info() -> CameraInfo:
    """Create matching camera calibration."""
    camera_info = CameraInfo()
    camera_info.header.frame_id = "hand_color_image_sensor"
    camera_info.header.stamp.sec = 10
    camera_info.width = 4
    camera_info.height = 3
    camera_info.k = [
        400.0,
        0.0,
        2.0,
        0.0,
        400.0,
        1.5,
        0.0,
        0.0,
        1.0,
    ]
    return camera_info


def make_tag() -> TagElement:
    """Create a timestamped base-camera reference tag."""
    tag = TagElement()
    tag.id = 7
    tag.pose.header.frame_id = "body"
    tag.pose.header.stamp.sec = 10
    tag.pose.pose.position.x = 1.0
    tag.pose.pose.orientation.w = 1.0
    return tag


def make_inputs():
    """Create one valid reference-view input set."""
    depth = make_image("16UC1", 2)
    depth.data = array("B", [232, 3]) * (
        depth.width * depth.height
    )
    return (
        make_image("rgb8", 3),
        depth,
        make_camera_info(),
        make_tag(),
        7,
        PoseData.identity(),
        "hand_color_image_sensor",
    )


def validate(
    inputs,
    maximum_timestamp_skew_sec=0.05,
) -> None:
    """Validate one mutable test input set."""
    validate_reference_view_inputs(
        *inputs,
        maximum_timestamp_skew_sec=maximum_timestamp_skew_sec,
        rgb_camera_info=inputs[2],
    )


def test_valid_reference_view_inputs_pass():
    """A synchronized complete input set is accepted."""
    validate(make_inputs())


@pytest.mark.parametrize("encoding", ["yuv422"])
def test_unsupported_rgb_encoding_is_rejected(encoding):
    """The reference image must use a supported color encoding."""
    inputs = list(make_inputs())
    inputs[0].encoding = encoding

    with pytest.raises(ValueError, match="RGB image encoding"):
        validate(inputs)


def test_mono8_reference_image_is_accepted():
    """Spot body-camera grayscale images remain selectable."""
    inputs = list(make_inputs())
    inputs[0] = make_image("mono8", 1)

    validate(inputs)


def test_unsupported_depth_encoding_is_rejected():
    """Registered depth must use a metric depth encoding."""
    inputs = list(make_inputs())
    inputs[1].encoding = "8UC1"

    with pytest.raises(ValueError, match="Depth image encoding"):
        validate(inputs)


def test_incomplete_image_data_is_rejected():
    """Truncated image data cannot be captured."""
    inputs = list(make_inputs())
    inputs[1].data = inputs[1].data[:-1]

    with pytest.raises(ValueError, match="Depth image data"):
        validate(inputs)


def test_empty_registered_depth_is_temporarily_not_ready():
    """A later depth frame may provide the required surface support."""
    inputs = list(make_inputs())
    inputs[1].data = array("B", [0]) * (
        inputs[1].step * inputs[1].height
    )

    with pytest.raises(
        ReferenceViewCaptureNotReady,
        match="valid depth support",
    ):
        validate(inputs)


def test_mismatched_dimensions_are_rejected():
    """Registered depth must align with the reference image."""
    inputs = list(make_inputs())
    inputs[1].width = 3
    inputs[1].step = 6
    inputs[1].data = array("B", [0]) * 18

    with pytest.raises(ValueError, match="dimensions must match"):
        validate(inputs)


def test_mismatched_frames_are_rejected():
    """Registered inputs must describe one optical frame."""
    inputs = list(make_inputs())
    inputs[1].header.frame_id = "hand_depth_image_sensor"

    with pytest.raises(ValueError, match="frames must match"):
        validate(inputs)


def test_timestamp_skew_is_temporarily_not_ready():
    """A later synchronized pair may satisfy the timestamp-skew gate."""
    inputs = list(make_inputs())
    inputs[1].header.stamp.nanosec = 60_000_000

    with pytest.raises(
        ReferenceViewCaptureNotReady,
        match="allowed skew",
    ):
        validate(inputs)


def test_timestamp_skew_boundary_is_accepted():
    """Inputs exactly at the configured tolerance are accepted."""
    inputs = list(make_inputs())
    inputs[1].header.stamp.nanosec = 50_000_000

    validate(inputs)


def test_cached_camera_info_is_accepted():
    """Valid cached calibration does not need an image timestamp."""
    inputs = list(make_inputs())
    inputs[2].header.stamp.sec = 0

    validate(inputs)


def test_zero_timestamp_is_rejected():
    """Unstamped input data cannot define a capture time."""
    inputs = list(make_inputs())
    inputs[0].header.stamp.sec = 0

    with pytest.raises(ValueError, match="must not be zero"):
        validate(inputs)


def test_wrong_reference_tag_is_rejected():
    """The camera dataset must use the object's selected tag."""
    inputs = list(make_inputs())
    inputs[3].id = 8

    with pytest.raises(ValueError, match="does not match"):
        validate(inputs)


def test_tag_timestamp_skew_is_not_camera_pairing_criterion():
    """TF resolves the tag and camera at their own timestamps."""
    inputs = list(make_inputs())
    inputs[3].pose.header.stamp.nanosec = 300_000_000

    validate(inputs)


def test_zero_tag_timestamp_is_rejected():
    """An unstamped tag cannot anchor the captured image."""
    inputs = list(make_inputs())
    inputs[3].pose.header.stamp.sec = 0

    with pytest.raises(ValueError, match="tag observation timestamp"):
        validate(inputs)


def test_empty_tag_observation_frame_is_rejected():
    """A tag pose without its source frame cannot be transformed."""
    inputs = list(make_inputs())
    inputs[3].pose.header.frame_id = ""

    with pytest.raises(ValueError, match="observation frame"):
        validate(inputs)


def test_invalid_tag_pose_is_rejected():
    """Invalid base-tag geometry cannot define the object frame."""
    inputs = list(make_inputs())
    inputs[3].pose.pose.orientation.w = 2.0

    with pytest.raises(ValueError, match="normalized"):
        validate(inputs)


@pytest.mark.parametrize(
    "intrinsics, message",
    [
        ([0.0, 0.0, 2.0, 0.0, 400.0, 1.5, 0.0, 0.0, 1.0],
         "focal lengths"),
        ([400.0, 0.0, float("nan"), 0.0, 400.0, 1.5, 0.0, 0.0, 1.0],
         "non-finite"),
    ],
)
def test_invalid_camera_intrinsics_are_rejected(
    intrinsics,
    message,
):
    """Projection requires finite positive camera intrinsics."""
    inputs = list(make_inputs())
    inputs[2].k = intrinsics

    with pytest.raises(ValueError, match=message):
        validate(inputs)


def test_empty_controlled_frame_is_rejected():
    """The saved overview pose must identify its controlled frame."""
    inputs = list(make_inputs())
    inputs[6] = " "

    with pytest.raises(ValueError, match="Controlled frame"):
        validate(inputs)


def test_non_camera_controlled_frame_is_rejected():
    """The stored pose must describe the image optical frame."""
    inputs = list(make_inputs())
    inputs[6] = "arm0.link_wr1"

    with pytest.raises(ValueError, match="RGB optical frame"):
        validate(inputs)


def test_invalid_controlled_frame_pose_is_rejected():
    """The overview pose must be finite and normalized."""
    inputs = list(make_inputs())
    inputs[5] = PoseData(
        position=Vector3Data.zero(),
        orientation=QuaternionData(
            x=0.0,
            y=0.0,
            z=0.0,
            w=2.0,
        ),
    )

    with pytest.raises(ValueError, match="normalized"):
        validate(inputs)


def test_validation_does_not_modify_inputs():
    """Validation leaves temporary capture inputs unchanged."""
    inputs = make_inputs()
    before = deepcopy(inputs)

    validate(inputs)

    assert inputs == before
