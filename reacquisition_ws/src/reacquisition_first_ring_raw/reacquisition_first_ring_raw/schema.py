"""schema.py

只定义字段名、默认值和常量, 不包含算法、ROS import 或 C++ 相关内容。

依据:
- 工程文档/第一环_最简原始数据中间件理论文档_v0.1.md
- 工程文档/低智能AI_Windows11无仿真无C++生成指南.md
"""

# --- 中间件协议元数据 -----------------------------------------------------

# JSON schema 版本号, 字段语义一旦发布, 不应随意改动.
RAW_FRAME_SCHEMA_VERSION = "0.1"

# 第一环最简原始数据中间件发布的话题名 (未来 ROS 2 适配用).
RAW_FRAME_TOPIC = "/reacquisition/first_ring/raw_frame"

# payload 编码格式, 当前固定为 JSON.
RAW_FRAME_ENCODING = "JSON"

# payload 在 ROS 2 下的消息类型 (当前 Windows 最简版不实际发布, 仅记录).
RAW_FRAME_ROS_TYPE = "std_msgs/msg/String"

# 第一环对第二环的输出接口 (不属于本中间件输入, 仅在文档中引用).
TARGET_OBSERVATION_TOPIC = "/reacquisition/target_observation_frame"
TARGET_OBSERVATION_ROS_TYPE = "std_msgs/msg/String"


# --- 坐标约定 (UAV 状态 raw_frame_convention 取值) -----------------------

COORD_CONVENTION_ENU_FLU = "ENU_FLU"
COORD_CONVENTION_NED_FRD = "NED_FRD"
COORD_CONVENTION_UNKNOWN = "UNKNOWN"

ALLOWED_COORD_CONVENTIONS = (
    COORD_CONVENTION_ENU_FLU,
    COORD_CONVENTION_NED_FRD,
    COORD_CONVENTION_UNKNOWN,
)


# --- 数据来源字符串 (source 取值) ----------------------------------------

SRC_MOCK_UAV_STATE = "mock_uav_state"
SRC_MOCK_TARGET_STATE = "mock_target_state"
SRC_MOCK_IMAGE = "mock_image"
SRC_MOCK_CAMERA_INFO = "mock_camera_info"
SRC_PX4_VEHICLE_ODOMETRY = "px4_vehicle_odometry"
SRC_GAZEBO_MODEL_STATE = "gazebo_model_state"
SRC_GAZEBO_GROUND_TRUTH = "gazebo_ground_truth"
SRC_UNAVAILABLE = "unavailable"

ALLOWED_UAV_SOURCES = (
    SRC_MOCK_UAV_STATE,
    SRC_PX4_VEHICLE_ODOMETRY,
    SRC_GAZEBO_MODEL_STATE,
)

ALLOWED_TARGET_SOURCES = (
    SRC_MOCK_TARGET_STATE,
    SRC_GAZEBO_GROUND_TRUTH,
    SRC_UNAVAILABLE,
)

ALLOWED_IMAGE_SOURCES = (
    SRC_MOCK_IMAGE,
)

ALLOWED_CAMERA_INFO_SOURCES = (
    SRC_MOCK_CAMERA_INFO,
)


# --- 顶层 JSON 必需字段 --------------------------------------------------

REQUIRED_TOP_LEVEL_FIELDS = (
    "schema_version",
    "header",
    "source_sequence",
    "config_revision",
    "uav_state",
    "target_state",
    "image",
    "camera_info",
    "middleware",
)

# 顶层 header.stamp 优先取自图像采集时间, 缺图时使用中间件封装时间.
PRIMARY_TIME_SOURCE_IMAGE = "image.header.stamp"
PRIMARY_TIME_SOURCE_MIDDLEWARE = "middleware.stamp"

ALLOWED_PRIMARY_TIME_SOURCES = (
    PRIMARY_TIME_SOURCE_IMAGE,
    PRIMARY_TIME_SOURCE_MIDDLEWARE,
)


# --- 数据传输模式 --------------------------------------------------------

DATA_TRANSPORT_METADATA_ONLY = "metadata_only"
DATA_TRANSPORT_ROS_TOPIC_REFERENCE = "ros_topic_reference"
DATA_TRANSPORT_BASE64_INLINE_DEBUG = "base64_inline_debug"

ALLOWED_DATA_TRANSPORTS = (
    DATA_TRANSPORT_METADATA_ONLY,
    DATA_TRANSPORT_ROS_TOPIC_REFERENCE,
    DATA_TRANSPORT_BASE64_INLINE_DEBUG,
)


# --- 图像编码 (与 sensor_msgs/Image 兼容) --------------------------------

IMAGE_ENCODING_RGB8 = "rgb8"
IMAGE_ENCODING_BGR8 = "bgr8"
IMAGE_ENCODING_MONO8 = "mono8"

ALLOWED_IMAGE_ENCODINGS = (
    IMAGE_ENCODING_RGB8,
    IMAGE_ENCODING_BGR8,
    IMAGE_ENCODING_MONO8,
)


# --- 相机畸变模型 --------------------------------------------------------

DISTORTION_MODEL_PLUMB_BOB = "plumb_bob"
DISTORTION_MODEL_RATIONAL_POLYNOMIAL = "rational_polynomial"

ALLOWED_DISTORTION_MODELS = (
    DISTORTION_MODEL_PLUMB_BOB,
    DISTORTION_MODEL_RATIONAL_POLYNOMIAL,
)


# --- 生产者 / 环境标签 (Windows 最简版固定值) -----------------------------

PRODUCER_WINDOWS_MOCK = "windows_mock_generator"
ENVIRONMENT_WINDOWS_NO_SIM_NO_CPP = "windows11_no_sim_no_cpp"


# --- 禁止输出字段 --------------------------------------------------------
# 理论文档 § 5.2 / 生成指南 § 5.2 明确禁止 Windows 最简版计算或输出这些字段.
# 任何一环算法或后续观测输出均不得在本中间件中伪造.
FORBIDDEN_OUTPUT_FIELDS = (
    "bbox_left",
    "bbox_top",
    "bbox_right",
    "bbox_bottom",
    "center_u",
    "center_v",
    "foot_u",
    "foot_v",
    "ground_position",
    "ground_covariance_xy",
    "reliability",
    "blur_risk",
    "edge_distance_px",
)


# --- 默认 frame_id -------------------------------------------------------

DEFAULT_GLOBAL_FRAME_ID = "map"
DEFAULT_CAMERA_OPTICAL_FRAME_ID = "camera_optical_frame"

# 默认图像话题引用 (mock 场景使用, 与理论文档保持一致).
DEFAULT_IMAGE_TOPIC = "/camera/image_raw"
DEFAULT_CAMERA_INFO_TOPIC = "/camera/camera_info"

# 默认图像尺寸 (与生成指南示例一致).
DEFAULT_IMAGE_HEIGHT = 720
DEFAULT_IMAGE_WIDTH = 1280
DEFAULT_IMAGE_STEP = 3840  # 1280 * 3 字节 (rgb8 / bgr8)
