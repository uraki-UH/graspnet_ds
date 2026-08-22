#include <algorithm>
#include <chrono>
#include <cctype>
#include <array>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <ios>
#include <iostream>
#include <iterator>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>

namespace fs = std::filesystem;

namespace
{

struct PointXYZRGB
{
  float x;
  float y;
  float z;
  float rgb;
};

struct NpyArray
{
  std::vector<std::size_t> shape;
  std::vector<double> data;
};

std::string trim(std::string s)
{
  auto is_space = [](unsigned char c) { return std::isspace(c) != 0; };
  s.erase(s.begin(), std::find_if_not(s.begin(), s.end(), is_space));
  s.erase(std::find_if_not(s.rbegin(), s.rend(), is_space).base(), s.end());
  return s;
}

std::string extract_quoted_value(const std::string & header, const std::string & key)
{
  const std::size_t key_pos = header.find(key);
  if (key_pos == std::string::npos) {
    throw std::runtime_error("NPY header key not found: " + key);
  }

  const std::size_t first_quote = header.find('\'', key_pos + key.size());
  if (first_quote == std::string::npos) {
    throw std::runtime_error("NPY header quote not found for key: " + key);
  }

  const std::size_t second_quote = header.find('\'', first_quote + 1);
  if (second_quote == std::string::npos) {
    throw std::runtime_error("NPY header closing quote not found for key: " + key);
  }

  return header.substr(first_quote + 1, second_quote - first_quote - 1);
}

bool extract_bool_value(const std::string & header, const std::string & key)
{
  const std::size_t key_pos = header.find(key);
  if (key_pos == std::string::npos) {
    throw std::runtime_error("NPY header key not found: " + key);
  }

  const std::size_t value_pos = header.find_first_not_of(" \t", key_pos + key.size());
  if (value_pos == std::string::npos) {
    throw std::runtime_error("NPY header bool value not found for key: " + key);
  }

  if (header.compare(value_pos, 4, "True") == 0) {
    return true;
  }
  if (header.compare(value_pos, 5, "False") == 0) {
    return false;
  }

  throw std::runtime_error("NPY header bool value is invalid for key: " + key);
}

std::vector<std::size_t> extract_shape(const std::string & header)
{
  const std::size_t key_pos = header.find("'shape':");
  if (key_pos == std::string::npos) {
    throw std::runtime_error("NPY header shape key not found");
  }

  const std::size_t open_pos = header.find('(', key_pos);
  const std::size_t close_pos = header.find(')', open_pos);
  if (open_pos == std::string::npos || close_pos == std::string::npos || close_pos <= open_pos) {
    throw std::runtime_error("NPY header shape tuple is invalid");
  }

  std::vector<std::size_t> shape;
  std::stringstream ss(header.substr(open_pos + 1, close_pos - open_pos - 1));
  std::string token;
  while (std::getline(ss, token, ',')) {
    token = trim(token);
    if (!token.empty()) {
      shape.push_back(static_cast<std::size_t>(std::stoull(token)));
    }
  }

  if (shape.empty()) {
    throw std::runtime_error("NPY header shape is empty");
  }

  return shape;
}

NpyArray load_npy(const std::string & path)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    throw std::runtime_error("Failed to open npy file: " + path);
  }

  char magic[6];
  in.read(magic, 6);
  if (!in || std::memcmp(magic, "\x93NUMPY", 6) != 0) {
    throw std::runtime_error("Invalid npy magic: " + path);
  }

  unsigned char major = 0;
  unsigned char minor = 0;
  in.read(reinterpret_cast<char *>(&major), 1);
  in.read(reinterpret_cast<char *>(&minor), 1);
  if (!in) {
    throw std::runtime_error("Failed to read npy version: " + path);
  }

  std::uint32_t header_len = 0;
  if (major == 1) {
    std::uint16_t len16 = 0;
    in.read(reinterpret_cast<char *>(&len16), 2);
    header_len = len16;
  } else if (major == 2 || major == 3) {
    in.read(reinterpret_cast<char *>(&header_len), 4);
  } else {
    throw std::runtime_error("Unsupported npy version: " + std::to_string(major));
  }

  if (!in) {
    throw std::runtime_error("Failed to read npy header length: " + path);
  }

  std::string header(header_len, '\0');
  in.read(header.data(), static_cast<std::streamsize>(header_len));
  if (!in) {
    throw std::runtime_error("Failed to read npy header: " + path);
  }

  const std::string descr = extract_quoted_value(header, "'descr':");
  const bool fortran_order = extract_bool_value(header, "'fortran_order':");
  if (fortran_order) {
    throw std::runtime_error("Fortran-order npy arrays are not supported: " + path);
  }

  const std::vector<std::size_t> shape = extract_shape(header);
  std::size_t count = 1;
  for (std::size_t dim : shape) {
    count *= dim;
  }

  const std::size_t elem_size = [&]() -> std::size_t {
      if (descr == "<f8" || descr == "|f8") {
        return 8;
      }
      if (descr == "<f4" || descr == "|f4") {
        return 4;
      }
      throw std::runtime_error("Unsupported npy dtype: " + descr + " in " + path);
    }();

  std::vector<char> raw(count * elem_size);
  in.read(raw.data(), static_cast<std::streamsize>(raw.size()));
  if (!in) {
    throw std::runtime_error("Failed to read npy payload: " + path);
  }

  NpyArray array;
  array.shape = shape;
  array.data.resize(count);

  if (elem_size == 8) {
    for (std::size_t i = 0; i < count; ++i) {
      double value = 0.0;
      std::memcpy(&value, raw.data() + i * 8, 8);
      array.data[i] = value;
    }
  } else {
    for (std::size_t i = 0; i < count; ++i) {
      float value = 0.0f;
      std::memcpy(&value, raw.data() + i * 4, 4);
      array.data[i] = static_cast<double>(value);
    }
  }

  return array;
}

cv::Matx33d load_cam_k(const std::string & path)
{
  const NpyArray array = load_npy(path);
  if (array.shape.size() != 2 || array.shape[0] != 3 || array.shape[1] != 3) {
    throw std::runtime_error("camK shape must be 3x3: " + path);
  }

  cv::Matx33d cam_k;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      cam_k(r, c) = array.data[static_cast<std::size_t>(r) * 3 + static_cast<std::size_t>(c)];
    }
  }
  return cam_k;
}

cv::Matx44d load_mat44(const std::string & path)
{
  const NpyArray array = load_npy(path);
  if (array.shape.size() != 2 || array.shape[0] != 4 || array.shape[1] != 4) {
    throw std::runtime_error("Matrix shape must be 4x4: " + path);
  }

  cv::Matx44d mat;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      mat(r, c) = array.data[static_cast<std::size_t>(r) * 4 + static_cast<std::size_t>(c)];
    }
  }
  return mat;
}

std::vector<cv::Matx44d> load_camera_poses(const std::string & path)
{
  const NpyArray array = load_npy(path);
  if (array.shape.size() != 3 || array.shape[1] != 4 || array.shape[2] != 4) {
    throw std::runtime_error("camera_poses shape must be (N, 4, 4): " + path);
  }

  std::vector<cv::Matx44d> poses;
  poses.reserve(array.shape[0]);
  for (std::size_t i = 0; i < array.shape[0]; ++i) {
    cv::Matx44d pose;
    const std::size_t base = i * 16;
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        pose(r, c) = array.data[base + static_cast<std::size_t>(r) * 4 + static_cast<std::size_t>(c)];
      }
    }
    poses.push_back(pose);
  }
  return poses;
}

bool as_bool(const rclcpp::Parameter & param)
{
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
    return param.as_bool();
  }
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return param.as_int() != 0;
  }
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    return param.as_double() != 0.0;
  }
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
    std::string value = param.as_string();
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return value == "1" || value == "true" || value == "yes" || value == "on";
  }
  throw std::runtime_error("Unsupported bool parameter type");
}

int as_int(const rclcpp::Parameter & param)
{
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return static_cast<int>(param.as_int());
  }
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    return static_cast<int>(param.as_double());
  }
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
    return param.as_bool() ? 1 : 0;
  }
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
    return std::stoi(param.as_string());
  }
  throw std::runtime_error("Unsupported int parameter type");
}

double as_double(const rclcpp::Parameter & param)
{
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    return param.as_double();
  }
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return static_cast<double>(param.as_int());
  }
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
    return param.as_bool() ? 1.0 : 0.0;
  }
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
    return std::stod(param.as_string());
  }
  throw std::runtime_error("Unsupported double parameter type");
}

std::string as_string(const rclcpp::Parameter & param)
{
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
    return param.as_string();
  }
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
    return param.as_bool() ? "true" : "false";
  }
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return std::to_string(param.as_int());
  }
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    std::ostringstream oss;
    oss << std::setprecision(16) << param.as_double();
    return oss.str();
  }
  throw std::runtime_error("Unsupported string parameter type");
}

std::vector<int> select_playback_frame_ids(
  const std::vector<int> & frame_ids, int start, int end, int frame_start, int frame_end)
{
  if (frame_ids.empty()) {
    return {};
  }

  int playback_start = start >= 0 ? start : frame_start;
  int playback_end = end >= 0 ? end : frame_end;

  if (playback_start < 0 && playback_end < 0) {
    return frame_ids;
  }

  if (playback_start < 0) {
    playback_start = frame_ids.front();
  }
  if (playback_end < 0) {
    playback_end = frame_ids.back();
  }
  if (playback_start > playback_end) {
    std::swap(playback_start, playback_end);
  }

  std::vector<int> playback;
  for (int frame_id : frame_ids) {
    if (frame_id >= playback_start && frame_id <= playback_end) {
      playback.push_back(frame_id);
    }
  }
  return playback;
}

class GraspNetPlayerCpp : public rclcpp::Node
{
public:
  GraspNetPlayerCpp()
  : Node("graspnet_player_cpp")
  {
    declare_parameter("root", "/data/graspnet");
    declare_parameter("split", "train_1");
    declare_parameter("scene_id", 0);
    declare_parameter("camera", "realsense");
    declare_parameter("ann_id", 0);
    declare_parameter("start", -1);
    declare_parameter("end", -1);
    declare_parameter("frame_start", -1);
    declare_parameter("frame_end", -1);
    declare_parameter("frame_id", "camera_color_optical_frame");
    declare_parameter("pointcloud_frame_id", "graspnet_table");
    declare_parameter("use_camera_pose", true);
    declare_parameter("invert_camera_pose", false);
    declare_parameter("use_table_frame", true);
    declare_parameter("hz", 1.0);
    declare_parameter("point_step", 2);
    declare_parameter("loop", true);
    declare_parameter("publish_rgb", false);
    declare_parameter("publish_depth", false);
    declare_parameter("publish_camera_info", false);

    root_ = as_string(get_parameter("root"));
    split_ = as_string(get_parameter("split"));
    scene_id_ = as_int(get_parameter("scene_id"));
    camera_ = as_string(get_parameter("camera"));
    ann_id_ = as_int(get_parameter("ann_id"));
    start_ = as_int(get_parameter("start"));
    end_ = as_int(get_parameter("end"));
    frame_start_ = as_int(get_parameter("frame_start"));
    frame_end_ = as_int(get_parameter("frame_end"));
    frame_id_ = as_string(get_parameter("frame_id"));
    pointcloud_frame_id_ = as_string(get_parameter("pointcloud_frame_id"));
    use_camera_pose_ = as_bool(get_parameter("use_camera_pose"));
    invert_camera_pose_ = as_bool(get_parameter("invert_camera_pose"));
    use_table_frame_ = as_bool(get_parameter("use_table_frame"));
    hz_ = as_double(get_parameter("hz"));
    point_step_ = std::max(1, as_int(get_parameter("point_step")));
    loop_ = as_bool(get_parameter("loop"));
    publish_rgb_ = as_bool(get_parameter("publish_rgb"));
    publish_depth_ = as_bool(get_parameter("publish_depth"));
    publish_camera_info_ = as_bool(get_parameter("publish_camera_info"));

    base_dir_ = collect_frame_ids();
    if (frame_ids_.empty()) {
      throw std::runtime_error("No frames found for scene " + std::to_string(scene_id_) + " under " + root_);
    }

    play_frame_ids_ = select_playback_frame_ids(frame_ids_, start_, end_, frame_start_, frame_end_);
    if (play_frame_ids_.empty()) {
      throw std::runtime_error("No frames found in requested playback range");
    }

    auto it = std::find(play_frame_ids_.begin(), play_frame_ids_.end(), ann_id_);
    current_index_ = it == play_frame_ids_.end()
      ? 0
      : static_cast<std::size_t>(std::distance(play_frame_ids_.begin(), it));

    const double period = 1.0 / std::max(hz_, 0.001);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period),
      std::bind(&GraspNetPlayerCpp::publish_frame, this));

    RCLCPP_INFO(
      get_logger(),
      "GraspNet player started: root=%s split=%s scene_id=%d camera=%s ann_id=%d frame_count=%zu play_frame_count=%zu start=%d end=%d frame_start=%d frame_end=%d use_camera_pose=%s invert_camera_pose=%s use_table_frame=%s hz=%.3f point_step=%d publish_rgb=%s publish_depth=%s publish_camera_info=%s",
      root_.c_str(),
      split_.c_str(),
      scene_id_,
      camera_.c_str(),
      ann_id_,
      frame_ids_.size(),
      play_frame_ids_.size(),
      start_,
      end_,
      frame_start_,
      frame_end_,
      use_camera_pose_ ? "true" : "false",
      invert_camera_pose_ ? "true" : "false",
      use_table_frame_ ? "true" : "false",
      hz_,
      point_step_,
      publish_rgb_ ? "true" : "false",
      publish_depth_ ? "true" : "false",
      publish_camera_info_ ? "true" : "false");

    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/camera/camera/depth/color/points", 10);
  }

private:
  std::string get_split_dir()
  {
    const std::vector<std::string> candidates = split_ == "train_1"
      ? std::vector<std::string>{"train_1", "train1"}
      : split_ == "train1" ? std::vector<std::string>{"train1", "train_1"} : std::vector<std::string>{split_};

    for (const std::string & candidate : candidates) {
      const fs::path candidate_dir = fs::path(root_) / candidate;
      if (fs::is_directory(candidate_dir)) {
        return candidate_dir.string();
      }
    }

    throw std::runtime_error("Split directory not found under root: " + root_);
  }

  std::string collect_frame_ids()
  {
    const std::string split_dir = get_split_dir();
    const fs::path base = fs::path(split_dir) / ("scene_" + pad4(scene_id_)) / camera_;
    const fs::path rgb_dir = base / "rgb";
    const fs::path depth_dir = base / "depth";
    const fs::path camk_path = base / "camK.npy";
    const fs::path poses_path = base / "camera_poses.npy";
    const fs::path cam0_wrt_table_path = base / "cam0_wrt_table.npy";

    if (!fs::exists(camk_path)) {
      throw std::runtime_error("camK file not found: " + camk_path.string());
    }
    if (!fs::exists(poses_path)) {
      throw std::runtime_error("camera_poses file not found: " + poses_path.string());
    }
    if (!fs::exists(cam0_wrt_table_path)) {
      throw std::runtime_error("cam0_wrt_table file not found: " + cam0_wrt_table_path.string());
    }
    if (!fs::is_directory(rgb_dir)) {
      throw std::runtime_error("RGB directory not found: " + rgb_dir.string());
    }
    if (!fs::is_directory(depth_dir)) {
      throw std::runtime_error("Depth directory not found: " + depth_dir.string());
    }

    std::set<int> rgb_ids;
    for (const auto & entry : fs::directory_iterator(rgb_dir)) {
      if (entry.is_regular_file() && entry.path().extension() == ".png") {
        rgb_ids.insert(std::stoi(entry.path().stem().string()));
      }
    }

    std::set<int> depth_ids;
    for (const auto & entry : fs::directory_iterator(depth_dir)) {
      if (entry.is_regular_file() && entry.path().extension() == ".png") {
        depth_ids.insert(std::stoi(entry.path().stem().string()));
      }
    }

    frame_ids_.clear();
    std::set_intersection(
      rgb_ids.begin(), rgb_ids.end(),
      depth_ids.begin(), depth_ids.end(),
      std::back_inserter(frame_ids_));

    base_dir_ = base.string();
    camk_path_ = camk_path.string();
    poses_path_ = poses_path.string();
    cam0_wrt_table_path_ = cam0_wrt_table_path.string();
    return base_dir_;
  }

  static std::string pad4(int value)
  {
    std::ostringstream oss;
    oss << std::setw(4) << std::setfill('0') << value;
    return oss.str();
  }

  static std::string format_frame_path(const std::string & base_dir, const std::string & subdir, int frame_id)
  {
    std::ostringstream oss;
    oss << base_dir << "/" << subdir << "/" << pad4(frame_id) << ".png";
    return oss.str();
  }

  void ensure_calibration()
  {
    if (!cam_k_loaded_) {
      cam_k_ = load_cam_k(camk_path_);
      cam_k_loaded_ = true;
    }
    if (use_camera_pose_ && !poses_loaded_) {
      camera_poses_ = load_camera_poses(poses_path_);
      poses_loaded_ = true;
    }
    if (use_camera_pose_ && use_table_frame_ && !cam0_wrt_table_loaded_) {
      cam0_wrt_table_ = load_mat44(cam0_wrt_table_path_);
      cam0_wrt_table_loaded_ = true;
    }
  }

  cv::Matx44d get_camera_pose(std::size_t frame_index)
  {
    if (!use_camera_pose_) {
      return cv::Matx44d::eye();
    }
    ensure_calibration();
    if (frame_index >= camera_poses_.size()) {
      throw std::runtime_error("camera pose index out of range");
    }

    cv::Matx44d pose = camera_poses_[frame_index];
    if (invert_camera_pose_) {
      pose = pose.inv();
    }
    if (use_table_frame_) {
      pose = cam0_wrt_table_ * pose;
    }
    return pose;
  }

  std::vector<PointXYZRGB> make_pointcloud(
    const cv::Mat & bgr, const cv::Mat & depth, const cv::Matx33d & cam_k, const cv::Matx44d * camera_pose)
  {
    const float fx = static_cast<float>(cam_k(0, 0));
    const float fy = static_cast<float>(cam_k(1, 1));
    const float cx = static_cast<float>(cam_k(0, 2));
    const float cy = static_cast<float>(cam_k(1, 2));

    const int width = depth.cols;
    const int height = depth.rows;
    std::vector<PointXYZRGB> points;
    points.reserve(
      static_cast<std::size_t>((width / point_step_ + 1) * (height / point_step_ + 1)));

    for (int v = 0; v < height; v += point_step_) {
      const uint16_t * depth_row = depth.ptr<uint16_t>(v);
      const cv::Vec3b * bgr_row = bgr.ptr<cv::Vec3b>(v);
      for (int u = 0; u < width; u += point_step_) {
        const uint16_t z_mm = depth_row[u];
        if (z_mm == 0) {
          continue;
        }

        const float z = static_cast<float>(z_mm) / 1000.0f;
        float x = (static_cast<float>(u) - cx) * z / fx;
        float y = (static_cast<float>(v) - cy) * z / fy;

        if (camera_pose != nullptr) {
          const float world_x = static_cast<float>(
            (*camera_pose)(0, 0) * x + (*camera_pose)(0, 1) * y +
            (*camera_pose)(0, 2) * z + (*camera_pose)(0, 3));
          const float world_y = static_cast<float>(
            (*camera_pose)(1, 0) * x + (*camera_pose)(1, 1) * y +
            (*camera_pose)(1, 2) * z + (*camera_pose)(1, 3));
          const float world_z = static_cast<float>(
            (*camera_pose)(2, 0) * x + (*camera_pose)(2, 1) * y +
            (*camera_pose)(2, 2) * z + (*camera_pose)(2, 3));
          x = world_x;
          y = world_y;
          points.push_back(PointXYZRGB{x, y, world_z, pack_rgb(bgr_row[u])});
        } else {
          points.push_back(PointXYZRGB{x, y, z, pack_rgb(bgr_row[u])});
        }
      }
    }

    return points;
  }

  static float pack_rgb(const cv::Vec3b & bgr)
  {
    const std::uint32_t r = static_cast<std::uint32_t>(bgr[2]);
    const std::uint32_t g = static_cast<std::uint32_t>(bgr[1]);
    const std::uint32_t b = static_cast<std::uint32_t>(bgr[0]);
    const std::uint32_t packed = (r << 16) | (g << 8) | b;
    float rgb = 0.0f;
    std::memcpy(&rgb, &packed, sizeof(float));
    return rgb;
  }

  sensor_msgs::msg::PointCloud2 make_pointcloud2(
    const std_msgs::msg::Header & header, const std::vector<PointXYZRGB> & points)
  {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header = header;
    msg.height = 1;
    msg.width = static_cast<std::uint32_t>(points.size());
    msg.is_bigendian = false;
    msg.is_dense = false;
    msg.point_step = 16;
    msg.row_step = msg.point_step * msg.width;
    msg.fields.resize(4);

    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;

    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;

    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;

    msg.fields[3].name = "rgb";
    msg.fields[3].offset = 12;
    msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[3].count = 1;

    msg.data.resize(points.size() * sizeof(PointXYZRGB));
    if (!points.empty()) {
      std::memcpy(msg.data.data(), points.data(), msg.data.size());
    }

    return msg;
  }

  void publish_frame()
  {
    const int frame_id = play_frame_ids_.at(current_index_);
    const std::string rgb_path = format_frame_path(base_dir_, "rgb", frame_id);
    const std::string depth_path = format_frame_path(base_dir_, "depth", frame_id);

    auto rgb_future = std::async(std::launch::async, [&rgb_path]() {
        return cv::imread(rgb_path, cv::IMREAD_COLOR);
      });
    auto depth_future = std::async(std::launch::async, [&depth_path]() {
        return cv::imread(depth_path, cv::IMREAD_UNCHANGED);
      });
    cv::Mat rgb_bgr = rgb_future.get();
    cv::Mat depth = depth_future.get();
    if (rgb_bgr.empty()) {
      RCLCPP_ERROR(get_logger(), "Failed to read RGB image: %s", rgb_path.c_str());
      return;
    }

    if (depth.empty()) {
      RCLCPP_ERROR(get_logger(), "Failed to read depth image: %s", depth_path.c_str());
      return;
    }
    if (depth.type() != CV_16UC1) {
      RCLCPP_ERROR(get_logger(), "Depth image must be CV_16UC1: %s", depth_path.c_str());
      return;
    }
    ensure_calibration();

    const cv::Matx33d cam_k = cam_k_;
    cv::Matx44d camera_pose = cv::Matx44d::eye();
    const cv::Matx44d * camera_pose_ptr = nullptr;
    if (use_camera_pose_) {
      camera_pose = get_camera_pose(static_cast<std::size_t>(frame_id));
      camera_pose_ptr = &camera_pose;
    }

    const std::vector<PointXYZRGB> points = make_pointcloud(rgb_bgr, depth, cam_k, camera_pose_ptr);

    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = use_camera_pose_ ? pointcloud_frame_id_ : frame_id_;

    const sensor_msgs::msg::PointCloud2 msg = make_pointcloud2(header, points);
    points_pub_->publish(msg);

    ++current_index_;
    if (current_index_ >= play_frame_ids_.size()) {
      if (loop_) {
        current_index_ = 0;
      } else {
        RCLCPP_INFO(get_logger(), "Reached last frame, stopping playback");
        timer_->cancel();
      }
    }
  }

  std::shared_ptr<rclcpp::TimerBase> timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;

  std::string root_;
  std::string split_;
  int scene_id_ = 0;
  std::string camera_;
  int ann_id_ = 0;
  int start_ = -1;
  int end_ = -1;
  int frame_start_ = -1;
  int frame_end_ = -1;
  std::string frame_id_;
  std::string pointcloud_frame_id_;
  bool use_camera_pose_ = true;
  bool invert_camera_pose_ = false;
  bool use_table_frame_ = true;
  double hz_ = 1.0;
  int point_step_ = 2;
  bool loop_ = true;
  bool publish_rgb_ = false;
  bool publish_depth_ = false;
  bool publish_camera_info_ = false;

  std::string base_dir_;
  std::string camk_path_;
  std::string poses_path_;
  std::string cam0_wrt_table_path_;

  std::vector<int> frame_ids_;
  std::vector<int> play_frame_ids_;
  std::size_t current_index_ = 0;

  bool cam_k_loaded_ = false;
  bool poses_loaded_ = false;
  bool cam0_wrt_table_loaded_ = false;
  cv::Matx33d cam_k_ = cv::Matx33d::eye();
  std::vector<cv::Matx44d> camera_poses_;
  cv::Matx44d cam0_wrt_table_ = cv::Matx44d::eye();
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<GraspNetPlayerCpp>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
