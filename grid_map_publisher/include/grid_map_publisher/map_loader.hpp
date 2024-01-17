#pragma once
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "Magick++.h"
#include "yaml-cpp/yaml.h"
#ifndef _WIN32
#include <libgen.h>
#else
#error no files <libgen.h>
#endif

class MapLoader {
public:
  MapLoader(rclcpp::Node &node) : node_(node) { grid_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(); }
  ~MapLoader() = default;

  bool load_file(std::string file_name, std::string map_topic_name, bool is_publish = true) {
    exist_map_data_ = load_map_file(file_name);
    if (is_publish)
      map_publisher_ = node_.create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_name, rclcpp::QoS(10));
    return exist_map_data_;
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr get_map() { return grid_map_; }
  bool exist_map_data() { return exist_map_data_; }
  void publish_map() {
    if (exist_map_data_ && map_publisher_) {
      map_publisher_->publish(*grid_map_);
    }
  }

private:
  bool exist_map_data_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;

  enum class MapMode {
    /**
     * Together with associated threshold values (occupied and free):
     *   lightness >= occupied threshold - Occupied (100)
     *             ... (anything in between) - Unknown (-1)
     *    lightness <= free threshold - Free (0)
     */
    Trinary,
    /**
     * Together with associated threshold values (occupied and free):
     *   alpha < 1.0 - Unknown (-1)
     *   lightness >= occ_th - Occupied (100)
     *             ... (linearly interpolate to)
     *   lightness <= free_th - Free (0)
     */
    Scale,
    /**
     * Lightness = 0 - Free (0)
     *          ... (linearly interpolate to)
     * Lightness = 100 - Occupied (100)
     * Lightness >= 101 - Unknown
     */
    Raw,
  };
  struct LoadParameters {
    std::string image_file_name;
    double resolution{0};
    std::vector<double> origin{0, 0, 0};
    double free_thresh;
    double occupied_thresh;
    MapMode mode;
    bool negate;
  };
  static constexpr int8_t OCC_GRID_UNKNOWN = -1;
  static constexpr int8_t OCC_GRID_FREE = 0;
  static constexpr int8_t OCC_GRID_OCCUPIED = 100;
  inline geometry_msgs::msg::Quaternion orientationAroundZAxis(double angle) {
    tf2::Quaternion q;
    q.setRPY(0, 0, angle); // void returning function
    return tf2::toMsg(q);
  }
  const char *map_mode_to_string(MapMode map_mode) {
    switch (map_mode) {
    case MapMode::Trinary:
      return "trinary";
    case MapMode::Scale:
      return "scale";
    case MapMode::Raw:
      return "raw";
    default:
      throw std::invalid_argument("map_mode");
    }
  }
  MapMode map_mode_from_string(std::string map_mode_name) {
    for (auto &c : map_mode_name) {
      c = tolower(c);
    }

    if (map_mode_name == "scale") {
      return MapMode::Scale;
    } else if (map_mode_name == "raw") {
      return MapMode::Raw;
    } else if (map_mode_name == "trinary") {
      return MapMode::Trinary;
    } else {
      throw std::invalid_argument("map_mode_name");
    }
  }
  nav_msgs::msg::OccupancyGrid::SharedPtr grid_map_;
  bool load_map_file(std::string yaml_file_name);
  LoadParameters loadMapYaml(const std::string &yaml_filename);
  void loadMapFromFile(const LoadParameters &load_parameters, nav_msgs::msg::OccupancyGrid &map);

  rclcpp::Node &node_;
};

bool MapLoader::load_map_file(std::string yaml_file) {
  if (yaml_file.empty()) {
    return false;
  }

  std::cout << "[INFO] [load_map_file]: Loading yaml file: " << yaml_file << std::endl;
  LoadParameters load_parameters;
  try {
    load_parameters = loadMapYaml(yaml_file);
  } catch (YAML::Exception &e) {
    std::cerr << "[ERROR] [map_io]: Failed processing YAML file " << yaml_file << " at position (" << e.mark.line << ":"
              << e.mark.column << ") for reason: " << e.what() << std::endl;
    return false;
  } catch (std::exception &e) {
    std::cerr << "[ERROR] [map_io]: Failed to parse map YAML loaded from file " << yaml_file
              << " for reason: " << e.what() << std::endl;
    return false;
  }

  try {
    loadMapFromFile(load_parameters, *grid_map_);
  } catch (std::exception &e) {
    std::cerr << "[ERROR] [map_io]: Failed to load image file " << load_parameters.image_file_name
              << " for reason: " << e.what() << std::endl;
    return false;
  }

  return true;
}

template <typename T> T yaml_get_value(const YAML::Node &node, const std::string &key) {
  try {
    return node[key].as<T>();
  } catch (YAML::Exception &e) {
    std::stringstream ss;
    ss << "Failed to parse YAML tag '" << key << "' for reason: " << e.msg;
    throw YAML::Exception(e.mark, ss.str());
  }
}

MapLoader::LoadParameters MapLoader::loadMapYaml(const std::string &yaml_filename) {
  YAML::Node doc = YAML::LoadFile(yaml_filename);

  LoadParameters load_parameters;

  auto image_file_name = yaml_get_value<std::string>(doc, "image");
  if (image_file_name.empty()) {
    throw YAML::Exception(doc["image"].Mark(), "The image tag was empty.");
  }
  if (image_file_name[0] != '/') {
    // dirname takes a mutable char *, so we copy into a vector
    std::vector<char> fname_copy(yaml_filename.begin(), yaml_filename.end());
    fname_copy.push_back('\0');
    image_file_name = std::string(dirname(fname_copy.data())) + '/' + image_file_name;
  }
  load_parameters.image_file_name = image_file_name;

  load_parameters.resolution = yaml_get_value<double>(doc, "resolution");
  load_parameters.origin = yaml_get_value<std::vector<double>>(doc, "origin");
  if (load_parameters.origin.size() != 3) {
    throw YAML::Exception(doc["origin"].Mark(), "value of the 'origin' tag should have 3 elements, not " +
                                                    std::to_string(load_parameters.origin.size()));
  }

  load_parameters.free_thresh = yaml_get_value<double>(doc, "free_thresh");
  load_parameters.occupied_thresh = yaml_get_value<double>(doc, "occupied_thresh");

  auto map_mode_node = doc["mode"];
  if (!map_mode_node.IsDefined()) {
    load_parameters.mode = MapMode::Trinary;
  } else {
    load_parameters.mode = map_mode_from_string(map_mode_node.as<std::string>());
  }

  try {
    load_parameters.negate = yaml_get_value<int>(doc, "negate");
  } catch (YAML::Exception &) {
    load_parameters.negate = yaml_get_value<bool>(doc, "negate");
  }

  std::cout << "[DEBUG] [map_io]: resolution: " << load_parameters.resolution << std::endl;
  std::cout << "[DEBUG] [map_io]: origin[0]: " << load_parameters.origin[0] << std::endl;
  std::cout << "[DEBUG] [map_io]: origin[1]: " << load_parameters.origin[1] << std::endl;
  std::cout << "[DEBUG] [map_io]: origin[2]: " << load_parameters.origin[2] << std::endl;
  std::cout << "[DEBUG] [map_io]: free_thresh: " << load_parameters.free_thresh << std::endl;
  std::cout << "[DEBUG] [map_io]: occupied_thresh: " << load_parameters.occupied_thresh << std::endl;
  std::cout << "[DEBUG] [map_io]: mode: " << map_mode_to_string(load_parameters.mode) << std::endl;
  std::cout << "[DEBUG] [map_io]: negate: " << load_parameters.negate << std::endl; // NOLINT

  return load_parameters;
}

void MapLoader::loadMapFromFile(const LoadParameters &load_parameters, nav_msgs::msg::OccupancyGrid &map) {
  Magick::InitializeMagick(nullptr);
  nav_msgs::msg::OccupancyGrid msg;

  std::cout << "[INFO] [map_io]: Loading image_file: " << load_parameters.image_file_name << std::endl;
  Magick::Image img(load_parameters.image_file_name);

  // Copy the image data into the map structure
  msg.info.width = img.size().width();
  msg.info.height = img.size().height();

  msg.info.resolution = load_parameters.resolution;
  msg.info.origin.position.x = load_parameters.origin[0];
  msg.info.origin.position.y = load_parameters.origin[1];
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation = orientationAroundZAxis(load_parameters.origin[2]);

  // Allocate space to hold the data
  msg.data.resize(msg.info.width * msg.info.height);

  // Copy pixel data into the map structure
  for (size_t y = 0; y < msg.info.height; y++) {
    for (size_t x = 0; x < msg.info.width; x++) {
      auto pixel = img.pixelColor(x, y);

      std::vector<Magick::Quantum> channels = {pixel.redQuantum(), pixel.greenQuantum(), pixel.blueQuantum()};
      if (load_parameters.mode == MapMode::Trinary && img.matte()) {
        // To preserve existing behavior, average in alpha with color channels in Trinary mode.
        // CAREFUL. alpha is inverted from what you might expect. High = transparent, low = opaque
        channels.push_back(MaxRGB - pixel.alphaQuantum());
      }
      double sum = 0;
      for (auto c : channels) {
        sum += c;
      }
      /// on a scale from 0.0 to 1.0 how bright is the pixel?
      double shade = Magick::ColorGray::scaleQuantumToDouble(sum / channels.size());

      // If negate is true, we consider blacker pixels free, and whiter
      // pixels occupied. Otherwise, it's vice versa.
      /// on a scale from 0.0 to 1.0, how occupied is the map cell (before thresholding)?
      double occ = (load_parameters.negate ? shade : 1.0 - shade);

      int8_t map_cell;
      switch (load_parameters.mode) {
      case MapMode::Trinary:
        if (load_parameters.occupied_thresh < occ) {
          map_cell = OCC_GRID_OCCUPIED;
        } else if (occ < load_parameters.free_thresh) {
          map_cell = OCC_GRID_FREE;
        } else {
          map_cell = OCC_GRID_UNKNOWN;
        }
        break;
      case MapMode::Scale:
        if (pixel.alphaQuantum() != OpaqueOpacity) {
          map_cell = OCC_GRID_UNKNOWN;
        } else if (load_parameters.occupied_thresh < occ) {
          map_cell = OCC_GRID_OCCUPIED;
        } else if (occ < load_parameters.free_thresh) {
          map_cell = OCC_GRID_FREE;
        } else {
          map_cell = std::rint((occ - load_parameters.free_thresh) /
                               (load_parameters.occupied_thresh - load_parameters.free_thresh) * 100.0);
        }
        break;
      case MapMode::Raw: {
        double occ_percent = std::round(shade * 255);
        if (OCC_GRID_FREE <= occ_percent && occ_percent <= OCC_GRID_OCCUPIED) {
          map_cell = static_cast<int8_t>(occ_percent);
        } else {
          map_cell = OCC_GRID_UNKNOWN;
        }
        break;
      }
      default:
        throw std::runtime_error("Invalid map mode");
      }
      msg.data[msg.info.width * (msg.info.height - y - 1) + x] = map_cell;
    }
  }

  // Since loadMapFromFile() does not belong to any node, publishing in a system time.
  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  msg.info.map_load_time = clock.now();
  msg.header.frame_id = "map";
  msg.header.stamp = clock.now();

  std::cout << "[DEBUG] [map_io]: Read map " << load_parameters.image_file_name << ": " << msg.info.width << " X "
            << msg.info.height << " map @ " << msg.info.resolution << " m/cell" << std::endl;

  map = msg;
}