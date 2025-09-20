#include "toolkit/loaders/bvh.hpp"
#include <cctype>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace toolkit::assets {

namespace detail {

// Trim string
inline std::string trim(const std::string &s) {
  size_t b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos)
    return "";
  size_t e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

// Convert Euler angles (BVH order) to quaternion
inline math::quat euler_to_quat(float x, float y, float z,
                                const std::vector<std::string> &order) {
  // BVH typically defines rotations in Zrotation / Xrotation / Yrotation order.
  // You must respect order.
  math::quat q = math::quat(1, 0, 0, 0); // identity
  for (const auto &ch : order) {
    if (ch == "Xrotation") {
      q = q * math::angle_axis(x, math::world_right);
    } else if (ch == "Yrotation") {
      q = q * math::angle_axis(y, math::world_up);
    } else if (ch == "Zrotation") {
      q = q * math::angle_axis(z, math::world_forward);
    }
  }
  return q;
}

} // namespace detail

// ----------------- LOADER -----------------
bvh_data load_bvh(std::string filepath) {
  std::ifstream in(filepath);
  if (!in) {
    throw std::runtime_error("Cannot open BVH file: " + filepath);
  }

  bvh_data data;
  std::vector<std::vector<std::string>> joint_channels; // per joint
  std::vector<int> stack;                               // hierarchy stack

  std::string line;
  int current_parent = -1;
  bool in_hierarchy = true;

  while (std::getline(in, line)) {
    line = detail::trim(line);
    if (line.empty())
      continue;

    std::istringstream iss(line);
    std::string token;
    iss >> token;

    if (token == "ROOT" || token == "JOINT") {
      std::string name;
      iss >> name;
      data.names.push_back(name);
      data.parents.push_back(current_parent);
      data.offsets.emplace_back(0, 0, 0);
      joint_channels.emplace_back();
      stack.push_back((int)data.names.size() - 1);
      current_parent = (int)data.names.size() - 1;
    } else if (token == "End") {
      std::string dummy;
      iss >> dummy; // Site
      data.names.push_back("EndSite_" + data.names[current_parent]);
      data.parents.push_back(current_parent);
      data.offsets.emplace_back(0, 0, 0);
      joint_channels.emplace_back();
      stack.push_back((int)data.names.size() - 1);
      current_parent = (int)data.names.size() - 1;
    } else if (token == "{") {
      // nothing
    } else if (token == "}") {
      stack.pop_back();
      if (!stack.empty())
        current_parent = stack.back();
    } else if (token == "OFFSET") {
      float x, y, z;
      iss >> x >> y >> z;
      data.offsets[current_parent] = math::vector3(x, y, z);
    } else if (token == "CHANNELS") {
      int count;
      iss >> count;
      for (int i = 0; i < count; i++) {
        std::string ch;
        iss >> ch;
        joint_channels[current_parent].push_back(ch);
      }
    } else if (token == "MOTION") {
      in_hierarchy = false;
      break;
    }
  }

  // Now read motion section
  int frame_count = 0;
  while (std::getline(in, line)) {
    line = detail::trim(line);
    if (line.empty())
      continue;
    std::istringstream iss(line);
    std::string token;
    iss >> token;

    if (token == "Frames:") {
      iss >> frame_count;
    } else if (token == "Frame") { // "Frame Time:"
      std::string dummy;
      iss >> dummy; // "Time:"
      iss >> data.frametime;
      break;
    }
  }

  if (frame_count <= 0)
    throw std::runtime_error("BVH has no frames");

  data.local_rot.resize(
      frame_count,
      std::vector<math::quat>(data.names.size(), math::quat(1, 0, 0, 0)));
  data.local_pos.resize(
      frame_count,
      std::vector<math::vector3>(data.names.size(), math::vector3(0, 0, 0)));

  // Read frame data
  for (int f = 0; f < frame_count; f++) {
    if (!std::getline(in, line))
      throw std::runtime_error("Unexpected EOF in BVH motion data");
    std::istringstream iss(line);

    for (size_t j = 0; j < data.names.size(); j++) {
      float x = 0, y = 0, z = 0;
      for (const auto &ch : joint_channels[j]) {
        float v;
        iss >> v;
        if (ch == "Xposition")
          x = v;
        else if (ch == "Yposition")
          y = v;
        else if (ch == "Zposition")
          z = v;
        else if (ch == "Xrotation")
          x = v * (float)3.14159265f / 180.0f;
        else if (ch == "Yrotation")
          y = v * (float)3.14159265f / 180.0f;
        else if (ch == "Zrotation")
          z = v * (float)3.14159265f / 180.0f;
      }
      // Save
      if (!joint_channels[j].empty()) {
        bool has_pos =
            (std::find(joint_channels[j].begin(), joint_channels[j].end(),
                       "Xposition") != joint_channels[j].end());
        if (has_pos) {
          data.local_pos[f][j] = math::vector3(x, y, z);
        } else {
          data.local_pos[f][j] = data.offsets[j];
        }
        // use euler_to_quat with channel order
        data.local_rot[f][j] =
            detail::euler_to_quat(x, y, z, joint_channels[j]);
      }
    }
  }

  return data;
}

// ----------------- SAVER -----------------
void save_bvh(std::string filepath, bvh_data &data, bool save_position) {
  std::ofstream out(filepath);
  if (!out)
    throw std::runtime_error("Cannot open file to write: " + filepath);

  // Hierarchy
  out << "HIERARCHY\n";
  std::function<void(int, int)> write_joint = [&](int idx, int depth) {
    std::string indent(depth * 2, ' ');
    if (data.parents[idx] == -1) {
      out << "ROOT " << data.names[idx] << "\n";
    } else if (data.names[idx].rfind("EndSite_", 0) == 0) {
      out << indent << "End Site\n";
    } else {
      out << indent << "JOINT " << data.names[idx] << "\n";
    }
    out << indent << "{\n";
    out << indent << "  OFFSET " << data.offsets[idx].x() << " "
        << data.offsets[idx].y() << " " << data.offsets[idx].z() << "\n";
    if (data.names[idx].rfind("EndSite_", 0) != 0) {
      out << indent << "  CHANNELS "
          << (save_position || data.parents[idx] == -1 ? 6 : 3);
      if (save_position || data.parents[idx] == -1)
        out << " Xposition Yposition Zposition Zrotation Yrotation Xrotation\n";
      else
        out << " Zrotation Yrotation Xrotation\n";
    }
    // children
    for (size_t j = 0; j < data.parents.size(); j++) {
      if (data.parents[j] == idx) {
        write_joint(j, depth + 1);
      }
    }
    out << indent << "}\n";
  };
  for (size_t i = 0; i < data.parents.size(); i++) {
    if (data.parents[i] == -1) {
      write_joint((int)i, 0);
    }
  }

  // Motion
  int frame_count = (int)data.local_rot.size();
  out << "MOTION\n";
  out << "Frames: " << frame_count << "\n";
  out << "Frame Time: " << std::fixed << std::setprecision(6) << data.frametime
      << "\n";

  for (int f = 0; f < frame_count; f++) {
    for (size_t j = 0; j < data.names.size(); j++) {
      if (data.parents[j] == -1 || save_position) {
        out << data.local_pos[f][j].x() << " " << data.local_pos[f][j].y()
            << " " << data.local_pos[f][j].z() << " ";
      }
      // Convert quat back to euler (XYZ order for now)
      math::vector3 euler = math::quat_to_euler(data.local_rot[f][j]);
      out << euler.x() * 180.0f / 3.14159265f << " "
          << euler.y() * 180.0f / 3.14159265f << " "
          << euler.z() * 180.0f / 3.14159265f << " ";
    }
    out << "\n";
  }
}

}; // namespace toolkit::assets