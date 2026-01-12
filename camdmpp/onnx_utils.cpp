#include "onnx_utils.hpp"
#include <iostream>

using namespace toolkit::math;

const char *tensor_element_data_type_as_str(ONNXTensorElementDataType type) {
  switch (type) {
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:
    return "float32";
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8:
    return "uint8";
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8:
    return "int8";
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16:
    return "uint16";
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT16:
    return "int16";
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32:
    return "int32";
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64:
    return "int64";
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_STRING:
    return "string";
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_BOOL:
    return "bool";
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16:
    return "float16";
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE:
    return "float64";
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT32:
    return "uint32";
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT64:
    return "uint64";
  default:
    return "unknown";
  }
}

void print_onnx_node_details(Ort::Session &session,
                      Ort::AllocatorWithDefaultOptions &allocator,
                      bool is_input) {
  size_t node_count =
      is_input ? session.GetInputCount() : session.GetOutputCount();
  std::cout << "--- " << (is_input ? "Inputs" : "Outputs") << " (" << node_count
            << ") ---" << std::endl;

  for (size_t i = 0; i < node_count; ++i) {
    // Get node name
    Ort::AllocatedStringPtr name_ptr =
        is_input ? session.GetInputNameAllocated(i, allocator)
                 : session.GetOutputNameAllocated(i, allocator);
    std::string node_name = name_ptr.get();

    // Get node type and shape information
    Ort::TypeInfo type_info =
        is_input ? session.GetInputTypeInfo(i) : session.GetOutputTypeInfo(i);
    auto tensor_info = type_info.GetTensorTypeAndShapeInfo();

    // Get data type
    ONNXTensorElementDataType data_type = tensor_info.GetElementType();
    std::string type_str = tensor_element_data_type_as_str(data_type);

    // Get shape
    std::vector<int64_t> shape = tensor_info.GetShape();

    // Print the details
    std::cout << "[" << i << "] Name: " << node_name << std::endl;
    std::cout << "    Type: " << type_str << std::endl;
    std::cout << "    Shape: [";
    for (size_t j = 0; j < shape.size(); ++j) {
      // A value of -1 indicates a dynamic dimension (e.g., batch size)
      if (shape[j] == -1) {
        std::cout << "dynamic";
      } else {
        std::cout << shape[j];
      }
      if (j < shape.size() - 1) {
        std::cout << ", ";
      }
    }
    std::cout << "]" << std::endl;
  }
  std::cout << std::endl;
}

quat repr6d_to_quat(const std::array<float, 6> &data) {
  vector3 x = vector3(data[0], data[1], data[2]).normalized();
  vector3 y = vector3(data[3], data[4], data[5]).normalized();
  vector3 z = x.cross(y).normalized();
  y = z.cross(x).normalized();
  matrix3 mat;
  mat.row(0) = x;
  mat.row(1) = y;
  mat.row(2) = z;
  return quat(mat);
}

std::array<float, 6> quat_to_repr6d(const quat &data) {
  std::array<float, 6> ret;
  auto mat = data.normalized().toRotationMatrix();
  ret[0] = mat(0,0);
  ret[1] = mat(0,1);
  ret[2] = mat(0,2);
  ret[3] = mat(1,0);
  ret[4] = mat(1,1);
  ret[5] = mat(1,2);
  return ret;
}