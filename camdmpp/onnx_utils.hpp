#pragma once

#include "toolkit/math.hpp"
#include "toolkit/utils.hpp"
#include <onnxruntime_cxx_api.h>

// Helper function to convert tensor element data type enum to a string
const char *tensor_element_data_type_as_str(ONNXTensorElementDataType type);

// Helper function to print details for either inputs or outputs
void print_onnx_node_details(Ort::Session &session,
                             Ort::AllocatorWithDefaultOptions &allocator,
                             bool is_input);

toolkit::math::quat repr6d_to_quat(const std::array<float, 6> &data);
std::array<float, 6> quat_to_repr6d(const toolkit::math::quat &data);