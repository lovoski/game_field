#pragma once

#include "toolkit/utils.hpp"
#include <onnxruntime_cxx_api.h>

// Helper function to convert tensor element data type enum to a string
const char *tensor_element_data_type_as_str(ONNXTensorElementDataType type);

// Helper function to print details for either inputs or outputs
void print_onnx_node_details(Ort::Session &session,
                             Ort::AllocatorWithDefaultOptions &allocator,
                             bool is_input);
