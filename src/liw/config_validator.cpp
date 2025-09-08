#include "liw/config_validator.h"

#include <XmlRpcValue.h>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
struct ParamSpec {
  std::string name;
  XmlRpc::XmlRpcValue::Type type;
};

std::string typeToString(XmlRpc::XmlRpcValue::Type type) {
  switch (type) {
    case XmlRpc::XmlRpcValue::TypeString:
      return "string";
    case XmlRpc::XmlRpcValue::TypeInt:
      return "integer";
    case XmlRpc::XmlRpcValue::TypeDouble:
      return "double";
    case XmlRpc::XmlRpcValue::TypeBoolean:
      return "boolean";
    case XmlRpc::XmlRpcValue::TypeArray:
      return "list";
    default:
      return "unknown";
  }
}
}  // namespace

void validateParameters(ros::NodeHandle& nh) {
  std::vector<ParamSpec> required{
      {"common/lidar_topic", XmlRpc::XmlRpcValue::TypeString},
      {"common/imu_topic", XmlRpc::XmlRpcValue::TypeString},
      {"common/image_topic", XmlRpc::XmlRpcValue::TypeString},
      {"common/image_type", XmlRpc::XmlRpcValue::TypeString},
      {"common/gravity_acc", XmlRpc::XmlRpcValue::TypeArray},
      {"lidar_parameter/lidar_type", XmlRpc::XmlRpcValue::TypeInt},
      {"imu_parameter/acc_cov", XmlRpc::XmlRpcValue::TypeDouble},
      {"camera_parameter/image_width", XmlRpc::XmlRpcValue::TypeInt},
      {"camera_parameter/image_height", XmlRpc::XmlRpcValue::TypeInt},
      {"extrinsic_parameter/extrinsic_enable", XmlRpc::XmlRpcValue::TypeBoolean},
  };

  for (const auto& spec : required) {
    XmlRpc::XmlRpcValue val;
    if (!nh.getParam(spec.name, val)) {
      throw std::runtime_error("Missing required parameter '" + spec.name + "'");
    }
    if (spec.type == XmlRpc::XmlRpcValue::TypeDouble) {
      if (val.getType() != XmlRpc::XmlRpcValue::TypeDouble && val.getType() != XmlRpc::XmlRpcValue::TypeInt) {
        throw std::runtime_error("Parameter '" + spec.name + "' has wrong type (expected number)");
      }
    } else if (val.getType() != spec.type) {
      throw std::runtime_error(
          "Parameter '" + spec.name + "' has wrong type (expected " + typeToString(spec.type) + ")");
    }
    if (spec.name == "common/gravity_acc") {
      if (val.size() != 3) {
        throw std::runtime_error("Parameter 'common/gravity_acc' must contain exactly three elements");
      }
    }
  }
}
