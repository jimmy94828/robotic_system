// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from kachaka_interfaces:msg/KachakaCommand.idl
// generated code does not contain a copyright notice
#include "kachaka_interfaces/msg/detail/kachaka_command__rosidl_typesupport_fastrtps_cpp.hpp"
#include "kachaka_interfaces/msg/detail/kachaka_command__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace kachaka_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_kachaka_interfaces
cdr_serialize(
  const kachaka_interfaces::msg::KachakaCommand & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: command_type
  cdr << ros_message.command_type;
  // Member: move_shelf_command_target_shelf_id
  cdr << ros_message.move_shelf_command_target_shelf_id;
  // Member: move_shelf_command_destination_location_id
  cdr << ros_message.move_shelf_command_destination_location_id;
  // Member: move_shelf_command_undock_on_destination
  cdr << (ros_message.move_shelf_command_undock_on_destination ? true : false);
  // Member: return_shelf_command_target_shelf_id
  cdr << ros_message.return_shelf_command_target_shelf_id;
  // Member: undock_shelf_command_target_shelf_id
  cdr << ros_message.undock_shelf_command_target_shelf_id;
  // Member: move_to_location_command_target_location_id
  cdr << ros_message.move_to_location_command_target_location_id;
  // Member: return_home_command_silent
  cdr << (ros_message.return_home_command_silent ? true : false);
  // Member: speak_command_text
  cdr << ros_message.speak_command_text;
  // Member: move_to_pose_command_x
  cdr << ros_message.move_to_pose_command_x;
  // Member: move_to_pose_command_y
  cdr << ros_message.move_to_pose_command_y;
  // Member: move_to_pose_command_yaw
  cdr << ros_message.move_to_pose_command_yaw;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_kachaka_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  kachaka_interfaces::msg::KachakaCommand & ros_message)
{
  // Member: command_type
  cdr >> ros_message.command_type;

  // Member: move_shelf_command_target_shelf_id
  cdr >> ros_message.move_shelf_command_target_shelf_id;

  // Member: move_shelf_command_destination_location_id
  cdr >> ros_message.move_shelf_command_destination_location_id;

  // Member: move_shelf_command_undock_on_destination
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.move_shelf_command_undock_on_destination = tmp ? true : false;
  }

  // Member: return_shelf_command_target_shelf_id
  cdr >> ros_message.return_shelf_command_target_shelf_id;

  // Member: undock_shelf_command_target_shelf_id
  cdr >> ros_message.undock_shelf_command_target_shelf_id;

  // Member: move_to_location_command_target_location_id
  cdr >> ros_message.move_to_location_command_target_location_id;

  // Member: return_home_command_silent
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.return_home_command_silent = tmp ? true : false;
  }

  // Member: speak_command_text
  cdr >> ros_message.speak_command_text;

  // Member: move_to_pose_command_x
  cdr >> ros_message.move_to_pose_command_x;

  // Member: move_to_pose_command_y
  cdr >> ros_message.move_to_pose_command_y;

  // Member: move_to_pose_command_yaw
  cdr >> ros_message.move_to_pose_command_yaw;

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_kachaka_interfaces
get_serialized_size(
  const kachaka_interfaces::msg::KachakaCommand & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: command_type
  {
    size_t item_size = sizeof(ros_message.command_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: move_shelf_command_target_shelf_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.move_shelf_command_target_shelf_id.size() + 1);
  // Member: move_shelf_command_destination_location_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.move_shelf_command_destination_location_id.size() + 1);
  // Member: move_shelf_command_undock_on_destination
  {
    size_t item_size = sizeof(ros_message.move_shelf_command_undock_on_destination);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: return_shelf_command_target_shelf_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.return_shelf_command_target_shelf_id.size() + 1);
  // Member: undock_shelf_command_target_shelf_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.undock_shelf_command_target_shelf_id.size() + 1);
  // Member: move_to_location_command_target_location_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.move_to_location_command_target_location_id.size() + 1);
  // Member: return_home_command_silent
  {
    size_t item_size = sizeof(ros_message.return_home_command_silent);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: speak_command_text
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.speak_command_text.size() + 1);
  // Member: move_to_pose_command_x
  {
    size_t item_size = sizeof(ros_message.move_to_pose_command_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: move_to_pose_command_y
  {
    size_t item_size = sizeof(ros_message.move_to_pose_command_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: move_to_pose_command_yaw
  {
    size_t item_size = sizeof(ros_message.move_to_pose_command_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_kachaka_interfaces
max_serialized_size_KachakaCommand(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: command_type
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: move_shelf_command_target_shelf_id
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: move_shelf_command_destination_location_id
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: move_shelf_command_undock_on_destination
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: return_shelf_command_target_shelf_id
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: undock_shelf_command_target_shelf_id
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: move_to_location_command_target_location_id
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: return_home_command_silent
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: speak_command_text
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: move_to_pose_command_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: move_to_pose_command_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: move_to_pose_command_yaw
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = kachaka_interfaces::msg::KachakaCommand;
    is_plain =
      (
      offsetof(DataType, move_to_pose_command_yaw) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _KachakaCommand__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const kachaka_interfaces::msg::KachakaCommand *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _KachakaCommand__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<kachaka_interfaces::msg::KachakaCommand *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _KachakaCommand__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const kachaka_interfaces::msg::KachakaCommand *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _KachakaCommand__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_KachakaCommand(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _KachakaCommand__callbacks = {
  "kachaka_interfaces::msg",
  "KachakaCommand",
  _KachakaCommand__cdr_serialize,
  _KachakaCommand__cdr_deserialize,
  _KachakaCommand__get_serialized_size,
  _KachakaCommand__max_serialized_size
};

static rosidl_message_type_support_t _KachakaCommand__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_KachakaCommand__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace kachaka_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_kachaka_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<kachaka_interfaces::msg::KachakaCommand>()
{
  return &kachaka_interfaces::msg::typesupport_fastrtps_cpp::_KachakaCommand__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, kachaka_interfaces, msg, KachakaCommand)() {
  return &kachaka_interfaces::msg::typesupport_fastrtps_cpp::_KachakaCommand__handle;
}

#ifdef __cplusplus
}
#endif
