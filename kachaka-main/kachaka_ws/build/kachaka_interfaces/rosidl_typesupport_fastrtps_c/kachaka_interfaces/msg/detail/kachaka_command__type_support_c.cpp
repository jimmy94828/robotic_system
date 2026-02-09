// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from kachaka_interfaces:msg/KachakaCommand.idl
// generated code does not contain a copyright notice
#include "kachaka_interfaces/msg/detail/kachaka_command__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "kachaka_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "kachaka_interfaces/msg/detail/kachaka_command__struct.h"
#include "kachaka_interfaces/msg/detail/kachaka_command__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // move_shelf_command_destination_location_id, move_shelf_command_target_shelf_id, move_to_location_command_target_location_id, return_shelf_command_target_shelf_id, speak_command_text, undock_shelf_command_target_shelf_id
#include "rosidl_runtime_c/string_functions.h"  // move_shelf_command_destination_location_id, move_shelf_command_target_shelf_id, move_to_location_command_target_location_id, return_shelf_command_target_shelf_id, speak_command_text, undock_shelf_command_target_shelf_id

// forward declare type support functions


using _KachakaCommand__ros_msg_type = kachaka_interfaces__msg__KachakaCommand;

static bool _KachakaCommand__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _KachakaCommand__ros_msg_type * ros_message = static_cast<const _KachakaCommand__ros_msg_type *>(untyped_ros_message);
  // Field name: command_type
  {
    cdr << ros_message->command_type;
  }

  // Field name: move_shelf_command_target_shelf_id
  {
    const rosidl_runtime_c__String * str = &ros_message->move_shelf_command_target_shelf_id;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: move_shelf_command_destination_location_id
  {
    const rosidl_runtime_c__String * str = &ros_message->move_shelf_command_destination_location_id;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: move_shelf_command_undock_on_destination
  {
    cdr << (ros_message->move_shelf_command_undock_on_destination ? true : false);
  }

  // Field name: return_shelf_command_target_shelf_id
  {
    const rosidl_runtime_c__String * str = &ros_message->return_shelf_command_target_shelf_id;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: undock_shelf_command_target_shelf_id
  {
    const rosidl_runtime_c__String * str = &ros_message->undock_shelf_command_target_shelf_id;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: move_to_location_command_target_location_id
  {
    const rosidl_runtime_c__String * str = &ros_message->move_to_location_command_target_location_id;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: return_home_command_silent
  {
    cdr << (ros_message->return_home_command_silent ? true : false);
  }

  // Field name: speak_command_text
  {
    const rosidl_runtime_c__String * str = &ros_message->speak_command_text;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: move_to_pose_command_x
  {
    cdr << ros_message->move_to_pose_command_x;
  }

  // Field name: move_to_pose_command_y
  {
    cdr << ros_message->move_to_pose_command_y;
  }

  // Field name: move_to_pose_command_yaw
  {
    cdr << ros_message->move_to_pose_command_yaw;
  }

  return true;
}

static bool _KachakaCommand__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _KachakaCommand__ros_msg_type * ros_message = static_cast<_KachakaCommand__ros_msg_type *>(untyped_ros_message);
  // Field name: command_type
  {
    cdr >> ros_message->command_type;
  }

  // Field name: move_shelf_command_target_shelf_id
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->move_shelf_command_target_shelf_id.data) {
      rosidl_runtime_c__String__init(&ros_message->move_shelf_command_target_shelf_id);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->move_shelf_command_target_shelf_id,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'move_shelf_command_target_shelf_id'\n");
      return false;
    }
  }

  // Field name: move_shelf_command_destination_location_id
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->move_shelf_command_destination_location_id.data) {
      rosidl_runtime_c__String__init(&ros_message->move_shelf_command_destination_location_id);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->move_shelf_command_destination_location_id,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'move_shelf_command_destination_location_id'\n");
      return false;
    }
  }

  // Field name: move_shelf_command_undock_on_destination
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->move_shelf_command_undock_on_destination = tmp ? true : false;
  }

  // Field name: return_shelf_command_target_shelf_id
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->return_shelf_command_target_shelf_id.data) {
      rosidl_runtime_c__String__init(&ros_message->return_shelf_command_target_shelf_id);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->return_shelf_command_target_shelf_id,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'return_shelf_command_target_shelf_id'\n");
      return false;
    }
  }

  // Field name: undock_shelf_command_target_shelf_id
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->undock_shelf_command_target_shelf_id.data) {
      rosidl_runtime_c__String__init(&ros_message->undock_shelf_command_target_shelf_id);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->undock_shelf_command_target_shelf_id,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'undock_shelf_command_target_shelf_id'\n");
      return false;
    }
  }

  // Field name: move_to_location_command_target_location_id
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->move_to_location_command_target_location_id.data) {
      rosidl_runtime_c__String__init(&ros_message->move_to_location_command_target_location_id);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->move_to_location_command_target_location_id,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'move_to_location_command_target_location_id'\n");
      return false;
    }
  }

  // Field name: return_home_command_silent
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->return_home_command_silent = tmp ? true : false;
  }

  // Field name: speak_command_text
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->speak_command_text.data) {
      rosidl_runtime_c__String__init(&ros_message->speak_command_text);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->speak_command_text,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'speak_command_text'\n");
      return false;
    }
  }

  // Field name: move_to_pose_command_x
  {
    cdr >> ros_message->move_to_pose_command_x;
  }

  // Field name: move_to_pose_command_y
  {
    cdr >> ros_message->move_to_pose_command_y;
  }

  // Field name: move_to_pose_command_yaw
  {
    cdr >> ros_message->move_to_pose_command_yaw;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_kachaka_interfaces
size_t get_serialized_size_kachaka_interfaces__msg__KachakaCommand(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _KachakaCommand__ros_msg_type * ros_message = static_cast<const _KachakaCommand__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name command_type
  {
    size_t item_size = sizeof(ros_message->command_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name move_shelf_command_target_shelf_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->move_shelf_command_target_shelf_id.size + 1);
  // field.name move_shelf_command_destination_location_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->move_shelf_command_destination_location_id.size + 1);
  // field.name move_shelf_command_undock_on_destination
  {
    size_t item_size = sizeof(ros_message->move_shelf_command_undock_on_destination);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name return_shelf_command_target_shelf_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->return_shelf_command_target_shelf_id.size + 1);
  // field.name undock_shelf_command_target_shelf_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->undock_shelf_command_target_shelf_id.size + 1);
  // field.name move_to_location_command_target_location_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->move_to_location_command_target_location_id.size + 1);
  // field.name return_home_command_silent
  {
    size_t item_size = sizeof(ros_message->return_home_command_silent);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name speak_command_text
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->speak_command_text.size + 1);
  // field.name move_to_pose_command_x
  {
    size_t item_size = sizeof(ros_message->move_to_pose_command_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name move_to_pose_command_y
  {
    size_t item_size = sizeof(ros_message->move_to_pose_command_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name move_to_pose_command_yaw
  {
    size_t item_size = sizeof(ros_message->move_to_pose_command_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _KachakaCommand__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_kachaka_interfaces__msg__KachakaCommand(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_kachaka_interfaces
size_t max_serialized_size_kachaka_interfaces__msg__KachakaCommand(
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

  // member: command_type
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: move_shelf_command_target_shelf_id
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
  // member: move_shelf_command_destination_location_id
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
  // member: move_shelf_command_undock_on_destination
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: return_shelf_command_target_shelf_id
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
  // member: undock_shelf_command_target_shelf_id
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
  // member: move_to_location_command_target_location_id
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
  // member: return_home_command_silent
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: speak_command_text
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
  // member: move_to_pose_command_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: move_to_pose_command_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: move_to_pose_command_yaw
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
    using DataType = kachaka_interfaces__msg__KachakaCommand;
    is_plain =
      (
      offsetof(DataType, move_to_pose_command_yaw) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _KachakaCommand__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_kachaka_interfaces__msg__KachakaCommand(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_KachakaCommand = {
  "kachaka_interfaces::msg",
  "KachakaCommand",
  _KachakaCommand__cdr_serialize,
  _KachakaCommand__cdr_deserialize,
  _KachakaCommand__get_serialized_size,
  _KachakaCommand__max_serialized_size
};

static rosidl_message_type_support_t _KachakaCommand__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_KachakaCommand,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, kachaka_interfaces, msg, KachakaCommand)() {
  return &_KachakaCommand__type_support;
}

#if defined(__cplusplus)
}
#endif
