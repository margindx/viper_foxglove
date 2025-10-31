//
// Created by Hamza El-Kebir on 8/20/25.
//

#include "DynamicProtobufDecoder.hpp"

mdx::ClientChannel::ClientChannel(const foxglove::ClientChannel &channel) {
    this->id = channel.id;
    this->topic = std::string(channel.topic);
    this->encoding = std::string(channel.encoding);
    this->schema_name = std::string(channel.schema_name);
    this->schema_encoding = std::string(channel.schema_encoding);
    if (channel.schema && channel.schema_len > 0) {
        this->schema = std::vector<std::byte>(channel.schema, channel.schema + channel.schema_len);
    }
    this->schema_len = channel.schema_len;
}

void recursivePrint(const google::protobuf::Message &message, std::stringstream &ss, int indent) {
    const auto* reflection = message.GetReflection();
    std::vector<const google::protobuf::FieldDescriptor*> fields;
    reflection->ListFields(message, &fields);

    for (const auto* field : fields) {
        ss << std::string(indent * 2, ' ') << field->name() << ": ";
        if (field->is_repeated()) {
            ss << "[\n";
            int count = reflection->FieldSize(message, field);
            for (int i = 0; i < count; ++i) {
                ss << std::string((indent + 1) * 2, ' ');
                // Use GetRepeated... methods for repeated fields
                // This part would need to be expanded similarly to printFieldValue for all types
                ss << reflection->GetRepeatedString(message, field, i) << ",\n";
            }
            ss << std::string(indent * 2, ' ') << "]";
        } else {
            printFieldValue(message, field, ss, indent);
        }
        ss << "\n";
    }
}

void printDecodedMessage(const google::protobuf::Message &message) {
    std::stringstream ss;
    ss << message.GetDescriptor()->name() << " {\n";
    recursivePrint(message, ss, 1);
    ss << "}\n";
    std::cout << ss.str();
}

void printFieldValue(const google::protobuf::Message &message, const google::protobuf::FieldDescriptor *field,
                     std::stringstream &ss, int indent) {
    const auto* reflection = message.GetReflection();
    switch (field->cpp_type()) {
        case google::protobuf::FieldDescriptor::CPPTYPE_INT32:   ss << reflection->GetInt32(message, field); break;
        case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:  ss << reflection->GetUInt32(message, field); break;
        case google::protobuf::FieldDescriptor::CPPTYPE_INT64:   ss << reflection->GetInt64(message, field); break;
        case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:  ss << reflection->GetUInt64(message, field); break;
        case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:   ss << reflection->GetFloat(message, field); break;
        case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:  ss << reflection->GetDouble(message, field); break;
        case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:    ss << (reflection->GetBool(message, field) ? "true" : "false"); break;
        case google::protobuf::FieldDescriptor::CPPTYPE_STRING:  ss << "\"" << reflection->GetString(message, field) << "\""; break;
        case google::protobuf::FieldDescriptor::CPPTYPE_ENUM:    ss << reflection->GetEnum(message, field)->name(); break;
        case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
            ss << "{\n";
            recursivePrint(reflection->GetMessage(message, field), ss, indent + 1);
            ss << std::string(indent * 2, ' ') << "}";
            break;
    }
}

template<>
foxglove::schemas::Vector3 decodeMessage<foxglove::schemas::Vector3>(const google::protobuf::Message& message) {
    if (message.GetDescriptor()->full_name() != "foxglove.Vector3") {
        throw std::runtime_error("Message is not a foxglove.Vector3");
    }

    foxglove::schemas::Vector3 vec;
    const auto* vec_refl = message.GetReflection();
    const auto* vec_desc = message.GetDescriptor();
    vec.x = vec_refl->GetDouble(message, vec_desc->FindFieldByName("x"));
    vec.y = vec_refl->GetDouble(message, vec_desc->FindFieldByName("y"));
    vec.z = vec_refl->GetDouble(message, vec_desc->FindFieldByName("z"));
    return vec;
}

template<>
foxglove::schemas::Quaternion decodeMessage<foxglove::schemas::Quaternion>(const google::protobuf::Message& message) {
    if (message.GetDescriptor()->full_name() != "foxglove.Quaternion") {
        throw std::runtime_error("Message is not a foxglove.Quaternion");
    }

    foxglove::schemas::Quaternion quat;
    const auto* quat_refl = message.GetReflection();
    const auto* quat_desc = message.GetDescriptor();
    quat.x = quat_refl->GetDouble(message, quat_desc->FindFieldByName("x"));
    quat.y = quat_refl->GetDouble(message, quat_desc->FindFieldByName("y"));
    quat.z = quat_refl->GetDouble(message, quat_desc->FindFieldByName("z"));
    quat.w = quat_refl->GetDouble(message, quat_desc->FindFieldByName("w"));
    return quat;
}

template<>
foxglove::schemas::Color decodeMessage<foxglove::schemas::Color>(const google::protobuf::Message& message) {
    if (message.GetDescriptor()->full_name() != "foxglove.Color") {
        throw std::runtime_error("Message is not a foxglove.Color");
    }

    foxglove::schemas::Color color;
    const auto* color_refl = message.GetReflection();
    const auto* color_desc = message.GetDescriptor();
    color.r = color_refl->GetDouble(message, color_desc->FindFieldByName("r"));
    color.g = color_refl->GetDouble(message, color_desc->FindFieldByName("g"));
    color.b = color_refl->GetDouble(message, color_desc->FindFieldByName("b"));
    color.a = color_refl->GetDouble(message, color_desc->FindFieldByName("a"));
    return color;
}

template<>
foxglove::schemas::Pose decodeMessage<foxglove::schemas::Pose>(const google::protobuf::Message& message) {
    if (message.GetDescriptor()->full_name() != "foxglove.Pose") {
        throw std::runtime_error("Message is not a foxglove.Pose");
    }

    foxglove::schemas::Pose pose;
    const auto* pose_refl = message.GetReflection();
    const auto* pose_desc = message.GetDescriptor();

    if (pose_refl->HasField(message, pose_desc->FindFieldByName("position"))) {
        const auto &pos_msg = pose_refl->GetMessage(message, pose_desc->FindFieldByName("position"));
        pose.position.emplace(decodeMessage<foxglove::schemas::Vector3>(pos_msg));
    }

    if (pose_refl->HasField(message, pose_desc->FindFieldByName("orientation"))) {
        const auto &ori_msg = pose_refl->GetMessage(message, pose_desc->FindFieldByName("orientation"));
        pose.orientation.emplace(decodeMessage<foxglove::schemas::Quaternion>(ori_msg));
    }

    return pose;
}

template<>
foxglove::schemas::Timestamp decodeMessage<foxglove::schemas::Timestamp>(const google::protobuf::Message& message) {
    if (message.GetDescriptor()->full_name() != "foxglove.Timestamp") {
        throw std::runtime_error("Message is not a foxglove.Timestamp");
    }

    foxglove::schemas::Timestamp time{};
    const auto* time_refl = message.GetReflection();
    const auto* time_desc = message.GetDescriptor();
    time.sec = time_refl->GetInt64(message, time_desc->FindFieldByName("seconds"));
    time.nsec = time_refl->GetInt32(message, time_desc->FindFieldByName("nanos"));
    return time;
}

template<>
foxglove::schemas::FrameTransform decodeMessage<foxglove::schemas::FrameTransform>(const google::protobuf::Message& message) {
    if (message.GetDescriptor()->full_name() != "foxglove.FrameTransform") {
        throw std::runtime_error("Message is not a foxglove.FrameTransform");
    }

    foxglove::schemas::FrameTransform tf;
    const auto* tf_refl = message.GetReflection();
    const auto* tf_desc = message.GetDescriptor();
    const auto& timestamp_msg = tf_refl->GetMessage(message, tf_desc->FindFieldByName("timestamp"));

    if (tf_refl->HasField(message, tf_desc->FindFieldByName("timestamp"))) {
        auto timestamp = decodeMessage<foxglove::schemas::Timestamp>(timestamp_msg);
        tf.timestamp = timestamp;
    }

    auto parent_frame_id = tf_refl->GetString(message, tf_desc->FindFieldByName("parent_frame_id"));
    auto child_frame_id = tf_refl->GetString(message, tf_desc->FindFieldByName("child_frame_id"));

    if (tf_refl->HasField(message, tf_desc->FindFieldByName("translation"))) {
        const auto &translation_msg = tf_refl->GetMessage(message, tf_desc->FindFieldByName("translation"));
        auto translation = decodeMessage<foxglove::schemas::Vector3>(translation_msg);
    }

    if (tf_refl->HasField(message, tf_desc->FindFieldByName("rotation"))) {
        const auto &rotation_msg = tf_refl->GetMessage(message, tf_desc->FindFieldByName("rotation"));
        auto rotation = decodeMessage<foxglove::schemas::Quaternion>(rotation_msg);
    }

    return tf;
}

// Template specialization for CubePrimitive
template<>
foxglove::schemas::CubePrimitive decodeMessage<foxglove::schemas::CubePrimitive>(const google::protobuf::Message& message) {
    if (message.GetDescriptor()->full_name() != "foxglove.CubePrimitive") {
        throw std::runtime_error("Message is not a foxglove.CubePrimitive");
    }

    foxglove::schemas::CubePrimitive cube;
    const auto* cube_refl = message.GetReflection();
    const auto* cube_desc = message.GetDescriptor();

    // Decode Pose
    if (cube_refl->HasField(message, cube_desc->FindFieldByName("pose"))) {
        const auto *pose_field = cube_desc->FindFieldByName("pose");
        const auto &pose_msg = cube_refl->GetMessage(message, pose_field);
        cube.pose.emplace(decodeMessage<foxglove::schemas::Pose>(pose_msg));
    }

    // Decode Size
    if (cube_refl->HasField(message, cube_desc->FindFieldByName("size"))) {
        const auto *size_field = cube_desc->FindFieldByName("size");
        const auto &size_msg = cube_refl->GetMessage(message, size_field);
        cube.size.emplace(decodeMessage<foxglove::schemas::Vector3>(size_msg));
    }

    // Decode Color
    if (cube_refl->HasField(message, cube_desc->FindFieldByName("size"))) {
        const auto *color_field = cube_desc->FindFieldByName("color");
        const auto &color_msg = cube_refl->GetMessage(message, color_field);
        cube.color.emplace(decodeMessage<foxglove::schemas::Color>(color_msg));
    }

    return cube;
}
