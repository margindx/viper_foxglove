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
