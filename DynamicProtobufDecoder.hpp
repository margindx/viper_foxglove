//
// Created by Hamza El-Kebir on 8/20/25.
//

#ifndef VIPER_DYNAMICPROTOBUFDECODER_HPP
#define VIPER_DYNAMICPROTOBUFDECODER_HPP

#include <iostream>
#include <string>
#include <vector>
#include <cstddef>
#include <memory>

#include <foxglove/server.hpp>

// Protobuf headers for dynamic compilation and reflection
#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/dynamic_message.h>
#include <google/protobuf/util/json_util.h>

namespace mdx {
    /// @brief A channel advertised by a client.
    struct ClientChannel {
        /// @brief The ID of the channel.
        uint32_t id;
        /// @brief The topic of the channel.
        std::string topic;
        /// @brief The encoding of the channel.
        std::string encoding;
        /// @brief The name of the schema of the channel.
        std::string schema_name;
        /// @brief The encoding of the schema of the channel.
        std::string schema_encoding;
        /// @brief The schema of the channel.
        std::vector<std::byte> schema;
        /// @brief The length of the schema of the channel.
        size_t schema_len;

        ClientChannel() : id(0), schema_len(0) {}

        ClientChannel(const foxglove::ClientChannel& channel);
    };
}


// A class to manage the dynamic decoding process
class DynamicProtobufDecoder {
public:
    const std::string encoding;

    // Constructor builds the schema database from the channel info
    DynamicProtobufDecoder(const mdx::ClientChannel& channel) : encoding(channel.encoding){
        // 1. Parse the schema (FileDescriptorSet) from the raw bytes
        google::protobuf::FileDescriptorSet file_descriptor_set;
        if (!file_descriptor_set.ParseFromArray(channel.schema.data(), channel.schema.size())) {
            throw std::runtime_error("Failed to parse FileDescriptorSet from schema data.");
        }

        // 2. Build the DescriptorPool from the parsed schema files.
        for (const auto& file_proto : file_descriptor_set.file()) {
            if (m_pool.BuildFile(file_proto) == nullptr) {
                std::cerr << "Warning: Could not build file " << file_proto.name()
                          << " in DescriptorPool. It might already exist or have errors." << std::endl;
            }
        }

        // 3. Find the specific MessageDescriptor for the message type we want to decode.
        m_message_descriptor = m_pool.FindMessageTypeByName(channel.schema_name);
        if (m_message_descriptor == nullptr) {
            throw std::runtime_error("Could not find message type '" + channel.schema_name + "' in the provided schema.");
        }

        // 4. Get the prototype message from the factory.
        m_prototype = m_factory.GetPrototype(m_message_descriptor);
        if (m_prototype == nullptr) {
            throw std::runtime_error("Failed to create a prototype for message type '" + channel.schema_name + "'.");
        }
    }

    DynamicProtobufDecoder(const DynamicProtobufDecoder&) = delete;
    DynamicProtobufDecoder& operator=(const DynamicProtobufDecoder&) = delete;

    std::unique_ptr<google::protobuf::Message> decode(const std::byte *data, const size_t data_len) {
        if (encoding == "protobuf") {
            return decodeProtobuf(data, data_len);
        } else {
            return decodeJson(data, data_len);
        }
    }

    std::unique_ptr<google::protobuf::Message> decode(const std::vector<std::byte>& data) {
        return decode(data.data(), data.size());
    }

    std::unique_ptr<google::protobuf::Message> decodeProtobuf(const std::vector<std::byte>& data) {
        return decodeProtobuf(data.data(), data.size());
    }

    // Decodes a raw BINARY message payload into a DynamicMessage
    std::unique_ptr<google::protobuf::Message> decodeProtobuf(const std::byte* data, const size_t data_len) {
        std::unique_ptr<google::protobuf::Message> message(m_prototype->New());
        if (!message->ParseFromArray(data, data_len)) {
            throw std::runtime_error("Failed to parse binary payload for message type '" + std::string(m_message_descriptor->full_name()) + "'.");
        }
        return message;
    }

    std::unique_ptr<google::protobuf::Message> decodeJson(const std::byte* data, const size_t data_len) {
        // Convert the byte vector to a string and call the other overload.
        std::string json_payload(reinterpret_cast<const char*>(data), data_len);
        return decodeJson(json_payload);
    }

    std::unique_ptr<google::protobuf::Message> decodeJson(const std::vector<std::byte>& data) {
        return decodeJson(data.data(), data.size());
    }

    // Decodes a JSON string payload into a DynamicMessage
    std::unique_ptr<google::protobuf::Message> decodeJson(const std::string& json_payload) {
        std::unique_ptr<google::protobuf::Message> message(m_prototype->New());

        // Use the JsonStringToMessage utility to parse the JSON.
        // It uses the message's descriptor to map keys to fields.
        auto status = google::protobuf::util::JsonStringToMessage(json_payload, message.get());

        if (!status.ok()) {
            throw std::runtime_error("Failed to parse JSON payload: " + status.ToString());
        }
        return message;
    }

private:
    google::protobuf::DescriptorPool m_pool;
    google::protobuf::DynamicMessageFactory m_factory{&m_pool};
    const google::protobuf::Descriptor* m_message_descriptor = nullptr;
    const google::protobuf::Message* m_prototype = nullptr;
};

// Helper to print a single field's value
void printFieldValue(const google::protobuf::Message& message, const google::protobuf::FieldDescriptor* field, std::stringstream& ss, int indent);

// Recursively builds a string representation of a message
void recursivePrint(const google::protobuf::Message& message, std::stringstream& ss, int indent);

// Generic function to pretty-print any decoded protobuf message.
void printDecodedMessage(const google::protobuf::Message& message);

// Generic template (default case)
template<typename T>
T decodeMessage(const google::protobuf::Message& message) {
    std::cout << "No specific decoder for type. Printing generic representation:\n";
    printDecodedMessage(message);
    // For a generic case, we can't return a specific type 'T',
    // so we'll just return a default-constructed one.
    return T{};
}

// Forward-declare explicit specializations so other TUs do not instantiate the
// primary template for these types (which would produce duplicate-symbol linker errors).
#ifdef _WIN32
#pragma push_macro("ERROR")
#pragma push_macro("DEBUG")
#pragma push_macro("WARNING")
#pragma push_macro("INFO")
#pragma push_macro("constant")
#undef ERROR
#undef DEBUG
#undef WARNING
#undef INFO
#undef constant
#endif
#include <foxglove/schemas.hpp>
#ifdef _WIN32
#pragma pop_macro("ERROR")
#pragma pop_macro("DEBUG")
#pragma pop_macro("WARNING")
#pragma pop_macro("INFO")
#pragma pop_macro("constant")
#endif

template<> foxglove::schemas::Vector3 decodeMessage<foxglove::schemas::Vector3>(const google::protobuf::Message&);
template<> foxglove::schemas::Quaternion decodeMessage<foxglove::schemas::Quaternion>(const google::protobuf::Message&);
template<> foxglove::schemas::Color decodeMessage<foxglove::schemas::Color>(const google::protobuf::Message&);
template<> foxglove::schemas::Pose decodeMessage<foxglove::schemas::Pose>(const google::protobuf::Message&);
template<> foxglove::schemas::Timestamp decodeMessage<foxglove::schemas::Timestamp>(const google::protobuf::Message&);
template<> foxglove::schemas::FrameTransform decodeMessage<foxglove::schemas::FrameTransform>(const google::protobuf::Message&);
template<> foxglove::schemas::CubePrimitive decodeMessage<foxglove::schemas::CubePrimitive>(const google::protobuf::Message&);


#endif //VIPER_DYNAMICPROTOBUFDECODER_HPP
