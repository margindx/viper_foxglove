//
// Created by Hamza El-Kebir on 7/22/25.
//

#ifndef VIPER_FOXGLOVEINTERFACE_HPP
#define VIPER_FOXGLOVEINTERFACE_HPP

#include <foxglove/server.hpp>
#include <foxglove/schemas.hpp>
#include <foxglove/mcap.hpp>
#include <nlohmann/json.hpp>
#include <nlohmann/jsonschema.hpp>
#include <Eigen/Dense>

#include "FoxgloveUtility.hpp"
#include "FoxgloveMacros.hpp"

#include <thread>
#include <optional>
#include <cmath>
#include <iomanip>

using namespace std::literals::chrono_literals;

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

        ClientChannel() {}

        ClientChannel(const foxglove::ClientChannel& channel) {
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
    };

    struct Point3 {
        float x;
        float y;
        float z;
    };

    template <typename T>
    struct Point6 {
        T x, y, z;
        T nx, ny, nz;

        static Point6<T> fromPose(const foxglove::schemas::PoseInFrame &pose) {
            Eigen::Vector3<T> pos{
                static_cast<T>(pose.pose->position.value().x),
                static_cast<T>(pose.pose->position.value().y),
                static_cast<T>(pose.pose->position.value().z)
            };

            Eigen::Quaternion<T> quat{
                    static_cast<T>(pose.pose->orientation.value().x),
                    static_cast<T>(pose.pose->orientation.value().y),
                    static_cast<T>(pose.pose->orientation.value().z),
                    static_cast<T>(pose.pose->orientation.value().w)
            };

            Eigen::Vector3<T> unitVec{1, 0, 0};

            T qx = static_cast<T>(pose.pose->orientation.value().x);
            T qy = static_cast<T>(pose.pose->orientation.value().y);
            T qz = static_cast<T>(pose.pose->orientation.value().z);
            T qw = static_cast<T>(pose.pose->orientation.value().w);

            Eigen::Vector3<T> nor = {
                1 - 2*(qy*qy + qz*qz),
                2*(qx*qy + qw*qz),
                2*(qx*qz - qw*qy),
            };

            return {
                pos.x(),
                pos.y(),
                pos.z(),
                nor.x(),
                nor.y(),
                nor.z()
            };
        }
    };

    using Point6d = Point6<double>;
    using Point6f = Point6<float>;

    template <typename T>
    struct Point6View {
        // The type for our points map, which is a 3-column matrix with a custom stride.
        using PointsMap = Eigen::Map<const Eigen::Matrix<T, -1, 3, Eigen::RowMajor>, Eigen::Unaligned, Eigen::OuterStride<1>>;

        // The type for our normals map, which is also a 3-column matrix with a custom stride.
        // The stride is 6, since we need to jump over the x, y, z components of the point.
        using NormalsMap = Eigen::Map<const Eigen::Matrix<T, -1, 3, Eigen::RowMajor>, Eigen::Unaligned, Eigen::OuterStride<6>>;

        // An implicitly converting constructor that takes a const reference to the vector.
        // It creates and stores the Eigen::Map objects.
        Point6View(const std::vector<Point6<T>>& data)
                : points(reinterpret_cast<const T*>(data.data()), data.size(), 3, Eigen::OuterStride<6>(6)),
                  normals(reinterpret_cast<const T*>(data.data()) + 3, data.size(), 3, Eigen::OuterStride<6>(6)) {}

        // Expose the maps as public members.
        PointsMap points;
        NormalsMap normals;
    };

    struct RawForce {
        float f1;
        float f2;
        float f3;
        float f4;
    };

    struct Contact {
        bool hasContact;
//        float contactX = 0; /// X location of first contact in Viper frame
//        float contactY = 0; /// Y location of first contact in Viper frame
//        float contactZ = 0; /// Z location of first contact in Viper frame
        float contactDuration = 0; /// Contact duration in milliseconds
    };

    struct SwingTwist {
        double nx;
        double ny;
        double nz;
        double twist;
    };

    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(RawForce, f1, f2, f3, f4)
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Contact, hasContact, contactDuration)
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SwingTwist, nx, ny, nz, twist)

    constexpr char kViperLogChannelName[] = "/viper/log";
    constexpr char kViperTFChannelName[] = "/tf/viper";
    constexpr char kWorldTFChannelName[] = "/tf/world";
    constexpr char kHandheldPoseChannelName[] = "/hh/pose";
    constexpr char kHandheldPointCloudContactChannelName[] = "/hh/points/contact";
    constexpr char kHandheldPointCloudAllChannelName[] = "/hh/points/all";
    constexpr char kViperPosesChannelName[] = "/viper/poses";
    constexpr char kViperPosesHistoryChannelName[] = "/viper/poses/1/history";
    constexpr char kHandheldForcesRawChannelName[] = "/hh/forces/raw";
    constexpr char kHandheldForcesContactChannelName[] = "/hh/forces/contact";
    constexpr char kHandheldSwingTwistChannelName[] = "/hh/swing_twist";
    constexpr char kHandheldSceneChannelName[] = "/hh/scene";
}

class FoxgloveInterface {
    foxglove::Context context_;

    foxglove::WebSocketServerOptions wsOptions_;
    std::optional<foxglove::WebSocketServer> server_;
    std::optional<foxglove::schemas::LogChannel> logChannel_;
    std::optional<foxglove::schemas::SceneUpdateChannel> sceneChannel_;
    std::optional<foxglove::schemas::PoseInFrameChannel> poseChannel_;
    std::optional<foxglove::schemas::PosesInFrameChannel> hhPosesChannel_;
    std::optional<foxglove::schemas::PosesInFrameChannel> posesInFrameChannel_;
    std::optional<foxglove::schemas::PointCloudChannel> hhPointsContactChannel_;
    std::optional<foxglove::schemas::PointCloudChannel> hhPointsAllChannel_;

    std::optional<foxglove::schemas::FrameTransformChannel> viperTransformChannel_;
    std::optional<foxglove::schemas::FrameTransformChannel> worldTransformChannel_;

    std::optional<foxglove::RawChannel> rawForceChannel_;
    std::optional<foxglove::Schema> rawForceSchema_;

    std::optional<foxglove::RawChannel> contactChannel_;
    std::optional<foxglove::Schema> contactSchema_;

    std::optional<foxglove::RawChannel> swingTwistChannel_;
    std::optional<foxglove::Schema> swingTwistSchema_;

    foxglove::McapWriterOptions mcapOptions_;
    std::optional<foxglove::McapWriter> mcapWriter_;

    std::vector<mdx::Point6f> pointsAll_;
    std::vector<mdx::Point6f> pointsContact_;

    std::mutex swingTwistMtx_;
    mdx::SwingTwist swingTwist_;

    std::mutex contactMtx_;
    mdx::Contact contact_;

    std::mutex rawForceMtx_;
    mdx::RawForce rawForce_;

    std::mutex channelMapMtx_;
    /// Hash table mapping a (client_id, client_channel_id) pair to ClientChannel
    typedef std::map<std::pair<uint32_t, uint32_t>, mdx::ClientChannel> ClientChannelMap;
    std::shared_ptr<ClientChannelMap> channelMap_;

    void initRawChannels() {
        {
            json rawForceSchema = jsonschema::generate_schema<mdx::RawForce>();
            foxglove::Schema schema;
            schema.name = "RawForce";
            schema.encoding = "jsonschema";
            std::string schema_data = rawForceSchema.dump();
            schema.data = reinterpret_cast<const std::byte *>(schema_data.data());
            schema.data_len = schema_data.size();
            rawForceSchema_.emplace(schema);

            auto rawForceChannelResult = foxglove::RawChannel::create(mdx::kHandheldForcesRawChannelName, "json",
                                                                      schema, context_);
            if (!rawForceChannelResult.has_value()) {
                std::stringstream ss;
                ss << "Failed to create raw force channel: " << foxglove::strerror(rawForceChannelResult.error())
                   << std::endl;
                std::cerr << ss.str();
                throw std::runtime_error(ss.str());
            }
            rawForceChannel_.emplace(std::move(rawForceChannelResult.value()));
        }

        {
            json contactSchema = jsonschema::generate_schema<mdx::Contact>();
            foxglove::Schema schema;
            schema.name = "Contact";
            schema.encoding = "jsonschema";
            std::string schema_data = contactSchema.dump();
            schema.data = reinterpret_cast<const std::byte*>(schema_data.data());
            schema.data_len = schema_data.size();
            contactSchema_.emplace(schema);

            auto contactSchemaChannelResult = foxglove::RawChannel::create(mdx::kHandheldForcesContactChannelName, "json", schema, context_);
            if (!contactSchemaChannelResult.has_value()) {
                std::stringstream ss;
                ss << "Failed to create contact channel: " << foxglove::strerror(contactSchemaChannelResult.error()) << std::endl;
                std::cerr << ss.str();
                throw std::runtime_error(ss.str());
            }
            contactChannel_.emplace(std::move(contactSchemaChannelResult.value()));
        }

        {
            json swingTwistSchema = jsonschema::generate_schema<mdx::SwingTwist>();
            foxglove::Schema schema;
            schema.name = "SwingTwist";
            schema.encoding = "jsonschema";
            std::string schema_data = swingTwistSchema.dump();
            schema.data = reinterpret_cast<const std::byte*>(schema_data.data());
            schema.data_len = schema_data.size();
            swingTwistSchema_.emplace(schema);

            auto swingTwistSchemaChannelResult = foxglove::RawChannel::create(mdx::kHandheldSwingTwistChannelName, "json", schema, context_);
            if (!swingTwistSchemaChannelResult.has_value()) {
                std::stringstream ss;
                ss << "Failed to create SwingTwist channel: " << foxglove::strerror(swingTwistSchemaChannelResult.error()) << std::endl;
                std::cerr << ss.str();
                throw std::runtime_error(ss.str());
            }
            swingTwistChannel_.emplace(std::move(swingTwistSchemaChannelResult.value()));
        }
    }

    void initMcap(std::string &&path="viper.mcap") {
        mcapOptions_.path = path;
        mcapOptions_.context = context_;


        auto writerResult = foxglove::McapWriter::create(mcapOptions_);
        if (!writerResult.has_value()) {
            std::stringstream ss;
            ss << "Failed to create MCAP writer: " << foxglove::strerror(writerResult.error()) << std::endl;
            std::cerr << ss.str();
            throw std::runtime_error(ss.str());
        }

        mcapWriter_.emplace(std::move(writerResult.value()));
    }

    void initServer() {
        wsOptions_ = foxglove::WebSocketServerOptions{};
        wsOptions_.name = "mdx";
        wsOptions_.context = context_;

        channelMap_ = std::make_shared<ClientChannelMap>();
        std::function onMessageData = [&](uint32_t client_id, uint32_t client_channel_id, const std::byte *data, size_t data_len) {
            mdx::ClientChannel channel;
            {
                std::lock_guard<std::mutex> guard{channelMapMtx_};
                channel = channelMap_->operator[]({client_id, client_channel_id});
            }
            std::cout << "Received message from (" << client_id << ", " << client_channel_id << ") with topic " <<
            channel.topic << " and schema " << channel.schema_name << " with encoding " <<
            channel.encoding << " and schema encoding " << channel.schema_encoding << " with schema length " << channel.schema_len << std::endl;
        };

        std::function onClientAdvertise = [&](uint32_t client_id, const foxglove::ClientChannel &channel) {
            std::cout << "Received client channel advertisement with pair (" << client_id << ", " << channel.id << ") with topic " << channel.topic << std::endl;
            {
                std::lock_guard<std::mutex> guard{channelMapMtx_};
                foxglove::ClientChannel c{channel};
                channelMap_->operator[]({client_id, channel.id}) = c;
            }
        };

        std::function onClientUnadvertise = [&](uint32_t client_id, uint32_t client_channel_id) {
            std::cout << "Received client channel uadvertisement with pair (" << client_id << ", " << client_channel_id << ")" << std::endl;
            {
                std::lock_guard<std::mutex> guard{channelMapMtx_};
                channelMap_->erase({client_id, client_channel_id});
            }
        };
        wsOptions_.callbacks.onMessageData = onMessageData;
        wsOptions_.callbacks.onClientAdvertise = onClientAdvertise;
        wsOptions_.callbacks.onClientUnadvertise = onClientUnadvertise;
        wsOptions_.supported_encodings.emplace_back("json");
        wsOptions_.supported_encodings.emplace_back("protobuf");
        wsOptions_.capabilities =
            foxglove::WebSocketServerCapabilities::Time |
            foxglove::WebSocketServerCapabilities::ClientPublish |
            foxglove::WebSocketServerCapabilities::Services;

        auto serverResult = foxglove::WebSocketServer::create(std::move(wsOptions_));
        if (!serverResult.has_value()) {
            std::stringstream ss;
            ss << "Failed to create server: " << foxglove::strerror(serverResult.error()) << std::endl;
            std::cerr << ss.str();
            throw std::runtime_error(ss.str());
        }
        server_.emplace(std::move(serverResult.value()));

        FOXGLOVE_INIT_CHANNEL(Log, ViperLog, logChannel_);

        FOXGLOVE_INIT_CHANNEL_LOGGED(SceneUpdate, HandheldScene, sceneChannel_, logError);
        FOXGLOVE_INIT_CHANNEL_LOGGED(PoseInFrame, HandheldPose, poseChannel_, logError);
        FOXGLOVE_INIT_CHANNEL_LOGGED(PosesInFrame, ViperPoses, posesInFrameChannel_, logError);
        FOXGLOVE_INIT_CHANNEL_LOGGED(PosesInFrame, ViperPosesHistory, hhPosesChannel_, logError);
        FOXGLOVE_INIT_CHANNEL_LOGGED(PointCloud, HandheldPointCloudContact, hhPointsContactChannel_, logError);
        FOXGLOVE_INIT_CHANNEL_LOGGED(PointCloud, HandheldPointCloudAll, hhPointsAllChannel_, logError);
        FOXGLOVE_INIT_CHANNEL_LOGGED(FrameTransform, ViperTF, viperTransformChannel_, logError);
        FOXGLOVE_INIT_CHANNEL_LOGGED(FrameTransform, WorldTF, worldTransformChannel_, logError);
    }
public:
    explicit FoxgloveInterface(std::string &&path="viper.mcap") {
        initMcap(std::move(path));
        initServer();
        initRawChannels();
        std::this_thread::sleep_for(1000ms);
        logInfo("Initialized Foxglove interface");
    }

    bool hasContact = false;

    template <typename T>
    std::unique_ptr<foxglove::schemas::PointCloud>
    makePointCloud(std::vector<mdx::Point6<T>> &points,
                   const foxglove::schemas::PackedElementField::NumericType numericType = foxglove::utility::NumericType<T>::type) {
        foxglove::schemas::PointCloud pc;
        pc.pose.emplace();
//        pc.pose.value().orientation->w = 1;

        pc.frame_id = "viper";
        pc.point_stride = 6 * 4;

        foxglove::schemas::PackedElementField xField, yField, zField;
        xField.name = 'x';
        xField.type = numericType;
        xField.offset = 0;

        yField.name = 'y';
        yField.type = numericType;
        yField.offset = sizeof(T);

        zField.name = 'z';
        zField.type = numericType;
        zField.offset = 2*sizeof(T);

        foxglove::schemas::PackedElementField nxField, nyField, nzField;
        nxField.name = "nx";
        nxField.type = numericType;
        nxField.offset = 3*sizeof(T);

        nyField.name = "ny";
        nyField.type = numericType;
        nyField.offset = 4*sizeof(T);

        nzField.name = "nz";
        nzField.type = numericType;
        nzField.offset = 5*sizeof(T);

        std::vector<foxglove::schemas::PackedElementField> fields{xField, yField, zField, nxField, nyField, nzField};

        pc.fields = fields;

        std::vector<std::byte> buffer;
        buffer.reserve(points.size() * pc.point_stride);
        for (const auto &point: points) {
            const auto* byte_ptr = reinterpret_cast<const std::byte*>(&point);
            for (size_t i = 0; i < sizeof(mdx::Point6<T>); ++i) {
                buffer.push_back(byte_ptr[i]);
            }
        }
        pc.data = buffer;

        return std::make_unique<foxglove::schemas::PointCloud>(pc);
    }

    template <typename T>
    void logPointCloudByChannel(std::vector<mdx::Point6<T>> &points, foxglove::schemas::PointCloudChannel &channel) {
        auto pc = *makePointCloud(points);

        channel.log(pc);
    }

    template <typename T>
    void logPointCloudContact(std::vector<mdx::Point6<T>> &points) {
        logPointCloudByChannel(points, hhPointsContactChannel_.value());
    }

    template <typename T>
    void logPointCloudAll(std::vector<mdx::Point6<T>> &points) {
        logPointCloudByChannel(points, hhPointsAllChannel_.value());
    }

    static foxglove::schemas::Timestamp getTimestamp() {
        const auto now = std::chrono::system_clock::now();
        const auto nanos_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        const auto seconds_since_epoch = nanos_since_epoch / 1000000000;
        const auto remaining_nanos = nanos_since_epoch % 1000000000;

        foxglove::schemas::Timestamp timestamp = {
                static_cast<uint32_t>(seconds_since_epoch),
                static_cast<uint32_t>(remaining_nanos)
        };

        return timestamp;
    }

    void publishPointClouds() {
        auto timestamp = getTimestamp();

        auto pcAll = *makePointCloud(pointsAll_, foxglove::schemas::PackedElementField::NumericType::FLOAT32);
        pcAll.timestamp = timestamp;
        hhPointsAllChannel_.value().log(pcAll);

        auto pcContact = *makePointCloud(pointsContact_);
        pcContact.timestamp = timestamp;
        hhPointsContactChannel_.value().log(pcContact);
    }

    void logRawForce(mdx::RawForce &rawForce) {
        {
            std::lock_guard<std::mutex> guard_{rawForceMtx_};
            rawForce_ = rawForce;
        }

        json msg = rawForce;
        auto json_val = msg.dump();
        rawForceChannel_.value().log(reinterpret_cast<const std::byte*>(json_val.c_str()), json_val.size());
    }

    void logContact(const mdx::Contact &contact) {
        {
            std::lock_guard<std::mutex> guard_{contactMtx_};
            contact_ = contact;
        }

        hasContact = contact.hasContact;
        json msg = contact;
        auto json_val = msg.dump();
        contactChannel_.value().log(reinterpret_cast<const std::byte*>(json_val.c_str()), json_val.size());
    }

    void logSwingTwist(const mdx::SwingTwist &swingTwist) {
        {
            std::lock_guard<std::mutex> guard_{swingTwistMtx_};
            swingTwist_ = swingTwist;
        }

        json msg = swingTwist;
        auto json_val = msg.dump();
        swingTwistChannel_.value().log(reinterpret_cast<const std::byte*>(json_val.c_str()), json_val.size());
    }

    void logMessage(const std::string &message, foxglove::schemas::Log::LogLevel level) {
        foxglove::schemas::Log log;
        log.level = level;
        log.message = message;
        log.timestamp = getTimestamp();

        logChannel_.value().log(log);
    }



    void logInfo(const std::string &&message) {
        logMessage(message, foxglove::schemas::Log::LogLevel::INFO);
    }

    void logWarning(const std::string &&message) {
        logMessage(message, foxglove::schemas::Log::LogLevel::WARNING);
    }

    void logError(const std::string &&message) {
        logMessage(message, foxglove::schemas::Log::LogLevel::ERROR);
    }

    void logDebug(const std::string &&message) {
        logMessage(message, foxglove::schemas::Log::LogLevel::DEBUG);
    }

    void publishViperTransform(foxglove::schemas::FrameTransform &tf) {
        viperTransformChannel_.value().log(tf);
    }

    void publishWorldTransform(foxglove::schemas::FrameTransform &tf) {
        worldTransformChannel_.value().log(tf);
    }

    void publishLineToScene(foxglove::schemas::LinePrimitive &line, std::string &&lineName="handheld") {
        foxglove::schemas::SceneEntity entity;
        entity.id = lineName;
        entity.lines.push_back(line);

        foxglove::schemas::SceneUpdate sceneUpdate;
        sceneUpdate.entities.push_back(entity);

        sceneChannel_.value().log(sceneUpdate);
    }

    void publishArrowToScene(foxglove::schemas::ArrowPrimitive &arrow, std::string &&arrowName="handheld") {
        foxglove::schemas::SceneEntity entity;
        entity.id = arrowName;
        entity.arrows.push_back(arrow);
        entity.frame_id = "viper";

        foxglove::schemas::SceneUpdate sceneUpdate;
        sceneUpdate.entities.push_back(entity);

        sceneChannel_.value().log(sceneUpdate);
    }

    void publishPoses(foxglove::schemas::PosesInFrame &poses) {
        posesInFrameChannel_.value().log(poses);
    }

    void publishHHPoses(foxglove::schemas::PosesInFrame &poses) {
//        hhPosesChannel_.value().log(poses);
    }

    void publishPose(foxglove::schemas::PoseInFrame &pose) {
        auto pointMdx = mdx::Point6f::fromPose(pose);

        pointsAll_.push_back(pointMdx);

        if (hasContact) {
            pointsContact_.push_back(pointMdx);
        }

        poseChannel_.value().log(pose);
    }

    void growingCubeExample() {
        auto sec_since_epoch = std::chrono::duration_cast<std::chrono::duration<double>>(
                std::chrono::system_clock::now().time_since_epoch()).count();
        double size = abs(sin(sec_since_epoch)) + 1.0;

        std::stringstream ss;
        ss << "box size is " << std::setprecision(3) << size;
        logInfo(ss.str());

        foxglove::schemas::CubePrimitive cube;
        cube.size = {size, size, size};
        cube.color = {1, 0, 0, 1};

        foxglove::schemas::SceneEntity entity;
        entity.id = "box";
        entity.cubes.push_back(cube);


        foxglove::schemas::SceneUpdate sceneUpdate;
        sceneUpdate.entities.push_back(entity);

        sceneChannel_.value().log(sceneUpdate);
    }
};


#endif //VIPER_FOXGLOVEINTERFACE_HPP
