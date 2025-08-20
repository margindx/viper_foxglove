//
// Created by Hamza El-Kebir on 8/14/25.
//

#ifndef VIPER_FOXGLOVEMACROS_HPP
#define VIPER_FOXGLOVEMACROS_HPP

/**
 * @brief Initializes a Foxglove channel, handling potential creation errors.
 *
 * @param channel The base name of the channel schema (e.g., Log for LogChannel).
 * @param channel_name The channel name (e.g., ViperLog).
 * @param channel_var The variable (std::optional<...>) to store the created channel in.
 */
#define FOXGLOVE_INIT_CHANNEL(channel, channel_name, channel_var)                           \
    do {                                                                                    \
        /* Construct schema and constant names using token pasting */                       \
        auto result = foxglove::schemas::channel##Channel::create(mdx::k##channel_name##ChannelName, context_); \
        if (!result.has_value()) {                                                          \
            std::stringstream ss;                                                           \
            /* Stringify the channel name for a clean error message */                      \
            ss << "Failed to create " #channel_name " (" #channel ") channel: " << foxglove::strerror(result.error()); \
            std::cerr << ss.str() << std::endl;                                             \
            throw std::runtime_error(ss.str());                                             \
        }                                                                                   \
        /* Move the created channel into the destination variable */                        \
        (channel_var).emplace(std::move(result.value()));                                   \
    } while (0)

/**
 * @brief Initializes a Foxglove channel, handling potential creation errors.
 *
 * @param channel The base name of the channel schema (e.g., Log for LogChannel).
 * @param channel_name The channel name (e.g., ViperLog).
 * @param channel_var The variable (std::optional<...>) to store the created channel in.
 * @param error_logger A function that takes char* and appropriately logs the error.
 */
#define FOXGLOVE_INIT_CHANNEL_LOGGED(channel, channel_name, channel_var, error_logger)      \
    do {                                                                                    \
        /* Construct schema and constant names using token pasting */                       \
        auto result = foxglove::schemas::channel##Channel::create(mdx::k##channel_name##ChannelName, context_); \
        if (!result.has_value()) {                                                          \
            std::stringstream ss;                                                           \
            /* Stringify the channel name for a clean error message */                      \
            ss << "Failed to create " #channel_name " (" #channel ") channel: " << foxglove::strerror(result.error());         \
            (error_logger)(ss.str());                                                       \
            std::cerr << ss.str() << std::endl;                                             \
            throw std::runtime_error(ss.str());                                             \
        }                                                                                   \
        /* Move the created channel into the destination variable */                        \
        (channel_var).emplace(std::move(result.value()));                                   \
    } while (0)

#endif //VIPER_FOXGLOVEMACROS_HPP
