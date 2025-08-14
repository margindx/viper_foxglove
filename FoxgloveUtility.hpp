//
// Created by Hamza El-Kebir on 8/11/25.
//

#ifndef VIPER_FOXGLOVEUTILITY_HPP
#define VIPER_FOXGLOVEUTILITY_HPP

#include <foxglove/schemas.hpp>

namespace foxglove::utility {
    template <typename T>
    struct NumericType {
        static foxglove::schemas::PackedElementField::NumericType type;
    };

    template <>
    foxglove::schemas::PackedElementField::NumericType foxglove::utility::NumericType<float>::type;

    template <>
    foxglove::schemas::PackedElementField::NumericType foxglove::utility::NumericType<double>::type;

    template <>
    foxglove::schemas::PackedElementField::NumericType foxglove::utility::NumericType<int>::type;

    template <>
    foxglove::schemas::PackedElementField::NumericType foxglove::utility::NumericType<uint>::type;
}


#endif //VIPER_FOXGLOVEUTILITY_HPP
