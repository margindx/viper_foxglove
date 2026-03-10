//
// Created by Hamza El-Kebir on 8/11/25.
//

#include "FoxgloveUtility.hpp"

template <typename T>
foxglove::schemas::PackedElementField::NumericType foxglove::utility::NumericType<T>::type = foxglove::schemas::PackedElementField::NumericType::FLOAT64;

template <>
foxglove::schemas::PackedElementField::NumericType foxglove::utility::NumericType<float>::type = foxglove::schemas::PackedElementField::NumericType::FLOAT32;

template <>
foxglove::schemas::PackedElementField::NumericType foxglove::utility::NumericType<double>::type = foxglove::schemas::PackedElementField::NumericType::FLOAT64;

template <>
foxglove::schemas::PackedElementField::NumericType foxglove::utility::NumericType<int>::type = foxglove::schemas::PackedElementField::NumericType::INT32;

template <>
foxglove::schemas::PackedElementField::NumericType foxglove::utility::NumericType<unsigned int>::type = foxglove::schemas::PackedElementField::NumericType::UINT32;