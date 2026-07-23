//
// Created by Hamza El-Kebir on 8/11/25.
//

#ifndef VIPER_FOXGLOVEUTILITY_HPP
#define VIPER_FOXGLOVEUTILITY_HPP

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

namespace foxglove::utility {
    template <typename T>
    struct NumericType {
        static foxglove::schemas::PackedElementField::NumericType type;
    };

}


#endif //VIPER_FOXGLOVEUTILITY_HPP
