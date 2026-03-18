/**
 * @file cpp11_compatibility.hpp
 * @brief C++11 compatibility layer for missing C++14 features
 */

#pragma once

#include <memory>

// C++11 compatible make_unique implementation
#if __cplusplus < 201402L
namespace std {
    template<typename T, typename... Args>
    typename enable_if<!is_array<T>::value, unique_ptr<T>>::type
    make_unique(Args&&... args) {
        return unique_ptr<T>(new T(forward<Args>(args)...));
    }

    template<typename T>
    typename enable_if<is_array<T>::value && extent<T>::value == 0, unique_ptr<T>>::type
    make_unique(size_t size) {
        typedef typename remove_extent<T>::type U;
        return unique_ptr<T>(new U[size]());
    }

    template<typename T, typename... Args>
    typename enable_if<extent<T>::value != 0, void>::type
    make_unique(Args&&...) = delete;
}
#endif
