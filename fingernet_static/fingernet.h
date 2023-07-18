//
// foo_passport_check.h
//
#pragma once

#include <vector>
#include <string>

#define foo_API __declspec(dllexport)

namespace foo
{
    enum class ReturnCode
    {
        Ok = 0,
        NoResult = -1,
        NullHandle = -2,
        InvalidHandle = -3,
        NullParameter = -4,
        InvalidParameter = -5,
        BadImageSize = -6,
        ImageTooLarge = -7,
        ModelError = -8,
    };

    /**
    * 
    */
    extern "C" foo_API ReturnCode extract_fingerprint_features(unsigned char* raw, int w, int h, unsigned char* mask, unsigned char* enhanced, int* minutiae, int* num_minutiae);


    extern "C" foo_API ReturnCode draw_minutiae(unsigned char* raw, int w, int h, const char* filename);

}

// validate libs
// void ext_lib_versions();