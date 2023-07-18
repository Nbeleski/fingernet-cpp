//
// fingernet_static.cpp
//

#include <iostream>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>

#include "fingernet.h"
#include "FingerNetModel.h"
#include "resources_api.hpp"

using namespace std;

constexpr float PI = 3.141592653589;

// Should the allocation for the output images be done inside the DLL and a free_memory be provided?
// Let's discuss the pros and cons of each approach:
//
// Program allocates memory before calling the DLL function :
// Pros:
//      - Clear responsibility : The program explicitly manages the memory allocation, 
//      making it clear who is responsible for freeing the memory.
//      - Flexibility : The program can choose the appropriate memory allocation method
//      based on its specific needs(e.g., stack allocation, heap allocation, custom allocator).
//
// Cons :
//      - Dependency on program logic : The program needs to ensure that it allocates the correct amount
//      of memory before calling the DLL function.Failure to allocate the correct size can lead to memory corruption or access violations.
//      - Potential inefficiency : If the program overestimates the required memory size, it may allocate more memory than necessary,
//      leading to wasted resources.
//
// DLL allocates memory and returns it to the program :
// Pros:
//      - Convenience: The DLL takes care of memory allocation, relieving the program from managing memory - related details.
//      - Encapsulation : The DLL can encapsulate the memory allocation logic and provide a consistent interface for all callers.
//
// Cons :
//    - Responsibility for freeing memory : The program needs to ensure that it frees the memory returned by the DLL function.
//      If the program forgets to deallocate the memory or uses the wrong deallocation method, it can lead to memory leaks or other memory - related issues.
//    - Limited flexibility : The DLL may use a specific memory allocation method(e.g., heap allocation) that may not align with the program's requirements.
//       It can be challenging to override or customize this behavior.
//
//      Overall, the choice between these approaches depends on various factors, including the specific requirements of
//      your application and the level of control you want to have over memory allocation.If you have a clear memory allocation
//      strategy in your program or if you want to have control over memory management, it may be preferable to allocate memory 
//      in the program itself.On the other hand, if you want to abstract the memory allocation details and provide a more 
//      convenient interface, the DLL can handle memory allocation and return the allocated memory to the program.
//
// By chatgpt

namespace antheus
{
    // --------------------------------------------------------------------------------------------
    // Support functions
    // --------------------------------------------------------------------------------------------

    // I am using a template here to access the pixel values as a cv::Vec of specific size
    // not sure how to achieve this wihtout compile-time specialization
    template<int n>
    cv::Mat per_channel_argmax(cv::Mat& src)
    {
        // assert(src.elemType() == CV_32F);
        // assert(src.channels == n);

        // output mat has only on channel
        cv::Mat dst(src.cols, src.rows, CV_8UC1);

        // if src has only channel, output is 0
        if (src.channels() == 1)
        {
            dst = 0;
            return dst;
        }

        for (int i = 0; i < src.rows; ++i)
        {
            for (int j = 0; j < src.cols; ++j)
            {
                std::vector<float> tmp(src.channels());

                auto pixel_val = src.at<cv::Vec<float, n>>(i, j);

                for (int k = 0; k < src.channels(); ++k)
                {
                    tmp[k] = pixel_val[k];
                }

                auto idx = std::distance(tmp.begin(), std::max_element(tmp.begin(), tmp.end()));
                dst.at<uint8_t>(i, j) = idx;
            }
        }

        return dst;
    }

    std::pair<cv::Mat, cv::SparseMat> convertToSparse(const cv::Mat& mnt_s_out, float thr = 0.5)
    {
        // Convert the matrix to a sparse coordinate matrix
        cv::SparseMat sparse(mnt_s_out);
        for (int i = 0; i < mnt_s_out.rows; ++i)
        {
            for (int j = 0; j < mnt_s_out.cols; ++j)
            {
                float value = mnt_s_out.at<float>(i, j);
                if (value <= thr)
                    sparse.erase(i, j);
            }
        }

        // Extract the row and column indices as a cv::Mat
        cv::Mat mnt_list(sparse.nzcount(), 2, CV_32S);
        int counter = 0;
        for (cv::SparseMatIterator it = sparse.begin(); it != sparse.end(); ++it)
        {
            const cv::SparseMat::Node* node = it.node();
            int row = node->idx[0];
            int col = node->idx[1];
            mnt_list.at<int>(counter, 0) = row;
            mnt_list.at<int>(counter, 1) = col;

            ++counter;
        }

        // sparse is hard-copy?
        return std::make_pair(mnt_list, sparse);
    }

    // comparator to reverse sort vec4f using the last item
    bool vec4fInvComparator(const std::vector<float>& a, const std::vector<float>& b)
    {
        return a[3] > b[3];
    }

    // angle difference
    inline float angle_delta(float a, float b, float maxD = PI * 2)
    {
        auto delta = abs(a - b);
        return min(delta, maxD - delta);
    }

    // --------------------------------------------------------------------------------------------



    // --------------------------------------------------------------------------------------------
    // Main methods (which will be exported...)
    // --------------------------------------------------------------------------------------------

    std::unique_ptr<FingerNetModel> finger_net_model = nullptr;
    std::mutex m_lib;

    //
    ReturnCode extract_fingerprint_features(unsigned char* raw, int w, int h,
        unsigned char* mask, unsigned char* enhanced, int* minutiae, int* num_minutiae)
    {
        if (raw == nullptr || mask == nullptr || enhanced == nullptr || minutiae == nullptr || num_minutiae == nullptr)
            return ReturnCode::NullParameter;
        if (w <= 0 || h <= 0)
            return ReturnCode::BadImageSize;
        if (w > 512 || h > 512)
            return ReturnCode::ImageTooLarge;

        // avoid parellel calls to DLL, ov is set for parallel optimization of a single inference
        std::lock_guard lk(m_lib);

        try
        {
            if (finger_net_model == nullptr)
                finger_net_model = std::make_unique<FingerNetModel>();

            // ------------------------------------------------------------------------------------
            // pre-processing the input
            // converting to float, changing the size to fit input_shape correctly
            // ------------------------------------------------------------------------------------
            auto float_img = finger_net_model->pre_processing(raw, w, h);
            auto [enhance_img, mnt_h_out, mnt_o_out,
                mnt_s_out, mnt_w_out, ori_out_1,
                ori_out_2, seg_out] = finger_net_model->make_inference(float_img.data());


            // ------------------------------------------------------------------------------------
            // Mask & mask resized
            // ------------------------------------------------------------------------------------
            cv::Mat mat_mask_resized;
            auto mat_mask = cv::Mat(64, 64, CV_32FC1, seg_out.data());
            mat_mask.forEach<float>
                (
                    [](float& pixel, const int* position) -> void
                    {
                        pixel = std::round(pixel);
                    }
            );
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
            cv::Mat morphed_seg_out;
            cv::morphologyEx(mat_mask, morphed_seg_out, cv::MORPH_OPEN, kernel);
            cv::resize(morphed_seg_out, mat_mask_resized, cv::Size(512, 512), 0.0, 0.0, cv::INTER_NEAREST);
            mat_mask_resized.convertTo(mat_mask_resized, CV_8UC1);
            mat_mask.convertTo(mat_mask, CV_8UC1);

            // crop to original size
            cv::Mat mask_final_crop = mat_mask_resized(cv::Rect(0, 0, w, h)).clone();
            // copy to output
            std::memcpy(mask, mask_final_crop.ptr(), mask_final_crop.total() * mask_final_crop.elemSize());


            // ------------------------------------------------------------------------------------
            // Enhanced image (gabor-like)
            // ------------------------------------------------------------------------------------
            cv::Mat mat_enh_masked;
            auto mat_enh = cv::Mat(512, 512, CV_32FC1, enhance_img.data());
            mat_enh.convertTo(mat_enh, CV_8UC1);
            mat_enh_masked = mat_enh.mul(mat_mask_resized);

            // crop to original size
            cv::Mat enh_final_mask = mat_enh_masked(cv::Rect(0, 0, w, h)).clone();
            // copy to output
            std::memcpy(enhanced, enh_final_mask.ptr(), enh_final_mask.total() * enh_final_mask.elemSize());


            // ------------------------------------------------------------------------------------
            // Minutiae pre-processing
            // ------------------------------------------------------------------------------------
            
            // threshold used to filter potential minutiae
            auto mnt_thr = 0.5f;

            // mat_mnt_s to cv::Mat to apply a mask
            auto mat_mnt_s = cv::Mat(64, 64, CV_32FC1, mnt_s_out.data());
            cv::Mat mat_mnt_s_masked;
            mat_mnt_s.copyTo(mat_mnt_s_masked, mat_mask);

            // apply the thresholding (mnt_s_out>thresh in python)
            cv::Mat fMask;
            cv::threshold(mat_mnt_s_masked, fMask, mnt_thr, 1.0f, cv::THRESH_BINARY);
            fMask.convertTo(fMask, CV_32F);
            cv::Mat mat_mnt_s_thr;
            mat_mnt_s_thr = mat_mnt_s_masked.mul(fMask);


            // ------------------------------------------------------------------------------------
            // Sparse matrix and argmax operations
            // ------------------------------------------------------------------------------------

            auto [mnt_list, mnt_sparse] = convertToSparse(mat_mnt_s_thr);


            // Argmax ops could be done without converting to cv::Mat
            auto mat_mnt_w_out = cv::Mat(64, 64, CV_32FC(8), mnt_w_out.data());
            auto mat_mnt_h_out = cv::Mat(64, 64, CV_32FC(8), mnt_h_out.data());
            auto mat_mnt_o_out = cv::Mat(64, 64, CV_32FC(180), mnt_o_out.data());

            auto mat_mnt_w_argmax = per_channel_argmax<8>(mat_mnt_w_out);
            auto mat_mnt_h_argmax = per_channel_argmax<8>(mat_mnt_h_out);
            auto mat_mnt_o_argmax = per_channel_argmax<180>(mat_mnt_o_out);


            // ------------------------------------------------------------------------------------
            // Minutiae consolidation (+ angle correction)
            // ------------------------------------------------------------------------------------
            cv::Mat mnt_final(mnt_list.rows, 4, CV_32FC1, 0.0f);
            auto mnt_idx = 0;
            for (auto it = mnt_sparse.begin<float>(); it != mnt_sparse.end<float>(); ++it)
            {
                auto n = it.node();

                auto row = mnt_list.at<int>(mnt_idx, 0);
                auto col = mnt_list.at<int>(mnt_idx, 1);

                mnt_final.at<float>(mnt_idx, 0) = n->idx[0] * 8 + int(mat_mnt_w_argmax.at<uint8_t>(row, col));
                mnt_final.at<float>(mnt_idx, 1) = n->idx[1] * 8 + int(mat_mnt_h_argmax.at<uint8_t>(row, col));

                auto ori = (float(mat_mnt_o_argmax.at<uint8_t>(row, col)) * 2.0 - 89.0) * PI / 180;
                mnt_final.at<float>(mnt_idx, 2) = (ori >= 0 ? ori : ori + 2 * PI);
                mnt_final.at<float>(mnt_idx, 3) = mat_mnt_s.at<float>(row, col);

                mnt_idx++;
            }


            // ------------------------------------------------------------------------------------
            // Non maximum suppression
            // ------------------------------------------------------------------------------------
            std::vector <std::vector<float>> min_sort; // vec.assing() instead of loop?
            for (auto i = 0; i < mnt_final.rows; ++i)
            {
                // Watch out x & y ordering
                auto p = mnt_final.at<cv::Vec4f>(i);
                min_sort.emplace_back(std::vector<float>({ p[1] , p[0], p[2], p[3] }));
            }

            // Sort by confidence score
            std::sort(min_sort.begin(), min_sort.end(), vec4fInvComparator);

            // Calculate the distance and orientation difference between every potential minutia
            // TODO: Could this be done operating directly in matrices, like numpy?
            constexpr float maxD = 16;
            constexpr float maxO = PI / 6;
            cv::Mat in_range(min_sort.size(), min_sort.size(), CV_8UC1);
            in_range = 0;
            for (auto i = 0; i < min_sort.size(); ++i)
                in_range.at<uint8_t>(i, i) = 1;

            std::vector<uint8_t> keep_list(min_sort.size(), 1);
            for (decltype(min_sort.size()) i = 0; i < min_sort.size(); ++i) //decltype overkill
            {
                for (decltype(i) j = i + 1; j < min_sort.size(); ++j)
                {
                    auto D = sqrtf(powf(min_sort[i][0] - min_sort[j][0], 2) + powf(min_sort[i][1] - min_sort[j][1], 2));
                    auto O = angle_delta(min_sort[i][2], min_sort[j][2]);

                    if (D <= maxD && O <= maxO)
                    {
                        in_range.at<uint8_t>(i, j) = 1;
                        in_range.at<uint8_t>(j, i) = 1;
                    }
                }
            }

            // nms consolidation
            for (size_t i = 0; i < min_sort.size(); ++i)
            {
                if (keep_list[i] == 0)
                    continue;

                for (size_t j = i + 1; j < min_sort.size(); ++j)
                {
                    keep_list[j] = keep_list[j] * (1 - in_range.at<uint8_t>(j, i));
                }
            }

            // finally, we select the 'keep' minutiae
            // TODO: could save directly to output without the intermediate vector
            auto min_final = std::vector<std::array<float, 3>>();
            for (size_t i = 0; i < keep_list.size(); ++i)
            {
                if (keep_list[i] == 1)
                {
                    min_final.push_back({ min_sort[i][0] , min_sort[i][1] , min_sort[i][2] });
                }
            }

            // save to output
            size_t ptr = 0;
            *num_minutiae = min_final.size();
            for (const auto& m : min_final)
            {
                minutiae[ptr++] = int(m[0]);
                minutiae[ptr++] = int(m[1]);
                minutiae[ptr++] = int((m[2] * 180.0 / PI) + 0.5f);
            }

        }
        catch (const std::exception& e)
        {
            // Properly treat the exception?
            std::cout << e.what() << std::endl;
            throw(e);
        }

        return ReturnCode::Ok;
    }

    // --------------------------------------------------------------------------------------------

    // Draw result with minutiae to verify
    // mostly a debug/testing function
    ReturnCode draw_minutiae(unsigned char* raw, int w, int h, const char* filename)
    {
        std::vector<unsigned char> mask, enhanced;
        mask.resize(w * h);
        enhanced.resize(w * h);
        std::vector<int> minutiae;
        minutiae.resize(255 * 3);
        int num_minutiae = 0;

        auto ret = extract_fingerprint_features(raw, w, h, mask.data(), enhanced.data(), minutiae.data(), &num_minutiae);
        if (ret != ReturnCode::Ok)
            return ret;

        //printf("num_minutiae: %d\n", num_minutiae);

        auto img = cv::Mat(w, h, CV_8UC1, raw);
        int r = 15;
        cv::Mat draw_img;
        cv::cvtColor(img, draw_img, cv::COLOR_GRAY2BGR);
        for (size_t i = 0; i < num_minutiae * 3; i += 3) {
            cv::circle(draw_img, cv::Point(minutiae[i + 0], minutiae[i + 1]), 6, cv::Scalar(0, 0, 200), 2);
            cv::line(draw_img,
                cv::Point(minutiae[i + 0], minutiae[i + 1]),
                cv::Point(minutiae[i + 0] + r * cos(PI * minutiae[i + 2] / 180.f), minutiae[i + 1] + r * sin(PI * minutiae[i + 2] / 180.f)),
                cv::Scalar(0, 0, 200),
                2
            );
        }

        cv::imwrite(filename, draw_img);
        return ReturnCode::Ok;
    }
}


// Used in tests
void external_lib_versions()
{
    std::stringstream ov_ver;
    ov_ver << ov::get_openvino_version();

    cout << ov_ver.str() << endl;
    cout << "OpenCV: " << cv::getVersionString() << endl;
}