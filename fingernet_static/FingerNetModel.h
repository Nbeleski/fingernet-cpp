//
// FingerNetModel.h
//
#pragma once

#include <openvino/openvino.hpp>
#include <opencv2/opencv.hpp>

#include <array>

namespace foo
{
	class FingerNetModel
	{
	private:
		std::shared_ptr<ov::Core> core_ptr;
		std::shared_ptr<ov::Model> model;
		std::shared_ptr<ov::CompiledModel> compiled_model;

		std::string xml_string;
		std::vector<char> bin_vector;

		// The model currently has a fixed input shape
		// to have variable input size I need to better understand
		// openvino model optimizer
		const unsigned long long WIDTH = 512;
		const unsigned long long HEIGHT = 512;

	public:
		FingerNetModel();
		std::vector<float> pre_processing(unsigned char* raw, int width, int height);
		std::array<std::vector<float>, 8> make_inference(float* data);

	};
}

