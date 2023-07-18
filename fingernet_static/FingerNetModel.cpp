//
// FaceDetectionModel.cpp
//

#include <opencv2/opencv.hpp>

#include "FingerNetModel.h"
#include "resources_api.hpp"

constexpr int model_input_size = 448;

namespace antheus
{

	//
	// --- support functions --------------------------------------------------------------------------
	//

	// custom lib exception
	class FingerNetException : public std::exception {
	private:
		std::string message;

	public:
		explicit FingerNetException(char* msg) : message(msg) {}
		explicit FingerNetException(const std::string& msg) : message(msg) {}
		std::string what() {
			return message;
		}
	};

	// convert tensor to std::vector
	inline auto tensor_to_vector(ov::Tensor& tensor)
	{
		auto out_ptr = tensor.data<float>();
		auto out_vec = std::vector<float>(out_ptr, out_ptr + tensor.get_size());
		return out_vec;
	}

	// ------------------------------------------------------------------------------------------------



	//
	// --- class methods ------------------------------------------------------------------------------
	//

	FingerNetModel::FingerNetModel() try
	{
		core_ptr = std::make_shared<ov::Core>();
		
		auto fs = cmrc::resources::get_filesystem();
		auto model_xml = fs.open("fingernet.xml");
		auto model_bin = fs.open("fingernet.bin");

		xml_string = std::string(model_xml.begin(), model_xml.end());
		bin_vector = std::vector<char>(model_bin.begin(), model_bin.end());

		if (model_bin.size() <= 0 || model_xml.size() <= 0)
			throw std::exception("Recursos de modelo não inicializados");

		this->model = core_ptr->read_model(xml_string, ov::Tensor(ov::element::u8, { bin_vector.size() }, bin_vector.data()));
		this->compiled_model = std::make_shared<ov::CompiledModel>(core_ptr->compile_model(this->model, "CPU"));
	}
	catch (const ov::Exception& e)
	{
		std::cout << e.what() << std::endl;
		std::cin.get();
	}

	std::vector<float> FingerNetModel::pre_processing(unsigned char* raw, int width, int height)
	{
		auto img = cv::Mat(height, width, CV_8UC1, raw);

		// convert to float
		cv::Mat fimg(img.rows, img.cols, CV_32FC1);
		img.convertTo(fimg, CV_32FC1, 1.0 / 255.0);

		// Image size == model input
		if (height == HEIGHT && width == WIDTH)
			return std::vector<float>(reinterpret_cast<float*>(fimg.ptr()),
				reinterpret_cast<float*>(fimg.ptr()) + fimg.rows * fimg.cols * fimg.channels());

		// Otherwise paste the image into a 512, 512 mat
		cv::Mat fixed_size(HEIGHT, WIDTH, CV_32FC1, 1.0);
		fimg.copyTo(fixed_size(cv::Rect(0, 0, fimg.cols, fimg.rows)));

		return std::vector<float>(reinterpret_cast<float*>(fixed_size.ptr()),
			reinterpret_cast<float*>(fixed_size.ptr()) + fixed_size.rows * fixed_size.cols * fixed_size.channels());
	}


	std::array<std::vector<float>, 8> FingerNetModel::make_inference(float* data) try
	{

		auto infer_request = this->compiled_model->create_infer_request();
		auto input_tensor = ov::Tensor(ov::element::f32, { 1, WIDTH, HEIGHT, 1 }, data);

		infer_request.set_input_tensor(input_tensor);
		infer_request.infer();

		std::array<std::vector<float>, 8> out;
		for (auto i = 0; i < 8; i++)
		{
			auto ref = infer_request.get_output_tensor(i);
			out[i] = tensor_to_vector(ref);
		}

		return out;
	}
	catch (const ov::Exception& ex)
	{
		std::cout << ex.what() << std::endl;
		throw FingerNetException("Openvino Exception: " + std::string(ex.what()));
	}
	catch (const std::exception& ex)
	{
		std::cout << ex.what() << std::endl;
		throw FingerNetException("Exception: " + std::string(ex.what()));
	}
}