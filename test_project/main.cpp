//
//
//

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <filesystem>
#include <array>
#include <unordered_map>
#include <random>

#include <fingernet.h>
#include "D:\\Projetos\\cpp\\Fingerprint\\FingerNet\\include\\antheus\\codec.h"

#include "base64_codec.h"

#pragma comment(lib, "D:\\Projetos\\cpp\\Fingerprint\\FingerNet\\libs\\Release\\antheus_image_codec_static.lib")

using namespace std;
namespace fs = std::filesystem;

struct decoded_image
{
	std::vector<unsigned char> raw;
	int w = 0;
	int h = 0;
	int bpp = 0;
	int codec_index = 0;

	decoded_image()
	{
		w = 0;
		h = 0;
		bpp = 0;
		codec_index = 0;
	}
};

std::optional<decoded_image> decode_image(std::vector<unsigned char> buffer)
{
	decoded_image result;

	auto temp_buffer = decode_image_buffer(buffer.data(), buffer.size(), result.w, result.h, result.bpp, result.codec_index);
	if (temp_buffer == nullptr || result.w <= 0 || result.h <= 0 || result.bpp <= 0)
	{
		return {};
	}

	result.raw = std::vector<unsigned char>(temp_buffer, temp_buffer + result.w * result.h * result.bpp);
	delete temp_buffer;

	return result;
}

//
// --- main ---------------------------------------------------------------------------------------
//
int main(int argc, char** argv)
{
	std::cout << std::fixed;
	std::cout << std::setprecision(4);

	if (argc != 2)
		return EXIT_FAILURE;

	auto inputFile = std::string(argv[1]);

	ifstream ifs(inputFile, std::ios::binary);
	auto buffer = std::vector<unsigned char>(std::istreambuf_iterator<char>{ ifs }, {});
	auto r = decode_image(buffer);

	if (!r.has_value())
		return EXIT_FAILURE;

	auto image = r.value();

	if (image.bpp == 3)
		return EXIT_FAILURE;

	//std::vector<unsigned char> mask, enhanced;
	//mask.resize(image.raw.size());
	//enhanced.resize(image.raw.size());
	//
	//std::vector<int> minutiae;
	//minutiae.resize(255 * 3);
	//int num_minutiae = 0;

	//antheus::extract_fingerprint_features(image.raw.data(), image.w, image.h, mask.data(), enhanced.data(), minutiae.data(), &num_minutiae);
	antheus::draw_minutiae(image.raw.data(), image.w, image.h, "drawn_minutiae.png");

	return EXIT_SUCCESS;
}
// ------------------------------------------------------------------------------------------------


//
// Other func
//