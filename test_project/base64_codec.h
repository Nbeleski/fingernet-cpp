//
// base64_codec.h
//
#include <string>
#include <vector>

std::string base64_encode(unsigned char const* p, unsigned int len);
std::string base64_encode(const std::vector<unsigned char>& msg);
std::vector<unsigned char> base64_decode(std::string const& s);

std::string base64_decode_string(std::string const& encoded_string);
std::string base64_encode_string(const std::string& msg);
