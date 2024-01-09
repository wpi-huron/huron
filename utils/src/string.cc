#include "huron/utils/string.h"

namespace huron {
namespace utils {

std::vector<std::string> split(const std::string& str,
                               const std::string& delimiter) {
  std::vector<std::string> ret;
  size_t last = 0, next = 0;
  while ((next = str.find(delimiter, last)) != std::string::npos) {
    ret.push_back(str.substr(last, next-last));
    last = next + delimiter.length();
  }
  ret.push_back(str.substr(last));
  return ret;
}

}  // namespace utils
}  // namespace huron
