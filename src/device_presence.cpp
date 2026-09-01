#include "wbr_gps/device_presence.hpp"

#include <algorithm>
#include <cctype>
#include <dirent.h>
#include <string>

namespace wbr_gps {

namespace {
constexpr const char* kByIdDir = "/dev/serial/by-id";

std::string upper(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return (char)std::toupper(c); });
    return s;
}
} // namespace

std::string scan_by_id_for(const char* dir, const char* needle)
{
    DIR* d = ::opendir(dir);
    if (!d) return "";

    const std::string want = upper(needle);
    std::string result;

    struct dirent* ent;
    while ((ent = ::readdir(d)) != nullptr) {
        const std::string name = ent->d_name;
        if (name == "." || name == "..") continue;
        if (upper(name).find(want) != std::string::npos) {
            result = std::string(dir) + "/" + name;
            break;
        }
    }
    ::closedir(d);
    return result;
}

std::string find_leo_bodnar_serial()
{
    return scan_by_id_for(kByIdDir, "Leo_Bodnar");
}

bool leo_bodnar_present()
{
    return !find_leo_bodnar_serial().empty();
}

} // namespace wbr_gps
