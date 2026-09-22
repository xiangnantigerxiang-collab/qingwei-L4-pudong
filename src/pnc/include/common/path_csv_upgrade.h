#ifndef PNC_COMMON_PATH_CSV_UPGRADE_H
#define PNC_COMMON_PATH_CSV_UPGRADE_H

#include "common/path_csv.h"
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>
#include <sys/stat.h>
#include <unistd.h>

namespace path_csv {

// 仅规划装载旧地图时使用：三列补speed=10，多列只保留前四列，不解释额外字段。
inline ROW_STATUS_E NormalizeMapRow(std::string &tLine, ROW_S &tRow, bool &tChanged) {
    tChanged = false;
    const auto status = ParseRow(tLine, tRow);
    if(status != INVALID_ROW) return status;

    std::size_t begin = 0;
    std::size_t commas = 0;
    std::size_t fourth_comma = std::string::npos;
    while(commas < 4) {
        const auto comma = tLine.find(',', begin);
        if(comma == std::string::npos) break;
        ++commas;
        if(commas == 4) fourth_comma = comma;
        begin = comma + 1;
    }
    const bool carriage_return = !tLine.empty() && tLine.back() == '\r';
    if(commas == 2) {
        tLine.insert(tLine.size() - (carriage_return ? 1 : 0), ",10");
    } else if(fourth_comma != std::string::npos) {
        tLine.resize(fourth_comma);
        if(carriage_return) tLine += '\r';
    } else {
        return INVALID_ROW;
    }
    const auto normalized_status = ParseRow(tLine, tRow);
    tChanged = normalized_status == VALID_ROW;
    return normalized_status;
}

inline bool SameMapFile(const struct stat &tBefore, const struct stat &tNow) {
    return tBefore.st_dev == tNow.st_dev && tBefore.st_ino == tNow.st_ino &&
           tBefore.st_size == tNow.st_size &&
           tBefore.st_mtim.tv_sec == tNow.st_mtim.tv_sec &&
           tBefore.st_mtim.tv_nsec == tNow.st_mtim.tv_nsec &&
           tBefore.st_ctim.tv_sec == tNow.st_ctim.tv_sec &&
           tBefore.st_ctim.tv_nsec == tNow.st_ctim.tv_nsec;
}

// 完整校验后才调用；临时文件与地图在同一目录，失败不覆盖原文件。
inline bool SaveFourColumnMap(const std::string &tPath, const struct stat &tBefore,
                              std::string &tError) {
    char *resolved = ::realpath(tPath.c_str(), nullptr);
    if(!resolved) {
        tError = "resolve map path failed";
        return false;
    }
    const std::string target(resolved);
    std::free(resolved);
    struct stat current;
    if(::stat(target.c_str(), &current) != 0 || !SameMapFile(tBefore, current) ||
       ::access(target.c_str(), W_OK) != 0) {
        tError = "map changed during loading or is not writable";
        return false;
    }
    std::ifstream input(target.c_str(), std::ios::binary);
    if(!input.is_open()) {
        tError = "reopen map failed";
        return false;
    }
    const std::string pattern = target + ".tmp.XXXXXX";
    std::vector<char> temporary(pattern.begin(), pattern.end());
    temporary.push_back('\0');
    const int fd = ::mkstemp(temporary.data());
    if(fd < 0) {
        tError = "create map temporary file failed";
        return false;
    }
    FILE *output = ::fdopen(fd, "wb");
    if(!output) {
        ::close(fd);
        ::unlink(temporary.data());
        tError = "open map temporary stream failed";
        return false;
    }
    bool ok = true;
    std::string line;
    while(ok && std::getline(input, line)) {
        const bool newline = !input.eof();
        ROW_S row;
        bool changed = false;
        if(NormalizeMapRow(line, row, changed) == INVALID_ROW) {
            ok = false;
            break;
        }
        ok = std::fwrite(line.data(), 1, line.size(), output) == line.size();
        if(ok && newline) ok = std::fputc('\n', output) != EOF;
    }
    ok = ok && !input.bad();
    if(ok) ok = std::fflush(output) == 0;
    if(ok) ok = ::fchmod(fd, tBefore.st_mode & 0777) == 0;
    if(ok) ok = ::fsync(fd) == 0;
    if(std::fclose(output) != 0) ok = false;
    if(ok) ok = ::stat(tPath.c_str(), &current) == 0 && SameMapFile(tBefore, current);
    if(ok) ok = std::rename(temporary.data(), target.c_str()) == 0;
    if(!ok) {
        ::unlink(temporary.data());
        tError = "map write/validation failed or source changed; original not replaced";
    }
    return ok;
}

}  // namespace path_csv

#endif
