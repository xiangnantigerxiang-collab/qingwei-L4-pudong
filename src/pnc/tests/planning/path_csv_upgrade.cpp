#include "common/path_csv_upgrade.h"
#include <dirent.h>
#include <iostream>
#include <iterator>

namespace {
int checks = 0;

void Check(bool passed, const char *message) {
    if(!passed) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
    ++checks;
}

void Write(const std::string &path, const std::string &content) {
    std::ofstream output(path, std::ios::binary);
    output << content;
    output.close();
    Check(output.good(), "fixture saved");
}

std::string Read(const std::string &path) {
    std::ifstream input(path, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

struct stat Snapshot(const std::string &path) {
    struct stat state;
    Check(::stat(path.c_str(), &state) == 0, "snapshot source file");
    return state;
}
}

int main(int argc, char **argv) {
    Check(argc == 2, "isolated output directory provided");
    const std::string directory = argv[1];
    const std::string path = directory + "/upgrade.csv";
    std::string error;
    struct CASE_S { const char *before; const char *after; };
    const CASE_S cases[] = {
        {"1,2,90\n3,4,90\n", "1,2,90,10\n3,4,90,10\n"},
        {"\xEF\xBB\xBF" "1,2,90\r\n\r\n# comment,a,b,c,d\r\n3,4,90",
         "\xEF\xBB\xBF" "1,2,90,10\r\n\r\n# comment,a,b,c,d\r\n3,4,90,10"},
        {"1,2,90,0,1\r\n3,4,90,0.25,unused,more\r\n", "1,2,90,0\r\n3,4,90,0.25\r\n"},
        {"1,2,90,0.5\n3,4,90\n", "1,2,90,0.5\n3,4,90,10\n"},
        {" 1 , 2 , 90 \r\n3,4,90,10,", " 1 , 2 , 90 ,10\r\n3,4,90,10"},
    };
    for(const auto &item : cases) {
        Write(path, item.before);
        Check(::chmod(path.c_str(), 0640) == 0, "set fixture permission bits");
        const auto before = Snapshot(path);
        Check(path_csv::SaveFourColumnMap(path, before, error), "atomic conversion succeeds");
        Check(Read(path) == item.after, "converted bytes preserve source fields, comments and newline form");
        Check((Snapshot(path).st_mode & 0777) == 0640, "permissions preserved");
    }
    for(const char *bad : {"1,2,90\n3,,90\n", "1,2,90\n3,4,nan\n", "1,2,90\n3,4\n",
                           "1,2,90\n3,4,90,-1\n", "1,2,90\n3,4,90,,1\n"}) {
        Write(path, bad);
        Check(!path_csv::SaveFourColumnMap(path, Snapshot(path), error), "invalid later row rejects conversion");
        Check(Read(path) == bad, "failure never commits an earlier partially converted prefix");
    }
    const std::string legacy = "1,2,90\n3,4,90\n";
    Write(path, legacy);
    const auto stale = Snapshot(path);
    Write(path, legacy + "5,6,90\n");
    const std::string changed = Read(path);
    Check(!path_csv::SaveFourColumnMap(path, stale, error) && Read(path) == changed,
          "source replacement/modification during loading is never overwritten");

    Write(path, legacy);
    Check(::chmod(path.c_str(), 0440) == 0, "set source read-only");
    const bool source_written = path_csv::SaveFourColumnMap(path, Snapshot(path), error);
    Check(::chmod(path.c_str(), 0640) == 0, "restore source permissions");
    Check(!source_written && Read(path) == legacy, "read-only source fails without data loss");
    const auto original = Snapshot(path);
    Check(::chmod(directory.c_str(), 0500) == 0, "make temporary-file directory unwritable");
    const bool directory_written = path_csv::SaveFourColumnMap(path, original, error);
    Check(::chmod(directory.c_str(), 0700) == 0, "restore directory permissions");
    Check(!directory_written && Read(path) == legacy, "temporary creation failure preserves original file");

    const std::string alias = directory + "/alias.csv";
    Check(::symlink("upgrade.csv", alias.c_str()) == 0, "create relative map symlink");
    Check(path_csv::SaveFourColumnMap(alias, Snapshot(alias), error), "upgrade symlink target");
    struct stat link;
    Check(::lstat(alias.c_str(), &link) == 0 && S_ISLNK(link.st_mode), "map symlink itself remains intact");
    Check(Read(path) == "1,2,90,10\n3,4,90,10\n", "actual linked file receives normalized data");
    Check(::unlink(alias.c_str()) == 0, "remove fixture symlink");

    DIR *entries = ::opendir(directory.c_str());
    Check(entries != nullptr, "inspect temporary directory");
    while(const auto entry = ::readdir(entries))
        Check(std::string(entry->d_name).find(".tmp.") == std::string::npos, "no temporary files remain after success or failure");
    ::closedir(entries);
    std::cout << "PASS CSV file upgrade: " << checks << " checks\n";
}
