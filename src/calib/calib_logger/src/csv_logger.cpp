#include <string.h>
#include "csv_logger.h"

CSVLogger::CSVLogger(const std::string &dir, const std::string prefix_name)
    : log_dir_(dir), prefix_name_(prefix_name)
{
}

CSVLogger::~CSVLogger()
{
    this->stop();
}

bool CSVLogger::start(const std::string &header)
{
    if (log_dir_.back() == '/')
    {
        log_dir_.pop_back();
    }
    std::string file_name = prefix_name_;
    if (!file_name.empty())
    {
        file_name = file_name + "_";
    }
    std::string filepath = log_dir_ + "/" + file_name + getFileTime() + ".csv";
    printf("记录csv数据到:%s\n", filepath.c_str());
    printf("数据头内容:%s\n", header.c_str());
    log_file_.open(filepath, std::ios::out | std::ios::app);
    if (log_file_.is_open())
    {
        start_ = std::chrono::high_resolution_clock::now();
        log_file_ << "timestamp," << header << "\n";
        return true;
    }
    return false;
}

void CSVLogger::stop()
{
    if (log_file_.is_open())
    {
        auto end = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::ratio<1, 1000>>(end - start_).count();
        log_file_ << "total:," << ms << "ms\n";
        log_file_.close();
    }
}

std::string CSVLogger::getFileTime()
{
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time_t);

    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    // int cs_count = ms.count() % 1000;
    // char csbuffer[4] = {0};
    // sprintf(csbuffer, "%d", cs_count);
    char t[64] = "%Y-%m-%d-%H-%M-%S";
    // char *time_format = strcat(t, csbuffer);
    char buffer[64];
    strftime(buffer, 64, t, &tm);
    std::string current_time(buffer);
    return current_time;
}

std::string CSVLogger::getLogTime()
{
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time_t);

    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    int cs_count = ms.count() % 1000;
    char csbuffer[4] = {0};
    sprintf(csbuffer, "%d", cs_count);
    char t[64] = "%m-%d %H:%M:%S:";
    char *time_format = strcat(t, csbuffer);
    char buffer[64];
    strftime(buffer, 64, time_format, &tm);
    std::string log_time(buffer);
    return log_time;
}