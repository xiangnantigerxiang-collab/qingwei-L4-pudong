#include <vector>
#include <chrono>
#include <fstream>

class CSVLogger
{
public:
    CSVLogger(const std::string &csv_dir, const std::string prefix_name = "");
    ~CSVLogger();
    bool start(const std::string &header);

    template <typename... Args>
    void log(Args... args)
    {
        if (log_file_.is_open())
        {
            log_file_ << getLogTime() << ",";
            ((log_file_ << std::forward<Args>(args) << ","), ...);
            log_file_ << "\n";
        }
    }
    void stop();

private:
    std::string getFileTime();
    std::string getLogTime();

private:
    std::string log_dir_;
    std::string prefix_name_;
    std::ofstream log_file_;
    std::chrono::high_resolution_clock::time_point start_;
};