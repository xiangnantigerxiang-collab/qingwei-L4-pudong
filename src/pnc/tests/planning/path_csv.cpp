#include "common/path_csv.h"
#include <cstdlib>
#include <fstream>
#include <iostream>

namespace {
int checks = 0;
void Check(bool condition, const char *message) {
    if(!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
    ++checks;
}
}

int main(int argc, char **argv) {
    struct CASE_S {
        const char *line;
        double speed;
    };
    const CASE_S valid[] = {
        {"1,2,90,10", 10},
        {"1,2,90,0", 0},
        {"1,2,90,2.5", 2.5},
        {" \t1 , 2 , 90 , 10\r", 10},
        {"\xEF\xBB\xBF" "1,2,90,1e1", 10},
    };
    for(const auto &item : valid) {
        path_csv::ROW_S row;
        Check(path_csv::ParseRow(item.line, row) == path_csv::VALID_ROW, "accept supported row");
        Check(row.columns == 4 && row.values[0] == 1 && row.values[1] == 2 &&
                  row.values[2] == 90 && row.values[3] == item.speed,
              "exactly four fields preserve geometry and nonnegative speed including zero");
    }
    for(const char *line : {"", " \t\r", "# x,y,heading,max_speed", "\xEF\xBB\xBF" " # comment"}) {
        path_csv::ROW_S row;
        Check(path_csv::ParseRow(line, row) == path_csv::SKIP_ROW, "skip blank and comment rows");
    }
    for(const char *line : {"1,2", "1,2,90", "1,2,90,10,0", "1,2,90,10,1",
                           "1 2 90 10", "1,,90,10", "1,2,90,", "1,2,90,10,",
                           "1,2,90,10,0,1", "1,2,90,-1", "1,2,90,nan", "1,2,90,inf",
                           "nan,2,90,10", "1,inf,90,10", "1,2,nan,10", "1,2,90,10,nan",
                           "1,2,90,10oops", "1,2,90,1e999", "1e100,2,90,10", "1,2,90,10 # tail"}) {
        path_csv::ROW_S row;
        row.values[0] = 123;
        Check(path_csv::ParseRow(line, row) == path_csv::INVALID_ROW, "reject malformed row without exception");
        Check(row.values[0] == 123 && row.columns == 0, "invalid input never commits a partial row");
    }
    std::size_t total_rows = 0;
    for(int i = 1; i < argc; ++i) {
        std::ifstream input(argv[i]);
        Check(input.is_open(), "open migrated CSV");
        std::string line;
        std::size_t rows = 0;
        bool valid_file = true;
        while(std::getline(input, line)) {
            path_csv::ROW_S row;
            const auto status = path_csv::ParseRow(line, row);
            if(status == path_csv::SKIP_ROW) continue;
            if(status != path_csv::VALID_ROW || row.columns != 4 || row.values[3] < 0) valid_file = false;
            ++rows;
        }
        Check(!input.bad() && valid_file && rows > 0, "all map rows have four columns and retain valid configured speeds");
        total_rows += rows;
    }
    std::cout << "PASS path CSV: " << checks << " checks, " << argc - 1 << " files, " << total_rows << " rows\n";
}
