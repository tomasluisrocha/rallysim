#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "core/telemetry.hpp"
#include "tests/check.hpp"

using namespace rallysim;

int main()
{
    const std::string path = "telemetry_test_output.csv";
    {
        telemetry_writer w(path, 0.001, {"step", "time", "pos.x"}, {"count", "s", "m"});
        check_true("writer opens a valid path", w.ok());
        w.row({0, 0.0, 0.1});
        w.row({1, 0.001, 1.0 / 3.0});
        w.row({2, 0.002, -2.5e-10});
        check_true("writer still ok after writing", w.ok());
    } // closes the file

    std::ifstream in(path, std::ios::binary);
    std::string l1, l2, r0, r1, r2, extra;
    std::getline(in, l1);
    std::getline(in, l2);
    std::getline(in, r0);
    std::getline(in, r1);
    std::getline(in, r2);
    const bool has_extra = static_cast<bool>(std::getline(in, extra));
    in.close();

    check_true("line 1 starts with schema 2", l1.rfind("# schema=2 dt=0.001", 0) == 0);
    check_true("line 1 lists units", l1.find("step[count] time[s] pos.x[m]") != std::string::npos);
    check_true("line 2 is the column names", l2 == "step,time,pos.x");
    check_true("row 0 text", r0 == "0,0,0.10000000000000001");
    check_true("no extra lines", !has_extra);

    // a value must read back as exactly the same double
    {
        std::stringstream ss(r1);
        std::string field;
        std::getline(ss, field, ',');
        std::getline(ss, field, ',');
        std::getline(ss, field, ',');
        check_true("1/3 round trips exactly", std::stod(field) == 1.0 / 3.0);
        std::stringstream s2(r2);
        std::getline(s2, field, ',');
        std::getline(s2, field, ',');
        std::getline(s2, field, ',');
        check_true("-2.5e-10 round trips exactly", std::stod(field) == -2.5e-10);
    }

    check_true("no carriage returns in the file", [&]
               {
        std::ifstream f(path, std::ios::binary);
        std::string all((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
        return all.find('\r') == std::string::npos; }());

    std::remove(path.c_str());

    {
        telemetry_writer bad("no_such_folder_xyz/out.csv", 0.001, {"a"}, {"m"});
        check_true("unwritable path reports not ok", !bad.ok());
    }

    return finish("telemetry");
}
