#include <clocale>

#include <string>
#include <fstream>

#include "CommonMini.hpp"
#include "Dat2csv.hpp"

int main(int argc, char** argv)
{
    SE_Options opt;
    opt.AddOption("file", "Simulation recording data file (.dat)", "filename");
    opt.AddOption("time_mode", "control timestamps in the csv (original, min_step, min_step_mixed)", "mode", "original");
    opt.AddOption("time_step", "use fixed time step (ms) - overrides time_mode", "time_step", "0.05");
    opt.AddOption("time_step_mixed", "use fixed time step (ms) - overrides time_mode", "time_step_mixed", "0.05");

    std::setlocale(LC_ALL, "C.UTF-8");

    if (opt.ParseArgs(argc, argv) != 0 || argc < 1)
    {
        opt.PrintUsage();
        return -1;
    }

    if (opt.GetOptionArg("file").empty())
    {
        printf("Missing file argument\n");
        opt.PrintUsage();
        return -1;
    }


    std::unique_ptr<Dat2csv> dat_to_csv;

    // Create dat2csv object for parsing the binary data file
    try
    {
        dat_to_csv = std::make_unique<Dat2csv>(opt.GetOptionArg("file"));
    }
    catch (const std::exception& e)
    {
        printf("%s", e.what());
        return -1;
    }

    log_mode log_mode_;
    if (opt.GetOptionSet("time_mode"))
    {
        std::string time_mode_str = opt.GetOptionArg("time_mode");
        if (!time_mode_str.empty())
        {
            if (time_mode_str == "original" )
            {
                log_mode_ = log_mode::ORIGINAL;
            }
            else if (time_mode_str == "min_step" )
            {
                log_mode_ = log_mode::MIN_STEP;
            }
            else if (time_mode_str == "min_step_mixed" )
            {
                log_mode_ = log_mode::MIN_STEP_MIXED;
            }
            else
            {
                LOG("Unsupported time mode: %s - using default (Original)", time_mode_str.c_str());
            }
        }
    }

    if (opt.GetOptionSet("time_step"))
    {
        std::string timeStep_str = opt.GetOptionArg("time_step");
        if (!timeStep_str.empty())
        {
            log_mode_ = log_mode::TIME_STEP;
            double delta_time_step = strtod(timeStep_str);
            dat_to_csv->SetStepTime(delta_time_step);
        }
        else
        {
            printf("Failed to provide fixed time step, Logging with default mode\n");
        }
    }

    if (opt.GetOptionSet("time_step_mixed"))
    {
        std::string timeStep_str = opt.GetOptionArg("time_step_mixed");
        if (!timeStep_str.empty())
        {
            log_mode_ = log_mode::TIME_STEP_MIXED;
            double delta_time_step = strtod(timeStep_str);
            dat_to_csv->SetStepTime(delta_time_step);
        }
        else
        {
            printf("Failed to provide fixed time step, Logging with default mode\n");
        }
    }

    dat_to_csv->SetLogMode(log_mode_);
    dat_to_csv->CreateCSV();

}