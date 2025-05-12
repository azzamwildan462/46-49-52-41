#ifndef SYSTEM_UTILS_HPP_
#define SYSTEM_UTILS_HPP_

#include <string>
#include <array>
#include <memory>
#include <stdexcept>

#include <vector>
#include <string>
#include <sstream>

std::vector<std::string> str_explode(const std::string &str, char delimiter)
{
    std::vector<std::string> result;
    std::stringstream ss(str);
    std::string token;

    while (std::getline(ss, token, delimiter))
    {
        result.push_back(token);
    }

    return result;
}

std::string execute_command(const std::string &command, const std::string &newline_replace = "\0")
{
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(command.c_str(), "r"), pclose);
    if (!pipe)
    {
        throw std::runtime_error("popen() failed!");
    }
    while (!feof(pipe.get()))
    {
        if (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
        {
            result += buffer.data();
        }
    }

    // Replace '\n' with newline_replace in the result string
    for (char &c : result)
    {
        if (c == '\n')
        {
            c = newline_replace.c_str()[0];
        }
    }

    return result;
}

#endif // SYSTEM_UTILS_HPP_