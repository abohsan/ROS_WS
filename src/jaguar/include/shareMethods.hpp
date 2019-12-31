#ifndef __SHAREMETHODS_HPP__
#define __SHAREMETHODS_HPP__

#include <iostream>
#include <vector>
#include <string>
#include <sstream>

static bool startWith(std::string original, std::string prefix)
{
    if (original.substr(0, prefix.size()) == prefix)
    {
        return true;
    }
    else
    {
        return false;
    }
}

static std::vector<std::string> split(std::string original, std::string prefix, std::string splitBy)
{

    std::vector<std::string> result;
    original.erase(0, prefix.size());
    std::stringstream ss(original);
    while (ss.good())
    {
        std::string substr;
        getline(ss, substr, ',');
        result.push_back(substr);
    }
    return result;
}
static std::vector<std::string> split(std::string original, std::string splitBy)
{

    std::vector<std::string> result;
    std::stringstream ss(original);
    while (ss.good())
    {
        std::string substr;
        getline(ss, substr, ',');
        result.push_back(substr);
    }
    return result;
}

#endif