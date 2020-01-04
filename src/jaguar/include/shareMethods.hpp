#ifndef __SHAREMETHODS_HPP__
#define __SHAREMETHODS_HPP__

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <math.h>
static double degreesToRadians(double angle_in_degrees)
{

    return angle_in_degrees * (M_PI / 180.0);
}

static double radiansToDegrees(double angle_in_radians)
{
    return angle_in_radians * (180.0 / M_PI);
}

static bool startWith(std::string original, std::string prefix)
{
    if (original.substr(0, prefix.size()) == prefix)
        return true;
    else
        return false;
}

/*
    original the string that need to analyzied 
    prefix = string will be removed 
    

*/
static std::vector<std::string> split(std::string original, std::string prefix, char splitBy)
{

    std::vector<std::string> result;
    original.erase(0, prefix.size());
    std::stringstream ss(original);
    while (ss.good())
    {
        std::string substr;
        getline(ss, substr, splitBy);
        result.push_back(substr);
    }
    return result;
}
static std::vector<std::string> split(std::string original, char splitBy)
{

    std::vector<std::string> result;
    std::stringstream ss(original);
    while (ss.good())
    {
        std::string substr;
        getline(ss, substr, splitBy);
        result.push_back(substr);
    }
    return result;
}
static void print(std::string str)
{
    std::cout << str << std::endl;
}

static void print(double str)
{
    std::cout << str << std::endl;
}

static void print(std::vector<std::string> data)
{
    for (int i = 0; i < data.size(); i++)
        print(data[i]);
}

static std::string toString(double value)
{
    std::stringstream ss;
    ss << value;
    return ss.str();
}

#endif