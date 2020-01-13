#ifndef __FILE_HPP__
#define __FILE_HPP__
#include "../include/shareMethods.hpp"
#include <iostream>
#include <fstream>
#include <string>
// using namespace std;

class File
{
public:
    File();
    ~File();


    static File *getInstance()
    {
        if (s_instance == nullptr)
            s_instance = new File();

        return s_instance;
    }

   
 
    void testPrint();
    double getFlipers(int i);
    double getArm(int i);
    void setFlipers(double value, int i);
    void setArm(double value, int i);

private:
    int arryFilperSize;
    int arryArmSize;
    double flipers_array[4];
    double arm_array[7];
    static File *s_instance;
    std::string filePath;

    void setValue(std::vector<std::string> data, int index);
    void write(); 
    void read();
};
#endif