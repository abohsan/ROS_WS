#include "../include/File.hpp"

File *File::s_instance = nullptr;

File::File()
{
    arryFilperSize = 4;
    arryArmSize = 7;
    filePath = "ROS_WS/configArm.txt";

    for (int i = 0; i < arryFilperSize; i++)
        flipers_array[i] = 0;

    for (int i = 0; i < arryArmSize; i++)
        arm_array[i] = 0;

    if(!std::ifstream(filePath).good()){
        write();
    }else{
       read();  
    }
   
}

void File::read()
{
    std::string line;
    std::vector<std::string> data;
    std::vector<std::string> flippers_vector;
    std::vector<std::string> arm_vector;

    std::ifstream myfile(filePath);
    if (myfile.is_open())
    {
        while (std::getline(myfile, line))
        {
            data = split(line, ':');
            setValue(split(data[0], ','), 0);
            setValue(split(data[1], ','), 1);
        }
        myfile.close();
    }
    else
        print("Unable to open file") ;
}
void File::setValue(std::vector<std::string> data, int index)
{
    if (index == 0)
    {
        for (int i = 0; i < arryFilperSize; i++)
        {
            if (i < data.size())
                flipers_array[i] = std::stod(data[i]);
        }
    }
    else if (index == 1)
    {
        for (int i = 0; i < arryArmSize; i++)
        {
            if (i < data.size())
                arm_array[i] = std::stod(data[i]);
        }
    }
}

void File::write()
{
  std::ofstream myfile;
  myfile.open (filePath);
  
        for (int i = 0; i < arryFilperSize; i++)
        {
            myfile << toString(flipers_array[i]);
            if (i != (arryFilperSize - 1))
            {
                myfile << ",";
            }
            else
            {
                myfile << ":";
            }
        }
        for (int i = 0; i < arryArmSize; i++)
        {
            myfile << toString(arm_array[i]);
            if (i != (arryArmSize - 1))
            {
                myfile << ",";
            }
        }
        myfile.close();

}

void File::testPrint()
{
    for (int i = 0; i < arryFilperSize; i++)
        print(flipers_array[i]);

    std::cout << "\nArm Values ---------------------------\n"
              << std::endl;

    for (int i = 0; i < arryArmSize; i++)
        print(arm_array[i]);
}

void File::setFlipers(double value, int i)
{
    if (i >= 0 && i < arryFilperSize)
        flipers_array[i] = value;

    write();
}

void File::setArm(double value, int i)
{
    if (i >= 0 && i < arryArmSize)
        arm_array[i] = value;
    write();
}

double File::getFlipers(int i)
{
    if (i >= 0 && i < arryFilperSize)
        return flipers_array[i];
    else
        return -1;
}

double File::getArm(int i)
{
    if (i >= 0 && i < arryArmSize)
        return arm_array[i];
    else
        return -1;
}

File::~File()
{
    delete s_instance;
    print("File::~File()");
}
