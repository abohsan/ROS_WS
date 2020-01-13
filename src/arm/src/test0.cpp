
#include "../include/File.hpp"
#include "../include/shareMethods.hpp"

std::string filePath = "configArm2.txt";
void write()
{
	std::ofstream oMyfile(filePath);
}
int main(int argc, char **argv)
{

	bool ddd = std::ifstream(filePath).good();

  std::ofstream myfile;
  myfile.open ("example1.txt");
//   myfile << "Writing this to a file.\n";
  myfile.close();

	return 0;
}
