#include <iostream>

#include "converter\pos_kml_converter.hpp"

inline void EnableMemLeakCheck(void)
{
	_CrtSetDbgFlag(_CrtSetDbgFlag(_CRTDBG_REPORT_FLAG) | _CRTDBG_LEAK_CHECK_DF);
	//_CrtSetBreakAlloc(15560);
}

void usage()
{
	std::cout<<std::endl;
	std::cout<<"Input format:"<<std::endl;
	std::cout<<"fly_pos_to_kml.exe POS_FILE OUTPUT_KML"<<std::endl;
	std::cout<<std::endl;
}

void main(int argc, char ** argv)
{
	EnableMemLeakCheck();

	if (argc != 3)
	{
		std::cout<<"ERROR: Invalid input arguments"<<std::endl;
		usage();
	} else {
		std::string pos_file = argv[1];
		std::string output_kml = argv[2];

		cvt::PosKmlConverter cvt(pos_file);
		if (cvt.Convert(output_kml))
		{
			std::cout<<"Convert failed."<<std::endl;
		}
	}

	std::cout<<"photo filtering."<<std::endl;
}