#include <iostream>

#include "photo_filtering/photo_filtering.hpp"

inline void EnableMemLeakCheck(void)
{
	_CrtSetDbgFlag(_CrtSetDbgFlag(_CRTDBG_REPORT_FLAG) | _CRTDBG_LEAK_CHECK_DF);
	//_CrtSetBreakAlloc(15560);
}

void usage()
{
	std::cout<<std::endl;
	std::cout<<"Input format:"<<std::endl;
	std::cout<<"\tphoto_filtering.exe IMAGE_FOLDER POS_FILE CROP_KML OUTPUT_FOLDER [GOUND_ELEVATION]"<<std::endl;
	std::cout<<std::endl;
}

void main(int argc, char ** argv)
{
	EnableMemLeakCheck();

	if (argc < 5 || argc > 6)
	{
		std::cout<<"ERROR: Invalid input arguments"<<std::endl;
		usage();
	} else {
		std::string img_folder = argv[1];
		std::string pos_file = argv[2];
		std::string crop_kml_file = argv[3];
		std::string output_folder = argv[4];
		double altitude = (argc == 6 ? atof(argv[5]) : 0);

		filter::PhotoFilter photo_filter(img_folder, pos_file);
		if (photo_filter.Filter(crop_kml_file, output_folder, altitude, filter::PhotoFilter::FT_PROJECTED_POS))
		{
			std::cout<<"Filter failed."<<std::endl;
		}
	}

	std::cout<<"photo filtering."<<std::endl;
}