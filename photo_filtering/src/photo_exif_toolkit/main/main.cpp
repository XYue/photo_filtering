#include <iostream>
#include <gflags/gflags.h>

#include "exif_toolkit/exif_toolkit.hpp"


DEFINE_string(config_file,     "",       "file which contains the image paths and pos information.\n"
			  "if flag clean is set, pos info will not be used or this flag is optional.\n"
			  "when flag clean is not set, this flag will be neccesary.\n"
			  "\tconfig_file format:\n"
			  "\tfile_fullpath [longitude latitude altitude pitch roll yaw].");
DEFINE_string(dir,     "",       "(optional) directory which contains tif/jpg images the pos info of which is needed to be clean.");
DEFINE_bool(clean, false, "(optional) set this flag to clean the pos info in the images of dir or config_file.");

/*
config_file format:
file_fullpath [longitude latitude altitude pitch roll yaw]
*/

void main(int argc, char ** argv)
{
	google::ParseCommandLineFlags(&argc, &argv, true);

	do 
	{
		if (FLAGS_clean && FLAGS_config_file.empty() && FLAGS_dir.empty())
		{
			std::cout<<"Please set config_file or dir."<<std::endl;
			break;
		} 
		
		if (!FLAGS_clean && FLAGS_config_file.empty())
		{
			std::cout<<"invalid config_file."<<std::endl;
			break;
		}

		toolkit::ExifToolkit exif_tool;
		if (FLAGS_clean)
		{
			if (!FLAGS_config_file.empty())
			{
				if (exif_tool.CleanPosExif(FLAGS_config_file))
				{
					std::cout<<"CleanPosExif failed."<< FLAGS_config_file <<std::endl;
					break;
				}
			} 

			if (!FLAGS_dir.empty())
			{
				if (exif_tool.CleanPosExif(FLAGS_dir))
				{
					std::cout<<"CleanPosExif failed."<< FLAGS_dir <<std::endl;
					break;
				}
			}
		} else {
			if (exif_tool.UpdatePosExif(FLAGS_config_file))
			{
				std::cout<<"UpdatePosExif failed."<<std::endl;
				break;
			}
		}

	} while (0);

	std::cout<<"photo_exif_toolkit."<<std::endl;
}