#include <iostream>

#include <gflags/gflags.h>

#include "converter\pos_kml_converter.hpp"

DEFINE_string(pos_file,     "",       "* pos fullpath");
DEFINE_string(kml_file,     "",       "* output kml file");
DEFINE_bool(output_point, false, "(optional) output camera positions in kml file.");
DEFINE_bool(output_proj_image, false, "(optional) output camera projections in kml file.");
DEFINE_bool(output_camera_linestring, true, "(optional) output camera position as linestring in kml file.");
DEFINE_double(img_width, 0.0,      "(optional) image width (pixel)");
DEFINE_double(img_height, 0.0,      "(optional) image height (pixel)");
DEFINE_double(focal_length, 0.0,      "(optional) focal length (mm)");
DEFINE_double(pixel_size, 0.0,      "(optional) pixel size (mm per pixel)");
DEFINE_double(ground_elev, 0.0,      "(optional) ground elevation (m)");

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

// 	if (argc != 3)
// 	{
// 		std::cout<<"ERROR: Invalid input arguments"<<std::endl;
// 		usage();
// 	} else {
// 		std::string pos_file = argv[1];
// 		std::string output_kml = argv[2];
// 
// 		cvt::PosKmlConverter cvt(pos_file);
// 		if (cvt.Convert(output_kml))
// 		{
// 			std::cout<<"Convert failed."<<std::endl;
// 		}
// 	}


	google::ParseCommandLineFlags(&argc, &argv, true);

	do 
	{
		if (FLAGS_pos_file.empty())
		{
			std::cout<<"error: invalid pos_file"<<std::endl;
			break;
		}

		if (FLAGS_kml_file.empty())
		{
			std::cout<<"error: invalid pos_file"<<std::endl;
			break;
		}

		cvt::PosKmlConverter::OutputOption option = cvt::PosKmlConverter::OutputOption::OO_NONE;
		if (FLAGS_output_point) option = static_cast<cvt::PosKmlConverter::OutputOption>
			(option | cvt::PosKmlConverter::OutputOption::OO_POINT);
		if (FLAGS_output_proj_image) option = static_cast<cvt::PosKmlConverter::OutputOption>
			(option | cvt::PosKmlConverter::OutputOption::OO_PROJ_IMAGE);
		if (FLAGS_output_camera_linestring) option = static_cast<cvt::PosKmlConverter::OutputOption>
			(option | cvt::PosKmlConverter::OutputOption::OO_LINE_STRING);

		double focal_in_pixel = FLAGS_pixel_size > DBL_EPSILON ? FLAGS_focal_length/ FLAGS_pixel_size : 0.;

		cvt::PosKmlConverter cvt(FLAGS_pos_file);
		if (cvt.Convert(FLAGS_kml_file, option,
			FLAGS_img_width, FLAGS_img_height, focal_in_pixel,
			FLAGS_ground_elev))
		{
			std::cout<<"Convert failed."<<std::endl;
		}
	} while (0);

	std::cout<<"photo filtering."<<std::endl;
}