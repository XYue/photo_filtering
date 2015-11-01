#include <iostream>

#include <gflags/gflags.h>

#include "converter\pos_kml_converter.hpp"

DEFINE_string(pos_file,     "",       "* pos fullpath");
DEFINE_string(img_folder,     "",       "* folder which contains images with pos exif infomation.");
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

void main(int argc, char ** argv)
{
	EnableMemLeakCheck();

	google::ParseCommandLineFlags(&argc, &argv, true);

	do 
	{
		if (FLAGS_pos_file.empty() && FLAGS_img_folder.empty())
		{
			std::cout<<"error: invalid pos_file or image folder"<<std::endl;
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

		std::string path;
		bool is_pos_file = false;
		if (!FLAGS_pos_file.empty())
		{
			path = FLAGS_pos_file;
			is_pos_file = true;
		} else if (!FLAGS_img_folder.empty()) {
			path = FLAGS_img_folder;
			is_pos_file = false;
		} else {
			std::cout<<"error: invalid pos_file or image folder"<<std::endl;
			break;
		}
		//cvt::PosKmlConverter cvt(FLAGS_pos_file);
		cvt::PosKmlConverter cvt(path, is_pos_file);
		if (cvt.Convert(FLAGS_kml_file, option,
			FLAGS_img_width, FLAGS_img_height, focal_in_pixel,
			FLAGS_ground_elev))
		{
			std::cout<<"Convert failed."<<std::endl;
		}
	} while (0);

	std::cout<<"photo filtering."<<std::endl;
}