#include <iostream>

#include <gflags/gflags.h>

#include "photo_filtering/photo_filtering.hpp"

DEFINE_string(img_dir,     "",       "* image directory");
DEFINE_string(pos_file,     "",       "* pos fullpath");
DEFINE_string(aoi_kml,     "",       "* kml file with AOI");
DEFINE_string(out_dir,     "",       "* output directory");
DEFINE_string(filter_type,     "proj_center",       "[proj_center|camera_pos] (optional) set \"proj_center\" to filter photos with valid projected center, otherwise, filter with camera position instead");
DEFINE_double(focal_length, -1,      "(optional) focal length (mm)");
DEFINE_double(pixel_size, -1,      "(optional) pixel size (mm/pixel)");
DEFINE_double(ground_elev, 0.0,      "(optional) ground elevation (m)");

inline void EnableMemLeakCheck(void)
{
	_CrtSetDbgFlag(_CrtSetDbgFlag(_CRTDBG_REPORT_FLAG) | _CRTDBG_LEAK_CHECK_DF);
	//_CrtSetBreakAlloc(463);
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

// 	if (argc < 5 || argc > 6)
// 	{
// 		std::cout<<"ERROR: Invalid input arguments"<<std::endl;
// 		usage();
// 	} else {
// 		std::string img_folder = argv[1];
// 		std::string pos_file = argv[2];
// 		std::string crop_kml_file = argv[3];
// 		std::string output_folder = argv[4];
// 		double altitude = (argc == 6 ? atof(argv[5]) : 0);
// 
// 		filter::PhotoFilter photo_filter(img_folder, pos_file);
// 		if (photo_filter.Filter(crop_kml_file, output_folder, altitude, filter::PhotoFilter::FT_PROJECTED_POS))
// 		{
// 			std::cout<<"Filter failed."<<std::endl;
// 		}
// 	}


	google::ParseCommandLineFlags(&argc, &argv, true);

	do 
	{
		if (FLAGS_img_dir.empty())
		{
			std::cout<<"error: invalid img_dir"<<std::endl;
			
			break;
		}
		if (FLAGS_pos_file.empty())
		{
			std::cout<<"error: invalid pos_file"<<std::endl;
			break;
		}
		if (FLAGS_aoi_kml.empty())
		{
			std::cout<<"error: invalid aoi_kml"<<std::endl;
			break;
		}
		if (FLAGS_out_dir.empty())
		{
			std::cout<<"error: invalid out_dir"<<std::endl;
			break;
		}
		if (FLAGS_filter_type.compare("proj_center") &&
			FLAGS_filter_type.compare("camera_pos") )
		{
			std::cout<<"error: invalid filter_type: "<<FLAGS_filter_type<<std::endl;
			break;
		}

		double focal_len_in_pixel = 0;
		if (FLAGS_focal_length > DBL_EPSILON &&
			FLAGS_pixel_size > DBL_EPSILON)
		{
			focal_len_in_pixel = FLAGS_focal_length / FLAGS_pixel_size;
		}

		filter::PhotoFilter::FilterType filter_type = filter::PhotoFilter::FT_PROJECTED_POS;
		if (FLAGS_filter_type == "camera_pos")
			filter_type = filter::PhotoFilter::FT_CAMERA_POS;


		std::cout<<std::endl
			<<"img_dir: "<<FLAGS_img_dir<<std::endl
			<<"pos_file: "<<FLAGS_pos_file<<std::endl
			<<"aoi_kml: "<<FLAGS_aoi_kml<<std::endl
			<<"out_dir: "<<FLAGS_out_dir<<std::endl
			<<"filter_type: "<<FLAGS_filter_type<<std::endl
			<<"ground_elev: "<<FLAGS_ground_elev<<std::endl;
		if (focal_len_in_pixel > DBL_EPSILON)
			std::cout<<"focal_length_in_pixel: "<<focal_len_in_pixel<<std::endl;


		filter::PhotoFilter photo_filter(FLAGS_img_dir, FLAGS_pos_file);
		if (photo_filter.Filter(FLAGS_aoi_kml, FLAGS_out_dir,
			FLAGS_ground_elev, 
			focal_len_in_pixel > DBL_EPSILON ? &focal_len_in_pixel : NULL,
			filter_type))
		{
			std::cout<<"Filter failed."<<std::endl;
			break;
		} else {
			std::cout<<"Filter done."<<std::endl;
		}		

	} while (0);

	google::ShutDownCommandLineFlags();


	std::cout<<"photo filtering."<<std::endl;
}