#pragma once

#include <string>
#include <vector>

namespace toolkit
{
	class ExifToolkit
	{
	public:
		typedef struct PhotoPos 
		{
			std::string filename;
			
			double longitude, latitude, altitude;
			double pitch, roll, yaw;
		} PhotoPos;

	public:
		ExifToolkit(){};
		~ExifToolkit(){};

		int CleanPosExif(const std::string & input_path);
		int UpdatePosExif(const std::string & config_file);

	protected:
		int parse_config_file(
			const std::string & config_file,
			std::vector<PhotoPos> & poses);
		int parse_config_file(
			const std::string & config_file,
			std::vector<std::string> & filenames);

		int parse_dir(
			const std::string & dir,
			std::vector<std::string> & filenames);

		inline bool img_ext_satisfied( const std::string & ext );

		std::string double_to_gps_rational_string( double value );
	};

}