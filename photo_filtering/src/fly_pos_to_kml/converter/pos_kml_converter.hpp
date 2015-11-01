#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

namespace cvt
{
	class PosKmlConverter
	{
	public:
		typedef enum
		{
			OO_NONE = 0,
			OO_POINT = 1,
			OO_PROJ_IMAGE = 2,
			OO_LINE_STRING = 4
		} OutputOption;

		typedef struct PhotoPOS 
		{
			double pitch, roll, yaw;

			double longitude;
			double latitude;
			double altitude;

			std::string filename;
		} PhotoPOS;

	public:
		PosKmlConverter(std::string pos_file);
		PosKmlConverter(std::string path, bool is_pos_file);
		~PosKmlConverter();

		int Convert(std::string kml_file,
			OutputOption option = OO_LINE_STRING,
			double img_width = 0., double img_height = 0., double focal = 0.,
			double groud_elevation = 0.);

	protected:
		int load_pos();

		int parse_photo_folder();

		int proj_image_points(
			const double & pitch, const double & roll, const double & yaw, 
			const double & img_width, const double & img_height, const double & focal,
			const Eigen::Vector3d & camera_pos, 
			const Eigen::Vector4d & proj_plane, 
			const std::vector<Eigen::Vector3d> & image_points, 
			std::vector<Eigen::Vector3d> & proj_points);

		int test_utm_zone();

		inline double cos_deg(const double x);
		inline double sin_deg(const double x);

	private:
		std::string _photo_folder;
		std::string _pos_file;

		std::vector<PhotoPOS> _photos;
	};
}