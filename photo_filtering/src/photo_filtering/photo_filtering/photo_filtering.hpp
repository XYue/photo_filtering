#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

namespace filter
{
	class PhotoFilter
	{
	public:
		typedef struct PhotoPOS 
		{
			std::string day, time;

			double longitude, latitude, altitude;
			double pitch, roll, yaw;

			std::string filename;
		} PhotoPOS;

		typedef struct Point
		{
			double x, y, z;
		} Point;

		enum FilterType
		{
			FT_CAMERA_POS,
			FT_PROJECTED_POS
		};

	public:
		PhotoFilter(std::string img_folder, std::string pos_file);
		~PhotoFilter();

		int Filter(std::string crop_kml, std::string output_folder, 
			double plane_altitude = 0, const double * focal_length = NULL, FilterType filter_type = FT_CAMERA_POS);

	protected:
		int load_pos();

		int get_contour_from_kml(const std::string & crop_kml,
			std::vector<Point> & contour);

		void get_bounding_rect(const std::vector<Point> & contour,
			Point & min, Point & max);

		inline bool point_in_polygon(const Point pt,
			const std::vector<Point> & contour,
			const Point * min = NULL,
			const Point * max = NULL);

		inline bool point_in_polygon(const Eigen::Vector3d pt,
			const std::vector<Point> & contour,
			const Point * min = NULL,
			const Point * max = NULL);

		// a in b ?
		inline bool polygon_in_polygon(
			const std::vector<Point> & contour_a,
			const std::vector<Point> & contour_b);

		int proj_image_center(
			const double & pitch,
			const double & roll,
			const double & yaw,
			const double & img_width,
			const double & img_height,
			const double & focal,
			const Eigen::Vector3d & camera_pos,
			const Eigen::Vector4d & proj_plane,
			Eigen::Vector3d & proj_point);

		int proj_image_points(
			const double & pitch,
			const double & roll,
			const double & yaw,
			const double & img_width,
			const double & img_height,
			const double & focal,
			const Eigen::Vector3d & camera_pos,
			const Eigen::Vector4d & proj_plane,
			const std::vector<Eigen::Vector3d> & image_points,
			std::vector<Eigen::Vector3d> & proj_points);

		inline double cos_deg(const double x);
		inline double sin_deg(const double x);

		int filter_camera_pos(std::string crop_kml, 
			std::string output_folder);

		int filter_projected_pos(std::string crop_kml, 
			std::string output_folder, 
			double plane_altitude = 0.);

		int image_info_from_exif(
			const std::string image_filename,
			double & width, double & height, double & focal_length);


	private:
		std::string _image_folder;
		std::string _pos_file;

		std::vector<PhotoPOS> _photos;
	};
}