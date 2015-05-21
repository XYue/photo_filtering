#pragma once

#include <string>
#include <vector>

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

	public:
		PhotoFilter(std::string img_folder, std::string pos_file);
		~PhotoFilter();

		int Filter(std::string crop_kml, std::string output_folder);

	protected:
		int load_pos();

		int get_contour_from_kml(const std::string & crop_kml,
			std::vector<Point> & contour);

		void get_bounding_rect(const std::vector<Point> & contour,
			Point & min, Point & max);

		bool point_in_polygon(const Point pt,
			const std::vector<Point> & contour,
			const Point * min = NULL,
			const Point * max = NULL);

	private:
		std::string _image_folder;
		std::string _pos_file;

		std::vector<PhotoPOS> _photos;
	};
}