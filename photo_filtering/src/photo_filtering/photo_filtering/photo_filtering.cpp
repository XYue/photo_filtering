#include "photo_filtering.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

#include <tinyxml2.h>
#include <boost/filesystem.hpp>


#undef  SAFE_TEXT
#define SAFE_TEXT(txt)  (txt ? txt : "")

namespace filter
{

	PhotoFilter::PhotoFilter( std::string img_folder, std::string pos_file ) :
		_image_folder(img_folder), _pos_file(pos_file)
	{

	}

	PhotoFilter::~PhotoFilter()
	{

	}

	int PhotoFilter::Filter( std::string crop_kml, std::string output_folder )
	{
		int ret = -1;

		do 
		{
			if (!boost::filesystem::is_directory(output_folder))
			{
				std::cout<<output_folder<<" is invalid."<<std::endl;
				break;
			}

			std::string pos_dir = output_folder + "\\pos";
			if (!boost::filesystem::exists(pos_dir) &&
				!boost::filesystem::create_directory(pos_dir))
			{
				std::cout<<"create "<<pos_dir<<" failed."<<std::endl;
				break;
			}

			std::string img_dir = output_folder + "\\img";
			if( !boost::filesystem::exists(img_dir) &&
				!boost::filesystem::create_directory(img_dir))
			{
				std::cout<<"create "<<img_dir<<" failed."<<std::endl;
				break;
			}


			if (_photos.empty() && load_pos())
			{
				std::cout<<"load_pos failed."<<std::endl;
				break;
			}

			std::vector<Point> contour;
			if (get_contour_from_kml(crop_kml, contour))
			{
				std::cout<<"get_contour_from_kml failed."<<std::endl;
				break;
			}

			std::ofstream dst_pos_file(pos_dir + "\\pos.fly");
			if (!dst_pos_file.good()) break;
			size_t num_photos = _photos.size();
			for (size_t i_p = 0; i_p < num_photos; ++i_p)
			{
				const PhotoPOS & photo = _photos[i_p];
				std::string image_fullname = _image_folder + "\\" + photo.filename;
				if (!boost::filesystem::exists(image_fullname)) continue;

				Point pt = {photo.longitude, photo.latitude, photo.altitude};	
				if (point_in_polygon(pt, contour))
				{
					std::string dst_image_fullname = img_dir + "\\" + photo.filename;
					boost::filesystem::copy_file(image_fullname,
						dst_image_fullname, 
						boost::filesystem::copy_option::overwrite_if_exists);

					if (dst_pos_file.good())
					{
						dst_pos_file << i_p <<" "
							<< photo.day <<" "
							<< photo.time <<" "
							<< std::setprecision(15)
							<< photo.longitude <<" "
							<< photo.latitude <<" "
							<< photo.altitude <<" "
							<< photo.pitch <<" "
							<< photo.roll <<" "
							<< photo.yaw <<" "
							<< dst_image_fullname << std::endl;
					}
				}
			}
			dst_pos_file.close();


			ret = 0;
		} while (0);
error0:

		return ret;
	}

	int PhotoFilter::load_pos()
	{
		int ret = -1;
		
		do 
		{
			std::ifstream file(_pos_file);
			if (!file.good()) break;


			std::vector<PhotoPOS> tmp_photos;


			std::string line;
			std::stringstream sstr;
			std::getline(file, line);
			while (file.good())
			{
				PhotoPOS photo;

				sstr.clear(); sstr.str("");
				sstr << line;

				std::string temp_str;
				sstr >> temp_str; // id

				sstr >> photo.day; // day
				sstr >> photo.time; // time

				sstr >> photo.longitude;
				sstr >> photo.latitude;
				sstr >> photo.altitude;

				sstr >> photo.pitch; // pitch
				sstr >> photo.roll; // roll
				sstr >> photo.yaw; // yaw

				sstr >> photo.filename;
				if (!photo.filename.empty())
				{
					photo.filename = photo.filename.substr(photo.filename.find_last_of("\\")+1);

					tmp_photos.push_back(photo);
				}

				line.clear();
				std::getline(file, line);
			}


			_photos.swap(tmp_photos);

			ret = 0;
		} while (0);
error0:

		return ret;
	}

	int PhotoFilter::get_contour_from_kml( 
		const std::string & crop_kml, 
		std::vector<Point> & contour )
	{
		int ret = -1;
		
		do 
		{
			std::vector<Point> tmp_contour;


			// find coordinates of the first valid polygon
			tinyxml2::XMLDocument doc;
			if (doc.LoadFile(crop_kml.c_str()) != tinyxml2::XML_NO_ERROR)
				goto error0;

			std::string polygon_name = "LinearRing";
			std::string coor_name = "coordinates";
			tinyxml2::XMLElement * elem = doc.RootElement();
			while (elem) 
			{
				if (!std::string(elem -> Value()).compare(polygon_name)) 
				{
					tinyxml2::XMLElement * coor = elem->FirstChildElement(coor_name.c_str());
					if (coor)
					{
						elem = coor;
						break;
					} else continue;
				}
				if (elem -> FirstChildElement())
					elem = elem -> FirstChildElement();
				else if (elem -> NextSiblingElement()) 
					elem = elem -> NextSiblingElement();
				else 
				{
					while (!elem -> Parent() -> NextSiblingElement()) 
					{
						if (elem -> Parent() -> ToElement() == doc.RootElement())
							goto error0;
						elem = elem -> Parent()->ToElement();
					}
					elem = elem -> Parent() -> NextSiblingElement();
				}
			}

			if ( !elem || std::string(elem -> Value()).compare(coor_name))
				goto error0;


			// read coordinates
			std::stringstream sstr;
			std::string tmp_str = SAFE_TEXT(elem->GetText());
			sstr << tmp_str;

			tmp_str.clear(); tmp_str = "";
			sstr >> tmp_str;
			while (!tmp_str.empty())
			{
				Point vertex;

				vertex.x = std::stod(tmp_str.substr(0, tmp_str.find_first_of(",")));
				tmp_str = tmp_str.substr(tmp_str.find_first_of(",")+1);

				vertex.y = std::stod(tmp_str.substr(0, tmp_str.find_first_of(",")));
				tmp_str = tmp_str.substr(tmp_str.find_first_of(",")+1);

				vertex.z = std::stod(tmp_str.substr(0, tmp_str.find_first_of(",")));

				tmp_contour.push_back(vertex);

				tmp_str.clear(); tmp_str = "";
				sstr >> tmp_str;
			}			

			if (tmp_contour.size() < 3) goto error0;


			contour.swap(tmp_contour);

			ret = 0;
		} while (0);
error0:

		return ret;
	}

	void PhotoFilter::get_bounding_rect( 
		const std::vector<Point> & contour,
		Point & min, Point & max )
	{
		min.x = std::numeric_limits<double>::max();
		min.y = std::numeric_limits<double>::max();
		min.z = std::numeric_limits<double>::max();

		max.x = std::numeric_limits<double>::min();
		max.y = std::numeric_limits<double>::min();
		max.z = std::numeric_limits<double>::min();

		int num_vertices = contour.size();
		for (int i_v = 0; i_v < num_vertices; ++i_v)
		{
			const Point & p = contour[i_v];

			min.x = p.x < min.x ? p.x : min.x;
			min.y = p.y < min.y ? p.y : min.y;
			min.z = p.z < min.z ? p.z : min.z;

			max.x = p.x > max.x ? p.x : max.x;
			max.y = p.y > max.y ? p.y : max.y;
			max.z = p.z > max.z ? p.z : max.z;
		}
	}

	bool PhotoFilter::point_in_polygon( 
		const Point pt, 
		const std::vector<Point> & contour, 
		const Point * min /*= NULL*/,
		const Point * max /*= NULL*/ )
	{
		bool be_in = false;

		if (min && max && 
			( pt.x < (*min).x ||
			pt.x > (*max).x ||
			pt.y < (*min).y ||
			pt.y > (*max).y))
			return be_in;

		int num_vertices = contour.size();
		int i, j;
		for (i = 0, j = num_vertices - 1; i < num_vertices; j = i++)
		{
			if ( ( (contour[i].y>pt.y) != (contour[j].y>pt.y) ) && 
				(pt.x < (contour[j].x-contour[i].x) * (pt.y-contour[i].y) / (contour[j].y-contour[i].y) + contour[i].x) ) 
				be_in = !be_in; 
		}


		return be_in;
	}

}
