#include "photo_filtering.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

#include <tinyxml2.h>
#include <boost/filesystem.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <Eigen/LU>
#include <proj_api.h>
#include <exiv2.hpp>
#include <gdal_priv.h>

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

	int PhotoFilter::Filter( std::string crop_kml, 
		std::string output_folder, 
		double plane_altitude /*= 0*/, 
		const double * focal_length /*= NULL*/,
		FilterType filter_type /*= FT_CAMERA_POS*/ )
	{
		int ret = -1;

		do 
		{
			if (!boost::filesystem::is_directory(output_folder) ||
				!boost::filesystem::is_regular_file(crop_kml))
			{
				std::cout<<output_folder<<" is invalid. or"<<std::endl;
				std::cout<<crop_kml<<" is invalid."<<std::endl;
				break;
			}


			if (_photos.empty() && load_pos())
			{
				std::cout<<"load_pos failed."<<std::endl;
				break;
			}

			
			switch (filter_type)
			{
			case filter::PhotoFilter::FT_CAMERA_POS:
				if (filter_camera_pos(crop_kml, output_folder))
				{
					std::cout<<"filter_camera_pos failed."<<std::endl;
					goto error0;
				}
				break;
			case filter::PhotoFilter::FT_PROJECTED_POS:
				if (filter_projected_pos(crop_kml, output_folder, plane_altitude, focal_length))
				{
					std::cout<<"filter_projected_pos failed."<<std::endl;
					goto error0;
				}
				break;
			default:
				std::cout<<"Unsupported filter type: "<<filter_type<<std::endl;
				goto error0;
			}


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

	inline bool PhotoFilter::point_in_polygon( 
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

	inline bool PhotoFilter::point_in_polygon(const Eigen::Vector3d pt,
		const std::vector<Point> & contour, 
		const Point * min /*= NULL*/, 
		const Point * max /*= NULL*/)
	{
		Point p = {pt(0), pt(1), pt(2)};

		return point_in_polygon(p, contour, min, max);
	}

	int PhotoFilter::proj_image_center(
		const double & pitch,
		const double & roll,
		const double & yaw, 
		const double & img_width, 
		const double & img_height,
		const double & focal,
		const Eigen::Vector3d & camera_pos, 
		const Eigen::Vector4d & proj_plane,
		Eigen::Vector3d & proj_point)
	{
		int ret = -1;

		do 
		{
			if (img_width < DBL_EPSILON || img_height < DBL_EPSILON ||
				focal < DBL_EPSILON ||
				(proj_plane(0) < DBL_EPSILON && proj_plane(1) < DBL_EPSILON && 
				proj_plane(2) < DBL_EPSILON ) )
			{
				std::cout<<"Invalid input parameters: "
					<< img_width << " " << img_height << " " << focal << std::endl;
				std::cout<<proj_plane(0)<<" "<<proj_plane(1)<<" "
					<<proj_plane(2)<<" "<<proj_plane(3)<<std::endl;
				break;
			}


			// from lyh
			/*
			* cos_deg和sin_deg是针对原始cos和sin函数封装的功能相同的函数，
			* 只是输入参数的单位是度。
			* 另外，因为cos和sin函数在角度为+/-263附近会计算出错，故有此
			* 处理
			*/
			double cos_yaw = cos_deg(yaw);
			double sin_yaw = sin_deg(yaw);
			double cos_roll = cos_deg(roll);
			double sin_roll = sin_deg(roll);
			double cos_pitch = cos_deg(pitch);
			double sin_pitch = sin_deg(pitch);

			// 绕Z轴旋转Yaw角的旋转矩阵(陀螺仪坐标系)
			Eigen::Matrix3d mat_yaw;
			mat_yaw <<  cos_yaw, -sin_yaw,  0,
				sin_yaw,  cos_yaw,  0,
				0, 0, 1;

			// 绕X轴旋转Roll角的旋转矩阵(陀螺仪坐标系)
			Eigen::Matrix3d mat_roll;
			mat_roll <<  1,   0,   0,
				0,  cos_roll, -sin_roll,
				0,  sin_roll,  cos_roll;

			// 绕Y轴旋转Pitch角的旋转矩阵(陀螺仪坐标系)
			Eigen::Matrix3d mat_pitch;
			mat_pitch << cos_pitch,  0,  sin_pitch,
				0, 1, 0 ,
				-sin_pitch,  0,  cos_pitch;

			/*
			* 这里虽然有三个旋转角，但只包含两组旋转，即
			* 世界坐标系 -> 陀螺仪坐标系
			* 飞机坐标系 -> 相机坐标系
			* 其中，
			* (1) 陀螺仪坐标系是将世界坐标系先绕Z轴逆时针旋转90度，再绕X轴(
			*     注意是旋转后的X轴)逆时针旋转180度所得，对应于adjust1和
			*     adjust2;
			* (2) 相机坐标系是将飞机坐标系绕Z轴逆时针旋转90度所得，对应于
			*     adjust3；
			*/
			double adjust1 = -90.0;
			double adjust2 = -180.0;
			double adjust3 = -90.0;
			double cos_adjust1 = cos_deg(adjust1);
			double sin_adjust1 = sin_deg(adjust1);
			double cos_adjust2 = cos_deg(adjust2);
			double sin_adjust2 = sin_deg(adjust2);
			double cos_adjust3 = cos_deg(adjust3);
			double sin_adjust3 = sin_deg(adjust3);

			// adjusts
			Eigen::Matrix3d mat_adjust1;
			mat_adjust1 << cos_adjust1, -sin_adjust1,  0,
				sin_adjust1,  cos_adjust1,  0,
				0, 0, 1;
			Eigen::Matrix3d mat_adjust2;
			mat_adjust2 << 1, 0, 0,
				0,  cos_adjust2, -sin_adjust2,
				0,  sin_adjust2,  cos_adjust2;
			Eigen::Matrix3d mat_adjust3;
			mat_adjust3 << cos_adjust3, -sin_adjust3,  0,
				sin_adjust3,  cos_adjust3,  0,
				0, 0, 1;

			/*
			* 将三组变换组合成旋转矩阵，其中Yaw/Pitch/Roll三个旋转矩阵的组
			* 合顺序很重要，且由于是从“惯性坐标系”转换到“物体坐标系”，故需
			* 要进行逆变换
			*/
			Eigen::Matrix3d mat_rot =
				mat_adjust3 * (mat_yaw*mat_pitch*mat_roll).transpose() * mat_adjust2 * mat_adjust1;

			Eigen::Vector3d c ( 0.0, 0.0, focal);
			c  = mat_rot.inverse() * c +camera_pos;

			// 将五点投影到指定投影面上
			const double &A = proj_plane[0];
			const double &B = proj_plane[1];
			const double &C = proj_plane[2];
			const double &D = proj_plane[3];
			const double t =
				-(A*camera_pos[0] + B*camera_pos[1] + C*camera_pos[2]+D) / 
				(A*(c[0]-camera_pos[0]) + B*(c[1]-camera_pos[1]) + C*(c[2]-camera_pos[2]));
			proj_point = camera_pos + ( c - camera_pos) * t;


			ret = 0;
		} while (0);

		return ret;
	}

	inline double PhotoFilter::cos_deg(const double x)
	{
		int numPI = (int)(x/180.0);
		double rad = (x-180.0*numPI)*M_PI/180.0;
		return (numPI/2*2 == numPI ? cos(rad) : -cos(rad));
	}

	inline double PhotoFilter::sin_deg(const double x)
	{
		int numPI = (int)(x/180.0);
		double rad = (x-180.0*numPI)*M_PI/180.0;
		return (numPI/2*2 == numPI ? sin(rad) : -sin(rad));
	}

	int PhotoFilter::filter_camera_pos(std::string crop_kml, std::string output_folder)
	{
		int ret = -1;

		do 
		{
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

			std::vector<Point> contour;
			if (get_contour_from_kml(crop_kml, contour))
			{
				std::cout<<"get_contour_from_kml failed."<<std::endl;
				break;
			}

			if (_photos.empty() && load_pos())
			{
				std::cout<<"load_pos failed."<<std::endl;
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

		return ret;
	}

	int PhotoFilter::filter_projected_pos(
		std::string crop_kml, std::string output_folder,
		double plane_altitude /*= 0.*/, 
		const double * focal_length /*= NULL*/)
	{
		int ret = -1;

		projPJ sproj = NULL;
		projPJ dproj = NULL;

		do 
		{

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


			std::vector<Point> contour;
			if (get_contour_from_kml(crop_kml, contour))
			{
				std::cout<<"get_contour_from_kml failed."<<std::endl;
				break;
			}
			size_t num_contour = contour.size();

			// initialize projection
			if (num_contour > 0)
			{
				double west = std::numeric_limits<double>::max();
				for (int i_c = 0; i_c < num_contour; ++i_c)
					west = std::min(west, contour[i_c].x);

				int utm_zone = static_cast<int>(west / 6.) + 31;
				if (utm_zone <0 || utm_zone > 60) 
				{
					std::cout<<"Invalid UTM zone: "<<utm_zone<<std::endl;
					break;
				}
				std::stringstream sstr;	
				sstr << utm_zone;

				std::string src_proj = "+proj=longlat +datum=WGS84 +ellps=WGS84";
				std::string dst_proj = "+proj=utm +datum=WGS84 +ellps=WGS84 +zone=" + sstr.str();
				sproj = pj_init_plus(src_proj.c_str());
				dproj = pj_init_plus(dst_proj.c_str());
				if(!sproj || !dproj)  
				{
					std::cout<<"Initial proj parameters failed."<<std::endl;
					break;
				}
			}
			
			// boost polygon intersect
			typedef boost::geometry::model::d2::point_xy<double> Point_XY;
			typedef boost::geometry::model::polygon<Point_XY> polygon;
			polygon aoi_contour;

			// to bj54			
			for (int i_c = 0; i_c < num_contour; ++i_c)
			{
				double x = contour[i_c].x * DEG_TO_RAD;
				double y = contour[i_c].y * DEG_TO_RAD;
				if(pj_transform(sproj, dproj, 1, 0, 
					&x, &y, NULL))
				{
					std::cout<<"pj_transform failed at contour conversion. "
						<<contour[i_c].x<<" "<<contour[i_c].y<<" "<<contour[i_c].z<<std::endl;
					goto error0;
				}
				contour[i_c].x = x;
				contour[i_c].y = y;

				// boost polygon intersect
				boost::geometry::append(aoi_contour, Point_XY(x, y));
			}

			boost::geometry::correct(aoi_contour);


			if (_photos.empty() && load_pos())
			{
				std::cout<<"load_pos failed."<<std::endl;
				break;
			}

			
			Eigen::Vector4d proj_plane(0, 0, 1, -plane_altitude);
			std::ofstream dst_pos_file(pos_dir + "\\pos.fly");
			if (!dst_pos_file.good()) break;
			size_t num_photos = _photos.size();
			for (size_t i_p = 0; i_p < num_photos; ++i_p)
			{
				const PhotoPOS & photo = _photos[i_p];
				std::string image_fullname = _image_folder + "\\" + photo.filename;
				if (!boost::filesystem::exists(image_fullname)) continue;
				
				// read width, height and focal from exif
				double img_width, img_height, img_focal;
				if (focal_length && *focal_length > DBL_EPSILON)
				{
					img_focal = *focal_length;
					if (image_info_from_exif(image_fullname, img_width, img_height))
					{
						std::cout<<"image_info_from_exif failed. "<<image_fullname<<std::endl;
						continue;
					}
				} else {
					if (image_info_from_exif(image_fullname, img_width, img_height, img_focal))
					{
						std::cout<<"image_info_from_exif failed. "<<image_fullname<<std::endl;
						continue;
					}
				}
				

				// convert to utm
				Eigen::Vector3d cam_pos(photo.longitude, photo.latitude, photo. altitude);
				double x = cam_pos(0) * DEG_TO_RAD;
				double y = cam_pos(1) * DEG_TO_RAD;
				if(pj_transform(sproj, dproj, 1, 0, 
					&x, &y, NULL))
				{
					std::cout<<"pj_transform failed at pos conversion. "
						<<cam_pos(0)<<" "<<cam_pos(1)<<" "<<cam_pos(2)<<std::endl;
					continue;
				}
				cam_pos(0) = x;
				cam_pos(1) = y;


				// project image center to the plane
// 				Eigen::Vector3d proj_pos;
// 				if (proj_image_center(photo.pitch, photo.roll, photo.yaw,
// 					img_width, img_height, img_focal, 
// 					cam_pos, proj_plane, proj_pos))
// 				{
// 					std::cout<<"proj_image_center failed. "<<i_p<<std::endl;
// 					std::cout<<img_width<<" "<<img_height<<" "<<img_focal<<" "
// 						<<cam_pos(0)<<" "<<cam_pos(1)<<" "<<cam_pos(2)<<std::endl;
// 					continue;
// 				}								
// 
// 
// 				Point pt = {proj_pos(0), proj_pos(1), proj_pos(2)};	

				std::vector<Eigen::Vector3d> useful_pts(13);
				useful_pts[0] = Eigen::Vector3d(0, 0, img_focal);

				useful_pts[1] = Eigen::Vector3d(-img_width/2., -img_height/2., img_focal);
				useful_pts[2] = Eigen::Vector3d(-img_width/2., img_height/2., img_focal);
				useful_pts[3] = Eigen::Vector3d(img_width/2., img_height/2., img_focal);
				useful_pts[4] = Eigen::Vector3d(img_width/2., -img_height/2., img_focal);

				double boundary_unit_x = img_width / 8.;
				double boundary_unit_y = img_height / 8.;
				useful_pts[5] = Eigen::Vector3d(-img_width/2. + boundary_unit_x, -img_height/2. + boundary_unit_y, img_focal);
				useful_pts[6] = Eigen::Vector3d(-img_width/2. + boundary_unit_x, img_height/2. - boundary_unit_y, img_focal);
				useful_pts[7] = Eigen::Vector3d(img_width/2. - boundary_unit_x, img_height/2. -boundary_unit_y, img_focal);
				useful_pts[8] = Eigen::Vector3d(img_width/2. - boundary_unit_x, -img_height/2. + boundary_unit_y, img_focal);
				useful_pts[9] = Eigen::Vector3d(0, -boundary_unit_y, img_focal);
				useful_pts[10] = Eigen::Vector3d(0, boundary_unit_y, img_focal);
				useful_pts[11] = Eigen::Vector3d(-boundary_unit_x, 0., img_focal);
				useful_pts[12] = Eigen::Vector3d(boundary_unit_x, 0, img_focal);
				
				std::vector<Eigen::Vector3d> proj_pts;
				if (proj_image_points(photo.pitch, photo.roll, photo.yaw,
					img_width, img_height, img_focal, 
					cam_pos, proj_plane, useful_pts,proj_pts))
				{
					std::cout<<"proj_image_points failed. "<<i_p<<std::endl;
					std::cout<<img_width<<" "<<img_height<<" "<<img_focal<<" "
						<<cam_pos(0)<<" "<<cam_pos(1)<<" "<<cam_pos(2)<<std::endl;
					continue;
				}

				std::vector<Point> image_contour;
				int start_index = 1;
				for (int i_co = start_index; i_co < start_index + 4; ++i_co)
				{
					Point pt = {proj_pts[i_co](0), proj_pts[i_co](1), proj_pts[i_co](2)};
					image_contour.push_back(pt);
				}
				if (!quadrilateral_convex(image_contour)) continue;

				// image contour
				std::vector<Point> image_intersection_contour;
				start_index = 1;
				for (int i_co = start_index; i_co < start_index + 4; ++i_co)
				{
					Point pt = {proj_pts[i_co](0), proj_pts[i_co](1), proj_pts[i_co](2)};
					image_intersection_contour.push_back(pt);
				}

				// boost intersect				
				polygon camera_range;
				for (int i_co = 1; i_co < 5; ++i_co)
				{
					boost::geometry::append(camera_range, Point_XY(proj_pts[i_co](0), proj_pts[i_co](1)));
				}
				boost::geometry::correct(camera_range);


				if (/*point_in_polygon(pt, contour)*/
					/*point_in_polygon(proj_pts[0], contour) ||
					point_in_polygon(proj_pts[5], contour) ||
					point_in_polygon(proj_pts[6], contour) ||
					point_in_polygon(proj_pts[7], contour) ||
					point_in_polygon(proj_pts[8], contour) ||
					point_in_polygon(proj_pts[9], contour) ||
					point_in_polygon(proj_pts[10], contour) ||
					point_in_polygon(proj_pts[11], contour) ||
					point_in_polygon(proj_pts[12], contour) ||*/
					boost::geometry::intersects(aoi_contour, camera_range) ||
					polygon_in_polygon(contour, image_intersection_contour))
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

		if(dproj)
		{
			pj_free(dproj);
			dproj = NULL;
		}
		if(sproj)
		{
			pj_free(sproj);
			sproj = NULL;
		}

		return ret;
	}

	int PhotoFilter::image_info_from_exif(
		const std::string image_filename, 
		double & width, double & height, double & focal_length)
	{
		int ret = -1;

		CPLSetConfigOption("GDAL_FILENAME_IS_UTF8","NO");
		GDALAllRegister();

		do 
		{
			double tmp_w, tmp_h, tmp_f;

			GDALDataset * ds = (GDALDataset *)GDALOpen(image_filename.c_str(), GA_ReadOnly);
			if (ds)
			{
				tmp_w = ds->GetRasterXSize();
				tmp_h = ds->GetRasterYSize();

				GDALClose(ds);
			} else {
				std::cout<<"Failed to open "<<image_filename<<std::endl;
				break;
			}


			Exiv2::Image::AutoPtr exiv2_img = Exiv2::ImageFactory::open(image_filename);
			if(!(exiv2_img.get()))
			{
				std::cout<<"exiv2 open"<< image_filename <<" failed..."<<std::endl;
				break;
			}

			exiv2_img->readMetadata();
			Exiv2::ExifData & exif_data = exiv2_img->exifData();


// 			Exiv2::ExifData::const_iterator exif_width = exif_data.findKey(Exiv2::ExifKey("Exif.Photo.PixelXDimension"));
// 			if (exif_width != exif_data.end())
// 			{
// 				tmp_w = exif_width->toFloat();			
// 			} else {
// 				exif_width = exif_data.findKey(Exiv2::ExifKey("Exif.Image.ImageWidth"));
// 				if (exif_width == exif_data.end())
// 				{
// 					std::cout<<"ImageWidth invalid. "<<image_filename<<std::endl;
// 					break;
// 				}
// 				tmp_w = exif_width->toFloat();
// 			}
// 
// 			Exiv2::ExifData::const_iterator exif_height = exif_data.findKey(Exiv2::ExifKey("Exif.Photo.PixelYDimension"));
// 			if (exif_height != exif_data.end())
// 			{
// 				tmp_h = exif_height->toFloat();			
// 			} else {
// 				exif_height = exif_data.findKey(Exiv2::ExifKey("Exif.Image.ImageLength"));
// 				if (exif_height == exif_data.end())
// 				{
// 					std::cout<<"ImageHeight invalid. "<<image_filename<<std::endl;
// 					break;
// 				}
// 				tmp_h = exif_height->toFloat();
// 			}


			Exiv2::ExifData::const_iterator exif_focal = exif_data.findKey(Exiv2::ExifKey("Exif.Photo.FocalLength"));
			if (exif_focal == exif_data.end())
			{
				std::cout<<"FocalLength invalid. "<<image_filename<<std::endl;
				break;
			}
			Exiv2::ExifData::const_iterator exif_x_res = exif_data.findKey(Exiv2::ExifKey("Exif.Photo.FocalPlaneXResolution"));
			if (exif_x_res == exif_data.end())
			{
				std::cout<<"FocalPlaneXResolution invalid. "<<image_filename<<std::endl;
				break;
			}
			Exiv2::ExifData::const_iterator exif_unit = exif_data.findKey(Exiv2::ExifKey("Exif.Photo.FocalPlaneResolutionUnit"));
			if (exif_unit == exif_data.end())
			{
				std::cout<<"FocalPlaneResolutionUnit invalid. "<<image_filename<<std::endl;
				break;
			}
			double f = exif_focal->toFloat();
			double x_res = exif_x_res->toFloat();
			long unit = exif_unit->toLong();
			tmp_f = f * x_res;
			if (unit == 2) tmp_f /= 25.4;

			width = tmp_w;
			height = tmp_h;
			focal_length = tmp_f;

			ret = 0;
		} while (0);

		GDALDestroyDriverManager();
		
		return ret;
	}

	int PhotoFilter::image_info_from_exif(
		const std::string image_filename,
		double & width, double & height)
	{
		int ret = -1;

		CPLSetConfigOption("GDAL_FILENAME_IS_UTF8","NO");
		GDALAllRegister();

		do 
		{
			GDALDataset * ds = (GDALDataset *)GDALOpen(image_filename.c_str(), GA_ReadOnly);
			if (ds)
			{
				width = ds->GetRasterXSize();
				height = ds->GetRasterYSize();

				GDALClose(ds);
			} else {
				std::cout<<"Failed to open "<<image_filename<<std::endl;
				break;
			}

			ret = 0;
		} while (0);

		GDALDestroyDriverManager();

		return ret;
	}

	int PhotoFilter::proj_image_points(
		const double & pitch, const double & roll, const double & yaw, 
		const double & img_width, const double & img_height, const double & focal,
		const Eigen::Vector3d & camera_pos, 
		const Eigen::Vector4d & proj_plane, 
		const std::vector<Eigen::Vector3d> & image_points, 
		std::vector<Eigen::Vector3d> & proj_points)
	{
		int ret = -1;

		do 
		{
			if (image_points.empty())
			{
				std::cout<<"Empty image points."<<std::endl;
				break;
			}

			if (img_width < DBL_EPSILON || img_height < DBL_EPSILON ||
				focal < DBL_EPSILON ||
				(proj_plane(0) < DBL_EPSILON && proj_plane(1) < DBL_EPSILON && 
				proj_plane(2) < DBL_EPSILON ) )
			{
				std::cout<<"Invalid input parameters: "
					<< img_width << " " << img_height << " " << focal << std::endl;
				std::cout<<proj_plane(0)<<" "<<proj_plane(1)<<" "
					<<proj_plane(2)<<" "<<proj_plane(3)<<std::endl;
				break;
			}


			// from lyh
			/*
			* cos_deg和sin_deg是针对原始cos和sin函数封装的功能相同的函数，
			* 只是输入参数的单位是度。
			* 另外，因为cos和sin函数在角度为+/-263附近会计算出错，故有此
			* 处理
			*/
			double cos_yaw = cos_deg(yaw);
			double sin_yaw = sin_deg(yaw);
			double cos_roll = cos_deg(roll);
			double sin_roll = sin_deg(roll);
			double cos_pitch = cos_deg(pitch);
			double sin_pitch = sin_deg(pitch);

			// 绕Z轴旋转Yaw角的旋转矩阵(陀螺仪坐标系)
			Eigen::Matrix3d mat_yaw;
			mat_yaw <<  cos_yaw, -sin_yaw,  0,
				sin_yaw,  cos_yaw,  0,
				0, 0, 1;

			// 绕X轴旋转Roll角的旋转矩阵(陀螺仪坐标系)
			Eigen::Matrix3d mat_roll;
			mat_roll <<  1,   0,   0,
				0,  cos_roll, -sin_roll,
				0,  sin_roll,  cos_roll;

			// 绕Y轴旋转Pitch角的旋转矩阵(陀螺仪坐标系)
			Eigen::Matrix3d mat_pitch;
			mat_pitch << cos_pitch,  0,  sin_pitch,
				0, 1, 0 ,
				-sin_pitch,  0,  cos_pitch;

			/*
			* 这里虽然有三个旋转角，但只包含两组旋转，即
			* 世界坐标系 -> 陀螺仪坐标系
			* 飞机坐标系 -> 相机坐标系
			* 其中，
			* (1) 陀螺仪坐标系是将世界坐标系先绕Z轴逆时针旋转90度，再绕X轴(
			*     注意是旋转后的X轴)逆时针旋转180度所得，对应于adjust1和
			*     adjust2;
			* (2) 相机坐标系是将飞机坐标系绕Z轴逆时针旋转90度所得，对应于
			*     adjust3；
			*/
			double adjust1 = -90.0;
			double adjust2 = -180.0;
			double adjust3 = -90.0;
			double cos_adjust1 = cos_deg(adjust1);
			double sin_adjust1 = sin_deg(adjust1);
			double cos_adjust2 = cos_deg(adjust2);
			double sin_adjust2 = sin_deg(adjust2);
			double cos_adjust3 = cos_deg(adjust3);
			double sin_adjust3 = sin_deg(adjust3);

			// adjusts
			Eigen::Matrix3d mat_adjust1;
			mat_adjust1 << cos_adjust1, -sin_adjust1,  0,
				sin_adjust1,  cos_adjust1,  0,
				0, 0, 1;
			Eigen::Matrix3d mat_adjust2;
			mat_adjust2 << 1, 0, 0,
				0,  cos_adjust2, -sin_adjust2,
				0,  sin_adjust2,  cos_adjust2;
			Eigen::Matrix3d mat_adjust3;
			mat_adjust3 << cos_adjust3, -sin_adjust3,  0,
				sin_adjust3,  cos_adjust3,  0,
				0, 0, 1;

			/*
			* 将三组变换组合成旋转矩阵，其中Yaw/Pitch/Roll三个旋转矩阵的组
			* 合顺序很重要，且由于是从“惯性坐标系”转换到“物体坐标系”，故需
			* 要进行逆变换
			*/
			Eigen::Matrix3d mat_rot =
				mat_adjust3 * (mat_yaw*mat_pitch*mat_roll).transpose() * mat_adjust2 * mat_adjust1;
			
			// 将五点投影到指定投影面上
			const double &A = proj_plane[0];
			const double &B = proj_plane[1];
			const double &C = proj_plane[2];
			const double &D = proj_plane[3];
			size_t num_pts = image_points.size();
			proj_points.clear();
			proj_points.resize(num_pts);
			for (unsigned int i_p = 0; i_p < num_pts; ++i_p)
			{
				Eigen::Vector3d pt = image_points[i_p];

				if (pt(0) < -img_width/2. || pt(0) > img_width / 2. ||
					pt(1) < -img_height/2. || pt(1) > img_height / 2. )
				{
					std::cout<<"Invalid image points: "<<i_p<<std::endl;
					std::cout<<pt(0)<<" "<<pt(1)<<std::endl;
					goto error0;
				}

				pt = mat_rot.inverse() * pt +camera_pos;

				double t =
					-(A*camera_pos[0] + B*camera_pos[1] + C*camera_pos[2]+D) / 
					(A*(pt[0]-camera_pos[0]) + B*(pt[1]-camera_pos[1]) + C*(pt[2]-camera_pos[2]));

				proj_points[i_p] = camera_pos + ( pt - camera_pos) * t;
			}


			ret = 0;
		} while (0);
error0:

		return ret;
	}

	inline bool PhotoFilter::polygon_in_polygon(
		const std::vector<Point> & contour_a, 
		const std::vector<Point> & contour_b)
	{
		size_t num_pta = contour_a.size();
		for (int i_pa = 0; i_pa < num_pta; ++i_pa)
		{
			if (!point_in_polygon(contour_a[i_pa], contour_b))
			{
				return false;
			}
		}

		return true;
	}

	bool PhotoFilter::quadrilateral_convex(const std::vector<Point> & vertices)
	{
		bool ret = false;

		do 
		{
			if (vertices.size() != 4) break;

			double a1 = vertices[2].y - vertices[0].y;
			double b1 = vertices[0].x - vertices[2].x;
			double c1 = (vertices[2].x - vertices[0].x) * vertices[0].y -
				(vertices[2].y - vertices[0].y) * vertices[0].x;

			double a2 = vertices[3].y - vertices[1].y;
			double b2 = vertices[1].x - vertices[3].x;
			double c2 = (vertices[3].x - vertices[1].x) * vertices[1].y -
				(vertices[3].y - vertices[1].y) * vertices[1].x;

			double intersection_x = (b1 * c2 - b2 * c1) / (a1 * b2 - a2 * b1);
			double intersection_y = (a1 * c2 - a2 * c1) / (a2 * b1 - a1 * b2);
			
			double min_x, max_x, min_y, max_y;
			min_x = std::min(vertices[0].x, vertices[2].x);
			max_x = std::max(vertices[0].x, vertices[2].x);
			min_y = std::min(vertices[0].y, vertices[2].y);
			max_y = std::max(vertices[0].y, vertices[2].y);
			if (intersection_x < min_x || intersection_x > max_x ||
				intersection_y < min_y || intersection_y > max_y) break;

			min_x = std::min(vertices[1].x, vertices[3].x);
			max_x = std::max(vertices[1].x, vertices[3].x);
			min_y = std::min(vertices[1].y, vertices[3].y);
			max_y = std::max(vertices[1].y, vertices[3].y);
			if (intersection_x < min_x || intersection_x > max_x ||
				intersection_y < min_y || intersection_y > max_y) break;

			ret = true;
		} while (0);

		return ret;
	}

}
