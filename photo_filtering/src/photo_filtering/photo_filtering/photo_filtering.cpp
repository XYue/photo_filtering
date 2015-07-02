#include "photo_filtering.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

#include <tinyxml2.h>
#include <boost/filesystem.hpp>
#include <Eigen/LU>
#include <proj_api.h>
#include <exiv2.hpp>

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
				if (filter_projected_pos(crop_kml, output_folder, plane_altitude))
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
			* cos_deg��sin_deg�����ԭʼcos��sin������װ�Ĺ�����ͬ�ĺ�����
			* ֻ����������ĵ�λ�Ƕȡ�
			* ���⣬��Ϊcos��sin�����ڽǶ�Ϊ+/-263���������������д�
			* ����
			*/
			double cos_yaw = cos_deg(yaw);
			double sin_yaw = sin_deg(yaw);
			double cos_roll = cos_deg(roll);
			double sin_roll = sin_deg(roll);
			double cos_pitch = cos_deg(pitch);
			double sin_pitch = sin_deg(pitch);

			// ��Z����תYaw�ǵ���ת����(����������ϵ)
			Eigen::Matrix3d mat_yaw;
			mat_yaw <<  cos_yaw, -sin_yaw,  0,
				sin_yaw,  cos_yaw,  0,
				0, 0, 1;

			// ��X����תRoll�ǵ���ת����(����������ϵ)
			Eigen::Matrix3d mat_roll;
			mat_roll <<  1,   0,   0,
				0,  cos_roll, -sin_roll,
				0,  sin_roll,  cos_roll;

			// ��Y����תPitch�ǵ���ת����(����������ϵ)
			Eigen::Matrix3d mat_pitch;
			mat_pitch << cos_pitch,  0,  sin_pitch,
				0, 1, 0 ,
				-sin_pitch,  0,  cos_pitch;

			/*
			* ������Ȼ��������ת�ǣ���ֻ����������ת����
			* ��������ϵ -> ����������ϵ
			* �ɻ�����ϵ -> �������ϵ
			* ���У�
			* (1) ����������ϵ�ǽ���������ϵ����Z����ʱ����ת90�ȣ�����X��(
			*     ע������ת���X��)��ʱ����ת180�����ã���Ӧ��adjust1��
			*     adjust2;
			* (2) �������ϵ�ǽ��ɻ�����ϵ��Z����ʱ����ת90�����ã���Ӧ��
			*     adjust3��
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
			* ������任��ϳ���ת��������Yaw/Pitch/Roll������ת�������
			* ��˳�����Ҫ���������Ǵӡ���������ϵ��ת��������������ϵ��������
			* Ҫ������任
			*/
			Eigen::Matrix3d mat_rot =
				mat_adjust3 * (mat_yaw*mat_pitch*mat_roll).transpose() * mat_adjust2 * mat_adjust1;

			Eigen::Vector3d c ( 0.0, 0.0, focal);
			c  = mat_rot.inverse() * c +camera_pos;

			// �����ͶӰ��ָ��ͶӰ����
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

	int PhotoFilter::filter_projected_pos(std::string crop_kml, std::string output_folder, double plane_altitude /*= 0.*/)
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
			}


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
				if (image_info_from_exif(image_fullname, img_width, img_height, img_focal))
				{
					std::cout<<"image_info_from_exif failed. "<<image_fullname<<std::endl;
					continue;
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
				Eigen::Vector3d proj_pos;
				if (proj_image_center(photo.pitch, photo.roll, photo.yaw,
					img_width, img_height, img_focal, 
					cam_pos, proj_plane, proj_pos))
				{
					std::cout<<"proj_image_center failed. "<<i_p<<std::endl;
					std::cout<<img_width<<" "<<img_height<<" "<<img_focal<<" "
						<<cam_pos(0)<<" "<<cam_pos(1)<<" "<<cam_pos(2)<<std::endl;
					continue;
				}								


				Point pt = {proj_pos(0), proj_pos(1), proj_pos(2)};	
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

		do 
		{
			Exiv2::Image::AutoPtr exiv2_img = Exiv2::ImageFactory::open(image_filename);
			if(!(exiv2_img.get()))
			{
				std::cout<<"exiv2 open"<< image_filename <<" failed..."<<std::endl;
				break;
			}


			double tmp_w, tmp_h, tmp_f;

			exiv2_img->readMetadata();
			Exiv2::ExifData & exif_data = exiv2_img->exifData();


			Exiv2::ExifData::const_iterator exif_width = exif_data.findKey(Exiv2::ExifKey("Exif.Photo.PixelXDimension"));
			if (exif_width != exif_data.end())
			{
				tmp_w = exif_width->toFloat();			
			} else {
				exif_width = exif_data.findKey(Exiv2::ExifKey("Exif.Image.ImageWidth"));
				if (exif_width == exif_data.end())
				{
					std::cout<<"ImageWidth invalid. "<<image_filename<<std::endl;
					break;
				}
				tmp_w = exif_width->toFloat();
			}

			Exiv2::ExifData::const_iterator exif_height = exif_data.findKey(Exiv2::ExifKey("Exif.Photo.PixelYDimension"));
			if (exif_height != exif_data.end())
			{
				tmp_h = exif_height->toFloat();			
			} else {
				exif_height = exif_data.findKey(Exiv2::ExifKey("Exif.Image.ImageLength"));
				if (exif_height == exif_data.end())
				{
					std::cout<<"ImageHeight invalid. "<<image_filename<<std::endl;
					break;
				}
				tmp_h = exif_height->toFloat();
			}


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
		
		return ret;
	}

}
