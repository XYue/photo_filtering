#include "pos_kml_converter.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

#define NOMINMAX
#include <windows.h>

#include <tinyxml2.h>
#include <proj_api.h>
#include <Eigen/LU>
#include <boost/filesystem.hpp>
#include <exiv2.hpp>

namespace cvt
{


	PosKmlConverter::PosKmlConverter( std::string pos_file ) :
		_pos_file(pos_file)
	{

	}

	PosKmlConverter::PosKmlConverter(std::string path, bool is_pos_file)
	{
		if (is_pos_file)
		{
			_pos_file = path;
			_photo_folder = "";
		} else {
			_photo_folder = path;
			_pos_file = "";
		}
	}

	PosKmlConverter::PosKmlConverter()
	{
		_pos_file = "";
		_photo_folder = "";
	}

	PosKmlConverter::~PosKmlConverter()
	{

	}

	int PosKmlConverter::Convert( std::string kml_file,
		OutputOption option/* = OO_LINE_STRING*/,
		double img_width/* = 0*/, double img_height/* = 0*/, double focal/* = 0.*/,
		double groud_elevation/* = 0. */)
	{
		int ret = -1;

		projPJ longlat_proj = NULL;
		projPJ utm_proj = NULL;

		do 
		{
			if (!_photo_folder.empty())
			{
				if (parse_photo_folder(kml_file))
				{
					std::cout<<"parse_photo_folder failed."<<std::endl;
					break;
				}
			}

			if (_photos.empty())
			{
				if (load_pos())
				{
					std::cout<<"load_pos failed."<<std::endl;
					break;
				}				
			}


			tinyxml2::XMLDocument doc;
			tinyxml2::XMLDeclaration *decl = doc.NewDeclaration("xml version=\"1.0\"");
			doc.InsertEndChild(decl);

			tinyxml2::XMLElement * kml_node = doc.NewElement("kml");
			if (!kml_node) break;
			kml_node->SetAttribute("xmlns", "http://www.opengis.net/kml/2.2");
			kml_node->SetAttribute("xmlns:gx", "http://www.google.com/kml/ext/2.2");
			doc.InsertEndChild(kml_node);

			tinyxml2::XMLElement * document_node = doc.NewElement("Document");
			if (!document_node) goto error0;
			kml_node->InsertEndChild(document_node);


			size_t num_photos = _photos.size();

			// points 
			if ((option & OO_POINT) || (option & OO_PROJ_IMAGE))
			{
				bool output_pos = option & OO_POINT;
				bool output_proj = option & OO_PROJ_IMAGE;

				if (img_width < DBL_EPSILON || img_height < DBL_EPSILON ||
					focal < DBL_EPSILON)
				{
					std::cout<<"invalid width, height or focal: "<<
						img_width<<" "<<img_height<<" "<<focal<<std::endl;
					output_proj = false;
				}

				std::vector<Eigen::Vector3d> proj_img_pts;
				Eigen::Vector4d proj_plane(0, 0, 1, -groud_elevation);


				if (output_proj)
				{					
					int utm_zone = test_utm_zone();
					std::stringstream sstr_utm_zone;	
					sstr_utm_zone << utm_zone;
					std::string str_longlat_proj = "+proj=longlat +datum=WGS84 +ellps=WGS84";
					std::string str_utm_proj = "+proj=utm +datum=WGS84 +ellps=WGS84 +zone=" + sstr_utm_zone.str();
					longlat_proj = pj_init_plus(str_longlat_proj.c_str());
					utm_proj = pj_init_plus(str_utm_proj.c_str());
					if(!longlat_proj || !utm_proj)  
					{
						std::cout<<"Initial proj parameters failed."<<std::endl;
						goto error0;
					}

					proj_img_pts.resize(5);
					proj_img_pts[0] = Eigen::Vector3d(0, 0, focal);
					proj_img_pts[1] = Eigen::Vector3d(-img_width/2., -img_height/2., focal);
					proj_img_pts[2] = Eigen::Vector3d(-img_width/2., img_height/2., focal);
					proj_img_pts[3] = Eigen::Vector3d(img_width/2., img_height/2., focal);
					proj_img_pts[4] = Eigen::Vector3d(img_width/2., -img_height/2., focal);
				}
				

				tinyxml2::XMLElement * xml_folder = doc.NewElement("Folder");
				if (!xml_folder) goto error0;
				document_node->InsertEndChild(xml_folder);

				tinyxml2::XMLElement * folder_name_node = doc.NewElement("name");
				if (!folder_name_node) goto error0;
				folder_name_node->InsertEndChild(doc.NewText("CameraPos"));
				xml_folder->InsertEndChild(folder_name_node);				

				for (size_t i_p = 0; i_p < num_photos; ++i_p)
				{
					const PhotoPOS & photo = _photos[i_p];

					tinyxml2::XMLElement * placemark_node = doc.NewElement("Placemark");
					if (!placemark_node) goto error0;
					xml_folder->InsertEndChild(placemark_node);

					tinyxml2::XMLElement * name_node = doc.NewElement("name");
					if (!name_node) goto error0;
					name_node->InsertEndChild(doc.NewText(photo.filename.c_str()));
					placemark_node->InsertEndChild(name_node);

					tinyxml2::XMLElement * xml_multi_geometry = doc.NewElement("MultiGeometry");
					if (!xml_multi_geometry) goto error0;
					placemark_node->InsertEndChild(xml_multi_geometry);

					if (output_pos)
					{
						tinyxml2::XMLElement * point_node = doc.NewElement("Point");
						if (!point_node) goto error0;
						xml_multi_geometry->InsertEndChild(point_node);

						tinyxml2::XMLElement * altitude_mode_node = doc.NewElement("altitudeMode");
						if (!altitude_mode_node) goto error0;
						altitude_mode_node->InsertEndChild(doc.NewText("absolute"));
						point_node->InsertEndChild(altitude_mode_node);

						tinyxml2::XMLElement * coordinates_node = doc.NewElement("coordinates");
						if (!coordinates_node) goto error0;
						point_node->InsertEndChild(coordinates_node);

						std::stringstream sstr;
						sstr<< std::setprecision(15)<< photo.longitude<<", "<<photo.latitude<<", "<<photo.altitude;
						coordinates_node->InsertEndChild(doc.NewText(sstr.str().c_str()));
					}

					if (output_proj)
					{
						Eigen::Vector3d cam_pos(photo.longitude, photo.latitude, photo. altitude);
						double x = cam_pos(0) * DEG_TO_RAD;
						double y = cam_pos(1) * DEG_TO_RAD;
						if(pj_transform(longlat_proj, utm_proj, 1, 0, 
							&x, &y, NULL))
						{
							std::cout<<"pj_transform failed at pos conversion. "
								<<cam_pos(0)<<" "<<cam_pos(1)<<" "<<cam_pos(2)<<std::endl;
							continue;
						}
						cam_pos(0) = x;
						cam_pos(1) = y;

						std::vector<Eigen::Vector3d> proj_pts;
						if (proj_image_points(photo.pitch, photo.roll, photo.yaw,
							img_width, img_height, focal, 
							cam_pos, proj_plane, proj_img_pts,proj_pts))
						{
							std::cout<<"proj_image_points failed. "<<i_p<<std::endl;
							std::cout<<cam_pos(0)<<" "<<cam_pos(1)<<" "<<cam_pos(2)<<std::endl;
							continue;
						}

						size_t num_pts = proj_pts.size();
						for (size_t i_pt = 0; i_pt < num_pts; ++i_pt)
						{
							double x = proj_pts[i_pt](0);
							double y = proj_pts[i_pt](1);
							if(pj_transform(utm_proj, longlat_proj, 1, 0, 
								&x, &y, NULL))
							{
								std::cout<<"pj_transform failed at pos conversion. "
									<<cam_pos(0)<<" "<<cam_pos(1)<<" "<<cam_pos(2)<<std::endl;
								continue;
							}
							proj_pts[i_pt](0) = x * RAD_TO_DEG;
							proj_pts[i_pt](1) = y * RAD_TO_DEG;
						}

						tinyxml2::XMLElement * xml_line_string_br = doc.NewElement("LineString");
						if (xml_line_string_br)
						{
							xml_multi_geometry->InsertEndChild(xml_line_string_br);

							tinyxml2::XMLElement * xml_ls_tessellate = doc.NewElement("tessellate");
							if (!xml_ls_tessellate) goto error0;
							xml_ls_tessellate->InsertEndChild(doc.NewText("1"));
							xml_line_string_br->InsertEndChild(xml_ls_tessellate);

							tinyxml2::XMLElement * ls_coordinates_node = doc.NewElement("coordinates");
							if (!ls_coordinates_node) goto error0;
							xml_line_string_br->InsertEndChild(ls_coordinates_node);

							std::stringstream sstr;
							sstr<< std::setprecision(15);
							for (size_t i_pt = 0; i_pt < num_pts; ++i_pt)
							{
								sstr <<  proj_pts[i_pt](0) <<","
									<<proj_pts[i_pt](1) <<","
									<<proj_pts[i_pt](2)<<" ";
							}
							sstr <<  proj_pts[1](0) <<","
								<<proj_pts[1](1) <<","
								<<proj_pts[1](2)<<" ";
							ls_coordinates_node->InsertEndChild(doc.NewText(sstr.str().c_str()));
						} else goto error0;
					}					
				}
			}


			// linestring
			if (option & OO_LINE_STRING)
			{
				tinyxml2::XMLElement * xml_folder = doc.NewElement("Folder");
				if (!xml_folder) goto error0;
				document_node->InsertEndChild(xml_folder);

				tinyxml2::XMLElement * folder_name_node = doc.NewElement("name");
				if (!folder_name_node) goto error0;
				folder_name_node->InsertEndChild(doc.NewText("CameraLineString"));
				xml_folder->InsertEndChild(folder_name_node);

				tinyxml2::XMLElement * ls_placemark_node = doc.NewElement("Placemark");
				if (!ls_placemark_node) goto error0;
				xml_folder->InsertEndChild(ls_placemark_node);

				tinyxml2::XMLElement * ls_name_node = doc.NewElement("name");
				if (!ls_name_node) goto error0;
				ls_name_node->InsertEndChild(doc.NewText("linestring"));
				ls_placemark_node->InsertEndChild(ls_name_node);

				tinyxml2::XMLElement * xml_line_string = doc.NewElement("LineString");
				if (!xml_line_string) goto error0;
				ls_placemark_node->InsertEndChild(xml_line_string);

				tinyxml2::XMLElement * xml_ls_tessellate = doc.NewElement("tessellate");
				if (!xml_ls_tessellate) goto error0;
				xml_ls_tessellate->InsertEndChild(doc.NewText("1"));
				xml_line_string->InsertEndChild(xml_ls_tessellate);

				tinyxml2::XMLElement * ls_coordinates_node = doc.NewElement("coordinates");
				if (!ls_coordinates_node) goto error0;
				xml_line_string->InsertEndChild(ls_coordinates_node);

				std::string line_string_coor = "";
				for (size_t i_p = 0; i_p < num_photos; ++i_p)
				{
					const PhotoPOS & photo = _photos[i_p];				

					std::stringstream sstr;
					sstr<< std::setprecision(15)<< photo.longitude<<","<<photo.latitude<<","<<photo.altitude;
					line_string_coor += sstr.str() + " ";
				}
				ls_coordinates_node->InsertEndChild(doc.NewText(line_string_coor.c_str()));
			}
			

			if (doc.SaveFile(kml_file.c_str())  != tinyxml2::XML_NO_ERROR)
			{
				std::cout<<"xml savefile failed."<<std::endl;
				break;
			}

			ret = 0;
		} while (0);
error0:

		if(utm_proj)
		{
			pj_free(utm_proj);
			utm_proj = NULL;
		}
		if(longlat_proj)
		{
			pj_free(longlat_proj);
			longlat_proj = NULL;
		}

		return ret;
	}

	int PosKmlConverter::load_pos()
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
				sstr >> temp_str; // day
				sstr >> temp_str; // time

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

		return ret;
	}

	int PosKmlConverter::proj_image_points(
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

	int PosKmlConverter::test_utm_zone()
	{
		double west = std::numeric_limits<double>::max();
		size_t num_photos = _photos.size();
		for (int i_c = 0; i_c < num_photos; ++i_c)
			west = std::min(west,
			_photos[i_c].longitude > std::numeric_limits<double>::epsilon() ?
			_photos[i_c].longitude : west);

		int utm_zone = static_cast<int>(west / 6.) + 31;
		if (utm_zone <0 || utm_zone > 60) 
		{
			std::cout<<"Invalid UTM zone: "<<utm_zone<<std::endl;
			return -1;
		}

		return utm_zone;
	}

	double PosKmlConverter::cos_deg(const double x)
	{
		int numPI = (int)(x/180.0);
		double rad = (x-180.0*numPI)*M_PI/180.0;
		return (numPI/2*2 == numPI ? cos(rad) : -cos(rad));
	}

	double PosKmlConverter::sin_deg(const double x)
	{
		int numPI = (int)(x/180.0);
		double rad = (x-180.0*numPI)*M_PI/180.0;
		return (numPI/2*2 == numPI ? sin(rad) : -sin(rad));
	}

	int PosKmlConverter::parse_photo_folder( const std::string & kml_filename )
{
		int ret = -1;

		do 
		{
			if (!boost::filesystem::is_directory(_photo_folder))
			{
				std::cout<< _photo_folder<<" folder not exists."<<std::endl;
				break; 
			}


			std::vector<PhotoPOS> pos;
			std::vector<std::string> img_files;

			// find all images in output folder
			HANDLE hFind = INVALID_HANDLE_VALUE;
			WIN32_FIND_DATA ffd;
			std::string spec;
			spec = _photo_folder + "\\*";
			hFind = FindFirstFile(spec.c_str(), &ffd);
			if (hFind == INVALID_HANDLE_VALUE) break;

			do 
			{
				if (strcmp(ffd.cFileName, ".") != 0 && 
					strcmp(ffd.cFileName, "..") != 0) 
				{
					if (!(ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) 
					{
						std::string name = ffd.cFileName;
						std::string ext = name.substr(name.find_last_of('.')+1);
						if (img_ext_satisfied(ext))
						{
							std::string img_filenmae = _photo_folder + "\\" + name;

							PhotoPOS p;
							if (read_pos_from_image_exif(img_filenmae, p))
							{
								std::cout<<"failed to read pos from "<<img_filenmae<<std::endl;
								FindClose(hFind);
								goto error0;
							}

							pos.push_back(p);
							img_files.push_back(img_filenmae)							;
						}
					}
				}
			} while (FindNextFile(hFind, &ffd) != 0);

			if (GetLastError() != ERROR_NO_MORE_FILES) 
			{
				FindClose(hFind);
				goto error0;
			}

			FindClose(hFind);
			hFind = INVALID_HANDLE_VALUE;


			// write down pos
			std::string tmp_pos_filename = kml_filename + ".posfile";
			if (write_pos_file(tmp_pos_filename, pos))
			{
				std::cout<<"failed to write pos file. "<< tmp_pos_filename  <<std::endl;
				break;
			}


			_pos_file = tmp_pos_filename;
			_photos.swap(pos);


			ret = 0;
		} while (0);
error0:

		return ret;
	}

	bool PosKmlConverter::img_ext_satisfied( const std::string & ext )
	{
		std::string suffix = ext;
		std::transform(suffix.begin(), suffix.end(), suffix.begin(), ::tolower);
		if (suffix.compare("jpeg") && suffix.compare("jpg") && 
			suffix.compare("tif") && suffix.compare("tiff") )
		{
			return false;
		}

		return true;
	}

	int PosKmlConverter::read_pos_from_image_exif( 
		const std::string & img_file, 
		PhotoPOS & pos )
	{
		int rtn = -1;

		do 
		{
			if (img_file.empty())
				break;

			Exiv2::Image::AutoPtr image = Exiv2::ImageFactory::open(img_file);
			if(!(image.get()))
			{
				std::cout<<"exif open image failed..."<<std::endl;
				break;
			}

			PhotoPOS tmp_pos;

			image->readMetadata();
			Exiv2::ExifData & exifData = image->exifData();

			Exiv2::ExifData::const_iterator gps_longitude =
				exifData.findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSLongitude"));
			Exiv2::ExifData::const_iterator gps_latitude =
				exifData.findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSLatitude"));
			Exiv2::ExifData::const_iterator gps_altitude =
				exifData.findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSAltitude"));
			Exiv2::ExifData::const_iterator comment = 
				exifData.findKey(Exiv2::ExifKey("Exif.Photo.UserComment"));

			if(gps_longitude != exifData.end() && 
				gps_latitude != exifData.end() &&
				gps_altitude != exifData.end())
			{
				double value0, value1, value2;

				value0 = gps_longitude->value().toFloat(0);
				value1 = gps_longitude->value().toFloat(1);
				value2 = gps_longitude->value().toFloat(2);
				tmp_pos.longitude = value0+(value1+value2/60.)/60.;

				value0 = gps_latitude->value().toFloat(0);
				value1 = gps_latitude->value().toFloat(1);
				value2 = gps_latitude->value().toFloat(2);
				tmp_pos.latitude = value0+(value1+value2/60.)/60.;

				tmp_pos.altitude = gps_altitude->toFloat();
			} else {
				tmp_pos.longitude = 0.;
				tmp_pos.latitude = 0.;
				tmp_pos.altitude = 0.;
			}

			if (comment != exifData.end())
			{
				std::string info =comment->value().toString();
				std::transform(info.begin(), info.end(), info.begin(), ::tolower);

				std::stringstream sstr;
				sstr << info;

				std::string buf;
				sstr >> buf;

				while (!buf.empty())
				{
					if (!buf.compare("pitch"))
					{
						buf.clear();
						sstr >> buf;
						if (buf.empty()) goto error0;

						std::stringstream value;
						value << buf;
						value >> tmp_pos.pitch;
					} else if (!buf.compare("roll")) {
						buf.clear();
						sstr >> buf;
						if (buf.empty()) goto error0;

						std::stringstream value;
						value << buf;
						value >> tmp_pos.roll;
					} else if (!buf.compare("yaw")) {
						buf.clear();
						sstr >> buf;
						if (buf.empty()) goto error0;

						std::stringstream value;
						value << buf;
						value >> tmp_pos.yaw;
					}

					buf.clear();
					sstr >> buf;
				}
			} else {
				tmp_pos.pitch = 0.;
				tmp_pos.roll = 0.;
				tmp_pos.yaw = 0.;
			}

			tmp_pos.filename = img_file;
			pos = tmp_pos;

			rtn = 0;
		} while (0);
error0:

		return rtn;
	}

	int PosKmlConverter::write_pos_file( 
		const std::string & pos_filename, 
		const std::vector<PhotoPOS> & pos )
	{
		int rtn = -1;

		do 
		{
			std::ofstream pos_file(pos_filename, std::ios::out);
			if (pos_file.is_open())
			{
				int num_img = pos.size();
				for(int i_img = 0; i_img < num_img; ++i_img)
				{
					const PhotoPOS & p = pos[i_img];

					pos_file << i_img << " day time ";
					pos_file << std::fixed << std::setprecision(15)
						<< p.longitude << " "
						<< p.latitude << " "
						<< p.altitude << " "
						<< p.pitch << " "
						<< p.roll << " "
						<< p.yaw << " "
						<< p.filename << std::endl;
				}
				pos_file.close(); 
			}

			rtn = 0;
		} while (0);
error0:

		return rtn;
	}

	int PosKmlConverter::SetInputPath( const std::string & input_path )
	{
		int ret = -1;

		do 
		{
			if (boost::filesystem::is_directory(input_path))
			{
				_photo_folder = input_path;
				_pos_file = "";
				_photos.swap(std::vector<PhotoPOS>());
			} else if (boost::filesystem::is_regular_file(input_path)) {
				_pos_file = input_path;
				_photo_folder = "";
				_photos.swap(std::vector<PhotoPOS>());
			} else {
				std::cout<<"invalid input path. "<<input_path<<std::endl;
				break;
			}

			ret = 0;
		} while (0);

		return ret;
	}

}