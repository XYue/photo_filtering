#include "exif_toolkit/exif_toolkit.hpp"

#include <windows.h>

#include <boost/filesystem.hpp>
#include <exiv2.hpp>

namespace toolkit
{


	int ExifToolkit::CleanPosExif( const std::string & input_path )
	{
		int ret = -1;

		do 
		{
			std::vector<std::string> img_files;

			if (boost::filesystem::is_directory(input_path))
			{
				if (parse_dir(input_path, img_files))
				{
					std::cout<<"parse_dir failed. "<<input_path<<std::endl;
					break;
				}
			} else if (boost::filesystem::is_regular_file(input_path)) {
				if (parse_config_file(input_path, img_files))
				{
					std::cout<<"parse_config_file failed. "<<input_path<<std::endl;
					break;
				}
			} else {
				std::cout<<"invalid input. "<<input_path<<std::endl;
				break;
			}

			Exiv2::LogMsg::setLevel(Exiv2::LogMsg::mute);
			size_t num_imgs = img_files.size();
			for (int i_img = 0; i_img < num_imgs; ++i_img)
			{
				Exiv2::Image::AutoPtr exiv2_dst_img = Exiv2::ImageFactory::open(img_files[i_img]);
				if(!(exiv2_dst_img.get()))
				{
					std::cout<<"exiv2 open"<< img_files[i_img] <<" failed..."<<std::endl;
					goto error0;;
				}
				exiv2_dst_img->readMetadata();
				Exiv2::ExifData & exif_data = exiv2_dst_img->exifData();


				// clear pos
				Exiv2::ExifData::iterator itr_gps;
				if ((itr_gps = exif_data.findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSLongitude"))) != exif_data.end())
				{
					exif_data.erase(itr_gps);
				}
				if ((itr_gps = exif_data.findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSLatitude"))) != exif_data.end())
				{
					exif_data.erase(itr_gps);
				}
				if ((itr_gps = exif_data.findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSAltitude"))) != exif_data.end())
				{
					exif_data.erase(itr_gps);
				}

				// clear user comment
				exif_data["Exif.Photo.UserComment"] = "";
// 				if ((itr_gps = exif_data.findKey(Exiv2::ExifKey("Exif.Photo.UserComment"))) != exif_data.end())
// 				{
// 					exif_data.erase(itr_gps);
// 				}


				exiv2_dst_img->writeMetadata();
			}

			ret = 0;
		} while (0);
error0:

		return ret;
	}

	int ExifToolkit::UpdatePosExif( const std::string & config_file )
	{
		int ret = -1;

		do 
		{
			std::vector<PhotoPos> poses;


			if (boost::filesystem::is_regular_file(config_file)) {
				if (parse_config_file(config_file, poses))
				{
					std::cout<<"parse_config_file failed. "<<config_file<<std::endl;
					break;
				}
			} else {
				std::cout<<"invalid input. "<<config_file<<std::endl;
				break;
			}


			size_t num_files = poses.size();
			for (int i_p = 0; i_p < num_files; ++i_p)
			{
				const PhotoPos & pos = poses[i_p];

				Exiv2::Image::AutoPtr exiv2_dst_img = Exiv2::ImageFactory::open(pos.filename);
				if(!(exiv2_dst_img.get()))
				{
					std::cout<<"exiv2 open"<< pos.filename <<" failed..."<<std::endl;
					goto error0;;
				}
				exiv2_dst_img->readMetadata();
				Exiv2::ExifData & exif_data = exiv2_dst_img->exifData();				


				// update pos
				exif_data["Exif.GPSInfo.GPSLongitude"] = double_to_gps_rational_string(pos.longitude);
				exif_data["Exif.GPSInfo.GPSLatitude"] = double_to_gps_rational_string(pos.latitude);
				exif_data["Exif.GPSInfo.GPSAltitude"] = 
					Exiv2::floatToRationalCast(static_cast<float>(pos.altitude));

				std::stringstream sstr;
				sstr << std::fixed
					<< std::setprecision(15)
					<< pos.pitch << " "
					<< pos.roll << " "
					<< pos.yaw;
				std::string orientation, buf;
				sstr >> buf;
				orientation = "pitch " + buf;
				sstr >> buf;
				orientation += " roll " + buf;
				sstr >> buf;
				orientation += " yaw " + buf;
				exif_data["Exif.Photo.UserComment"] = orientation;


				exiv2_dst_img->writeMetadata();
			}


			ret = 0;
		} while (0);
error0:

		return ret;
	}

	int ExifToolkit::parse_config_file(
		const std::string & config_file,
		std::vector<PhotoPos> & poses )
	{
		int ret = -1;

		do 
		{
			std::vector<PhotoPos> tmp_poses;


			std::ifstream file(config_file);
			if (!file.good()) break;

			std::string line;
			std::stringstream sstr;
			std::getline(file, line);
			while (file.good())
			{
				PhotoPos photo;

				sstr.clear(); sstr.str("");
				sstr << line;

				sstr >> photo.filename;
				if (boost::filesystem::is_regular_file(photo.filename))
				{
					sstr >> photo.longitude;
					sstr >> photo.latitude;
					sstr >> photo.altitude;

					sstr >> photo.pitch; // pitch
					sstr >> photo.roll; // roll
					sstr >> photo.yaw; // yaw

					tmp_poses.push_back(photo);
				}

				line.clear();
				std::getline(file, line);
			}


			poses.swap(tmp_poses);


			ret = 0;
		} while (0);

		return ret;
	}

	int ExifToolkit::parse_config_file( 
		const std::string & config_file,
		std::vector<std::string> & filenames )
	{
		int ret = -1;

		do 
		{
			std::vector<std::string> tmp_names;


			std::ifstream file(config_file);
			if (!file.good()) break;

			std::string line;
			std::stringstream sstr;
			std::getline(file, line);
			while (file.good())
			{
				std::string name;

				sstr.clear(); sstr.str("");
				sstr << line;

				sstr >> name;
				if (boost::filesystem::is_regular_file(name))
				{
					tmp_names.push_back(name);
				}

				line.clear();
				std::getline(file, line);
			}


			filenames.swap(tmp_names);


			ret = 0;
		} while (0);

		return ret;
	}

	inline bool ExifToolkit::img_ext_satisfied( const std::string & ext )
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

	int ExifToolkit::parse_dir( 
		const std::string & dir,
		std::vector<std::string> & filenames )
	{
		int ret = -1;

		do 
		{
			if (!boost::filesystem::is_directory(dir))
			{
				std::cout<< dir<<" folder not exists."<<std::endl;
				break; 
			}


			std::vector<std::string> tmp_names;


			// find all images in output folder
			HANDLE hFind = INVALID_HANDLE_VALUE;
			WIN32_FIND_DATA ffd;
			std::string spec;
			spec = dir + "\\*";
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
							tmp_names.push_back(dir + "\\" + name);
						}
					}
				}
			} while (FindNextFile(hFind, &ffd) != 0);

			if (GetLastError() != ERROR_NO_MORE_FILES) 
			{
				FindClose(hFind);
				break;
			}

			FindClose(hFind);
			hFind = INVALID_HANDLE_VALUE;


			filenames.swap(tmp_names);


			ret = 0;
		} while (0);

		return ret;
	}

	std::string ExifToolkit::double_to_gps_rational_string( double value )
	{
		std::string str;

		double rest = value;
		int temp = static_cast<int>(rest);
		Exiv2::Rational degrees, minutes, seconds;
		degrees = Exiv2::floatToRationalCast(temp);
		rest = (rest - temp)*60.;
		temp = static_cast<int>(rest);
		minutes = Exiv2::floatToRationalCast(temp);
		rest = (rest - temp)*60.;
		seconds = Exiv2::floatToRationalCast(rest);

		str = Exiv2::toString(degrees) + " " +
			Exiv2::toString(minutes) + " " +
			Exiv2::toString(seconds);

		return str;
	}

}