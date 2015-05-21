#include "pos_kml_converter.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

#include <tinyxml2.h>

namespace cvt
{


	PosKmlConverter::PosKmlConverter( std::string pos_file ) :
		_pos_file(pos_file)
	{

	}

	PosKmlConverter::~PosKmlConverter()
	{

	}

	int PosKmlConverter::Convert( std::string kml_file )
	{
		int ret = -1;

		do 
		{
			if (_photos.empty() && load_pos())
			{
				std::cout<<"load_pos failed."<<std::endl;
				break;
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
			for (size_t i_p = 0; i_p < num_photos; ++i_p)
			{
				const PhotoPOS & photo = _photos[i_p];

				tinyxml2::XMLElement * placemark_node = doc.NewElement("Placemark");
				if (!placemark_node) goto error0;
				document_node->InsertEndChild(placemark_node);

				tinyxml2::XMLElement * name_node = doc.NewElement("name");
				if (!name_node) goto error0;
				name_node->InsertEndChild(doc.NewText(photo.filename.c_str()));
				placemark_node->InsertEndChild(name_node);

				tinyxml2::XMLElement * point_node = doc.NewElement("Point");
				if (!point_node) goto error0;
				placemark_node->InsertEndChild(point_node);

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

			if (doc.SaveFile(kml_file.c_str())  != tinyxml2::XML_NO_ERROR)
			{
				std::cout<<"xml savefile failed."<<std::endl;
				break;
			}

			ret = 0;
		} while (0);
error0:

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

				sstr >> temp_str; // pitch
				sstr >> temp_str; // roll
				sstr >> temp_str; // yaw

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

}