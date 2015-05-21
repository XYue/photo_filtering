#pragma once

#include <string>
#include <vector>

namespace cvt
{
	class PosKmlConverter
	{
	public:
		typedef struct PhotoPOS 
		{
			double longitude;
			double latitude;
			double altitude;

			std::string filename;
		} PhotoPOS;

	public:
		PosKmlConverter(std::string pos_file);
		~PosKmlConverter();

		int Convert(std::string kml_file);

	protected:
		int load_pos();

	private:
		std::string _pos_file;
		std::vector<PhotoPOS> _photos;
	};
}