# include <iomanip>
# include <sstream>

# include <boost/make_shared.hpp>
# include <pcl/io/pcd_io.h>

# include "filewritingcell.h"

template <typename T> std::string tostr(const T& t) { std::ostringstream os; os<<t; return os.str(); }

FileWritingCell::FileWritingCell():
	Cell("FileWritingCell")
{
}

planCloudsPtr_t FileWritingCell::compute(planCloudsPtr_t planCloudListPtr)
{
	return planCloudListPtr;
}

planCloudsPtr_t FileWritingCell::write_files(std::string folderPath, planCloudsPtr_t planCloudListPtr, std::string bodyName)
{
	folderPath += "";
	std::string fileContent = "{\n\t\"Body\":\"";
	fileContent += bodyName + "\",\n\t\"Surfaces\":\n\t[\n";

	for (pointCloudSize_t i=0; i<planCloudListPtr->size(); ++i)
	{
		if (!planCloudListPtr->at(i).origin())
			planCloudListPtr->at(i).find_origin();
		if (!planCloudListPtr->at(i).frame())
			planCloudListPtr->at(i).find_frame();

		std::string surfName = "";
		surfName += "Surf" + tostr(i);
		planCloudPtr_t planCloudPtr = boost::make_shared<PlanCloud>(planCloudListPtr->at(i));
		fileContent += write_surf_string(planCloudPtr, surfName);
		if(i!=planCloudListPtr->size()-1)
		{
			fileContent += ",\n";
		}
		else
		{
			fileContent += "\n\t]\n}";
		}
	}
	////Output it to a file
	std::string filename0 = folderPath + bodyName + ".surf";
	const char * filename = filename0.c_str();
	std::ofstream out( filename );
	if( !out )
	{
		throw std::runtime_error("Couldn't open file.");
	}
	out << fileContent;
	out.close();
	return planCloudListPtr;
	//	std::cout << std::endl << std::endl << fileContent << std::endl << std::endl;
}

std::string FileWritingCell::write_surf_string(planCloudPtr_t planCloudPtr, std::string name)
{
	std::stringstream surfStringStream;
	surfStringStream << std::setprecision(30);
	surfStringStream << "\t\t{\n\t\t\t\"Name\":\"" << name << "\","<<std::endl;
	surfStringStream << "\t\t\t\"Origin\":[" << planCloudPtr->origin()->x() << ", "
						<< planCloudPtr->origin()->y() << ", "
						<< planCloudPtr->origin()->z() << "],"<<std::endl;
	surfStringStream << "\t\t\t\"Frame\":[[" << planCloudPtr->T().x()
						<< ", " << planCloudPtr->B().x() << ", "
						<< planCloudPtr->N().x() << "],"<<std::endl;
	surfStringStream << "\t\t\t\t[" << planCloudPtr->T().y()
			<< ", " << planCloudPtr->B().y() << ", "
			<< planCloudPtr->N().y() << "],"<<std::endl;
	surfStringStream << "\t\t\t\t[" << planCloudPtr->T().z() << ", "
			<< planCloudPtr->B().z() << ", "
			<< planCloudPtr->N().z() << "]],"<<std::endl;

	surfStringStream << "\t\t\t\"Points\":\n\t\t\t["<<std::endl;

	for(pointCloudPoints_t::size_type i = 0; i<planCloudPtr->cloud()->points.size(); ++i)
	{
		surfStringStream << "\t\t\t\t{\"x\":" << planCloudPtr->cloud()->points[i].x
				<< ", \"y\":" << planCloudPtr->cloud()->points[i].y << "}";
		if ( i != planCloudPtr->cloud()->points.size() - 1)
			surfStringStream << ","<<std::endl;
		else
			surfStringStream << "\n\t\t\t],"<<std::endl<<"\t\t\t\"Material\":\"plastic\""<<std::endl<<"\t\t}";
	}

	return surfStringStream.str();
}

void FileWritingCell::write_cloud_files(std::string folderPath, std::string filename, planCloudsPtr_t planCloudListPtr)
{
	std::cout<<"planCloudListPtr->size() = "<<planCloudListPtr->size()<<std::endl;
	for (pointCloudPoints_t::size_type i=0; i<planCloudListPtr->size(); ++i)
	{
		std::string completePath = folderPath;
		completePath += filename;
		completePath += tostr(i);
		completePath += ".pcd";
		pcl::io::savePCDFileASCII (completePath, *(planCloudListPtr->at(i).cloud()));
		std::cout<<"pcd file made of "<< planCloudListPtr->at(i).cloud()->size()
				<< "data points saved as " << completePath << std::endl;
	}
}
