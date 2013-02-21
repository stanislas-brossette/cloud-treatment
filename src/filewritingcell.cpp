#include "boost/make_shared.hpp"
#include "filewritingcell.h"

template <typename T> std::string tostr(const T& t) { std::ostringstream os; os<<t; return os.str(); }

FileWritingCell::FileWritingCell()
{
}

planCloudsPtr_t FileWritingCell::compute(planCloudsPtr_t planCloudListPtr)
{
	return planCloudListPtr;
}

void FileWritingCell::write_files(std::string folderPath, planCloudsPtr_t planCloudListPtr, std::string bodyName)
{
	folderPath += "";
	std::string fileContent = "{\n\t\"Body\":\"";
	fileContent += bodyName + "\",\n\t\"Surfaces\":\n\t[\n";

	for (pointCloudPoints_t::size_type i=0; i<planCloudListPtr->size(); ++i)
	{
		planCloudListPtr->at(i).find_origin();
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
	//	std::cout << std::endl << std::endl << fileContent << std::endl << std::endl;
}

std::string FileWritingCell::write_surf_string(planCloudPtr_t planCloudPtr, std::string name)
{
	std::string surfString = "";

	surfString += "\t\t{\n\t\t\t\"Name\":\"" + name + "\",\n";
	surfString += "\t\t\t\"Origin\":[" + tostr(planCloudPtr->origin()->x()) + ", "
			+ tostr(planCloudPtr->origin()->y()) + ", "
			+ tostr(planCloudPtr->origin()->z()) + "],\n";
	surfString += "\t\t\t\"Frame\":[[" + tostr(planCloudPtr->frame()->at(0).x())
			+ ", " + tostr(planCloudPtr->frame()->at(1).x()) + ", "
			+ tostr(planCloudPtr->frame()->at(2).x()) + "],\n";
	surfString += "\t\t\t\t[" + tostr(planCloudPtr->frame()->at(0).y())
			+ ", " + tostr(planCloudPtr->frame()->at(1).y()) + ", "
			+ tostr(planCloudPtr->frame()->at(2).y()) + "],\n";
	surfString += "\t\t\t\t[" + tostr(planCloudPtr->frame()->at(0).z()) + ", "
			+ tostr(planCloudPtr->frame()->at(1).z()) + ", "
			+ tostr(planCloudPtr->frame()->at(2).z()) + "]],\n";

	surfString += "\t\t\t\"Points\":\n\t\t\t[\n";


	Eigen::Vector3d projectedPoint;
	for(pointCloudPoints_t::size_type i = 0; i<planCloudPtr->cloud()->points.size(); ++i)
	{
		projectedPoint = planCloudPtr->project_point(i);
		surfString += "\t\t\t\t{\"x\":" + tostr(projectedPoint.x())
				+ ", \"y\":" + tostr(projectedPoint.y())/*
						+ ", \"z\":" + tostr(projectedPoint.z())*/ + "}";
		if ( i != planCloudPtr->cloud()->points.size() - 1)
			surfString += ",\n";
		else
			surfString += "\n\t\t\t],\n\t\t\t\"Material\":\"plastic\"\n\t\t}";
	}

	return surfString;
}


