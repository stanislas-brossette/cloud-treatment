#ifndef FACTORY_H
#define FACTORY_H

#include <map>

#include <boost/bind.hpp>
#include <boost/functional/factory.hpp>
#include <boost/function.hpp>
#include <boost/ref.hpp>
#include <boost/shared_ptr.hpp>


#include "application.h"
#include "cell.h"
#include "displayconvexcloudcell.h"
#include "displaykeypointcloudcell.h"
#include "displaynormalcloudcell.h"
#include "displayxyzcloudcell.h"
#include "dominantplanesegmentationcell.h"
#include "euclidianclustersextractioncell.h"
#include "filecell.h"
#include "filewritingcell.h"
#include "filtercell.h"
#include "hullconvexcell.h"
#include "modelcalibrationcell.h"
#include "normalestimationcell.h"
#include "orientationcell.h"
#include "passthroughfiltercell.h"
#include "planextractioncell.h"
#include "planprojectioncell.h"
#include "regiongrowingsegmentationcell.h"
#include "xyzswitchcell.h"


typedef boost::function < boost::shared_ptr < Cell > () > factory_t;
class Application;

class Factory
{
public:
	Factory(Application& application_ref)
		: application_ref_(application_ref)
	{
		factories_["DisplayConvexCloudCell"] = boost::bind
				(boost::factory < boost::shared_ptr < DisplayConvexCloudCell > > (),
				 boost::ref(application_ref));
		factories_["DisplayKeypointCloudCell"] = boost::bind
				(boost::factory < boost::shared_ptr < DisplayKeypointCloudCell > > (),
				 boost::ref(application_ref));
		factories_["DisplayNormalCloudCell"] = boost::bind
				(boost::factory < boost::shared_ptr < DisplayNormalCloudCell > > (),
				 boost::ref(application_ref));
		factories_["DisplayXYZCloudCell"] = boost::bind
				(boost::factory < boost::shared_ptr < DisplayXYZCloudCell > > (),
				 boost::ref(application_ref));
		factories_["DominantPlaneSegmentationCell"] =
				boost::factory < boost::shared_ptr < DominantPlaneSegmentationCell > > ();
		factories_["EuclidianClustersExtractionCell"] =
				boost::factory < boost::shared_ptr < EuclidianClustersExtractionCell > > ();
		factories_["FileCell"] =
				boost::factory < boost::shared_ptr < FileCell > > ();
		factories_["FileWritingCell"] =
				boost::factory < boost::shared_ptr < FileWritingCell > > ();
		factories_["FilterCell"] =
				boost::factory < boost::shared_ptr < FilterCell > > ();
		factories_["HullConvexCell"] =
				boost::factory < boost::shared_ptr < HullConvexCell > > ();
		factories_["ModelCalibrationCell"] =
				boost::factory < boost::shared_ptr < ModelCalibrationCell > > ();
		factories_["NormalEstimationCell"] =
				boost::factory < boost::shared_ptr < NormalEstimationCell > > ();
		factories_["OrientationCell"] =
				boost::factory < boost::shared_ptr < OrientationCell > > ();
		factories_["PassThroughFilterCell"] =
				boost::factory < boost::shared_ptr < PassThroughFilterCell > > ();
		factories_["PlanExtractionCell"] =
				boost::factory < boost::shared_ptr < PlanExtractionCell > > ();
		factories_["PlanProjectionCell"] =
				boost::factory < boost::shared_ptr < PlanProjectionCell > > ();
		factories_["RegionGrowingSegmentationCell"] =
				boost::factory < boost::shared_ptr < RegionGrowingSegmentationCell > > ();
		factories_["XYZSwitchCell"] =
				boost::factory < boost::shared_ptr < XYZSwitchCell > > ();
	}

	boost::shared_ptr < Cell > create(const std::string& type)
	{
		return factories_[type] ();
	}

private:
	std::map <std::string, factory_t > factories_;
	Application& application_ref_;
};

#endif // FACTORY_H
