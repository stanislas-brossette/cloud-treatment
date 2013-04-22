#ifndef FACTORY_H
#define FACTORY_H

#include <map>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/functional/factory.hpp>

#include "cell.h"
#include "filecell.h"
#include "filewritingcell.h"
#include "filtercell.h"
#include "hullconvexcell.h"
#include "orientationcell.h"
#include "passthroughfiltercell.h"
#include "planextractioncell.h"
#include "planprojectioncell.h"
#include "regiongrowingsegmentationcell.h"
#include "xyzswitchcell.h"

typedef boost::function < boost::shared_ptr < Cell > () > factory_t;

class Factory
{
public:
	Factory()
	{
		factories_["FileCell"] = boost::factory < boost::shared_ptr < FileCell > > ();
		factories_["FileWritingCell"] = boost::factory < boost::shared_ptr < FileWritingCell > > ();
		factories_["FilterCell"] = boost::factory < boost::shared_ptr < FilterCell > > ();
		factories_["HullConvexCell"] = boost::factory < boost::shared_ptr < HullConvexCell > > ();
		factories_["OrientationCell"] = boost::factory < boost::shared_ptr < OrientationCell > > ();
		factories_["PassThroughFilterCell"] = boost::factory < boost::shared_ptr < PassThroughFilterCell > > ();
		factories_["PlanExtractionCell"] = boost::factory < boost::shared_ptr < PlanExtractionCell > > ();
		factories_["PlanProjectionCell"] = boost::factory < boost::shared_ptr < PlanProjectionCell > > ();
		factories_["RegionGrowingSegmentationCell"] = boost::factory < boost::shared_ptr < RegionGrowingSegmentationCell > > ();
		factories_["XYZSwitchCell"] = boost::factory < boost::shared_ptr < XYZSwitchCell > > ();
	}

	boost::shared_ptr < Cell > create(const std::string& type)
	{
		return factories_[type] ();
	}

private:
	std::map <std::string, factory_t > factories_;
};

#endif // FACTORY_H
