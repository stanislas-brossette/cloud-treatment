#ifndef ORGANIZEDSEGMENTATIONCELL_H
#define ORGANIZEDSEGMENTATIONCELL_H

#include "cell.h"
#include "typedefs.h"

class OrganizedSegmentationCell : public Cell
{
public:
	OrganizedSegmentationCell();

	virtual ~OrganizedSegmentationCell ()
	{
	}

	planCloudsPtr_t compute(planCloudsPtr_t);

private:

};

#endif // ORGANIZEDSEGMENTATIONCELL_H
