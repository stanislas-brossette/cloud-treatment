#include "cell.h"

Cell::Cell()
{
	parameters()["name"] = "FileCell";
}

Cell::Cell(std::string name):
	cell_name_(name)
{
}
