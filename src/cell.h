#ifndef CELL_H
# define CELL_H

# include <vector>
# include <string>
# include <map>

# include <boost/shared_ptr.hpp>

# include "typedefs.h"
# include "plancloud.h"

/// \brief Abstract class representing an unitary operation on point clouds.
///
/// Each operation on a point cloud is implemented as a sub-class of this
/// root Cell abstract class.
///
/// The algorithm is implemented in practice in the #compute method.
/// This method is declared here as a pure virtual method and must be
/// reimplemented in each concrete cell.
///
/// \see FileWritingCell
class Cell
{
public:
	/// \name Constructors and desctructor.
	/// \{

	explicit Cell ();
	explicit Cell (std::string name);

	virtual ~Cell ()
	{
	}

	/// \}

	/// \brief Achieve the cell computation.
	///
	/// This method is pure virtual is the root class and
	/// must be reimplemented in each of the concrete classes.
	///
	/// Note that this class is the one that will be called
	/// for each input data so performancies are critical in
	/// this method.
	///
	/// \pre planCloudList must not be null
	/// \param planCloudList Shared pointer on a list of point clouds.
	///                      This pointer should never be null.
	/// \return Point cloud list after being processed, allowing
	///         you to chain calls.
	virtual planCloudsPtr_t compute (planCloudsPtr_t planCloudList) = 0;

	std::map <std::string, double>& parameters()
	{
		return parameters_;
	}

	const std::map <std::string, double>& parameters() const
	{
		return parameters_;
	}

	std::string& cell_name()
	{
		return cell_name_;
	}

private:
	std::map <std::string, double> parameters_;

protected:
	std::string cell_name_;
};

#endif // CELL_H
