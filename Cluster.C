using namespace std;

#include "Cluster.h"		// for struct ClustMember

#include <vector>
#include <cctype>		// for size_t


Cluster::Cluster()
{
}


Cluster::Cluster(const vector<size_t> &XLocs, const vector<size_t> &YLocs, const vector<float> &dataVals)
{
	resize(XLocs.size());

	if (XLocs.size() != YLocs.size() || XLocs.size() != dataVals.size())
	{
		clear();
	}
	else
	{
		for (size_t index = 0; index < size(); index++)
		{
			(*this)[index] = ClustMember(XLocs[index], YLocs[index], dataVals[index]);
		}
	}
}

void Cluster::AddMember(const ClustMember &newMember)
{
	push_back(newMember);
}

void Cluster::AddMember(const size_t &newX, const size_t &newY, const float &newVal)
{
	AddMember( ClustMember(newX, newY, newVal) );
}

vector<ClustMember>::const_iterator
Cluster::MaxMember() const
{
	if (!empty())
	{
		vector<ClustMember>::const_iterator maxElement = begin();

		for (vector<ClustMember>::const_iterator aMember = (begin() + 1);
		     aMember != end();
		     aMember++)
		{
			if (aMember->memberVal > maxElement->memberVal)
			{
				maxElement = aMember;
			}
		}

		return(maxElement);
	}
	else
	{
		return(end());
	}
}
