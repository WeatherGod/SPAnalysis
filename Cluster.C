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

/*
vector<ClustMember> Cluster::GiveMembers() const
{
	return(myMembers);
}

size_t Cluster::MemberCount() const
{
	return(myMembers.size());
}

bool Cluster::IsEmpty() const
{
	return(myMembers.empty());
}
*/
