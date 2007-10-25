using namespace std;

#include "Cluster.h"		// for struct ClustMember

#include <vector>
#include <cctype>		// for size_t


Cluster::Cluster()
	:	myMembers(0)
{
}

Cluster::Cluster(const Cluster &clustCopy)
	:	myMembers(clustCopy.myMembers)
{
}

Cluster::Cluster(const vector<size_t> &XLocs, const vector<size_t> &YLocs, const vector<float> &dataVals)
	:	myMembers(XLocs.size())
{
	if (XLocs.size() != YLocs.size() || XLocs.size() != dataVals.size())
	{
		myMembers.clear();
	}
	else
	{
		for (size_t index = 0; index < myMembers.size(); index++)
		{
			myMembers[index] = ClustMember(XLocs[index], YLocs[index], dataVals[index]);
		}
	}
}

void Cluster::AddMember(const ClustMember &newMember)
{
	myMembers.push_back(newMember);
}

void Cluster::AddMember(const size_t &newX, const size_t &newY, const float &newVal)
{
	AddMember( ClustMember(newX, newY, newVal) );
}

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

