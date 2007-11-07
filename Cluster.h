#ifndef _CLUSTER_H
#define _CLUSTER_H


#include <vector>
#include <cmath>		// for NAN
#include <cctype>		// for size_t

struct ClustMember
{
	size_t XLoc;
	size_t YLoc;
	float memberVal;

	ClustMember()
		:	XLoc(0),
			YLoc(0),
			memberVal(NAN)
	{
	};

	ClustMember(const ClustMember &newMember)
		:	XLoc(newMember.XLoc),
			YLoc(newMember.YLoc),
			memberVal(newMember.memberVal)
	{
	};

	ClustMember(const size_t &newX, const size_t &newY, const float &newVal)
		:	XLoc(newX),
			YLoc(newY),
			memberVal(newVal)
	{
	};
};


class Cluster : public vector<ClustMember>
{
	public:
		Cluster();
//		Cluster(const Cluster &clustCopy);
		Cluster(const vector<size_t> &XLocs, const vector<size_t> &YLocs, const vector<float> &dataVals);

		void AddMember(const ClustMember &newMember);
		void AddMember(const size_t &newX, const size_t &newY, const float &newVal);
//		vector<ClustMember> GiveMembers() const;

//		size_t MemberCount() const;
//		bool IsEmpty() const;

//	private:
//		vector<ClustMember> myMembers;
};


#endif
