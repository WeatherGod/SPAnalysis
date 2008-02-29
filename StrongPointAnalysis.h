#ifndef _STRONGPOINTANALYSIS_H
#define _STRONGPOINTANALYSIS_H

#include <vector>
#include <cctype>	// for size_t

#include "Cluster.h"


class StrongPointAnalysis
{
	enum PointLabel { UNINIT = 0, UNCHECKED, IGNORABLE, WEAK, STRONG };

	class PointLoc
	{
		public:
			PointLoc()
				: XLoc(0), YLoc(0)
			{
			};

			PointLoc(const PointLoc &copyLoc)
				: XLoc(copyLoc.XLoc), YLoc(copyLoc.YLoc)
			{
			};

			PointLoc(const size_t &XIndex, const size_t &YIndex)
				: XLoc(XIndex), YLoc(YIndex)
			{
			};

			size_t XLoc, YLoc;

		friend bool operator < (const PointLoc &Lefty, const PointLoc &Righty)
		{
			return(Lefty.XLoc < Righty.XLoc || ((Lefty.XLoc == Righty.XLoc) && (Lefty.YLoc < Righty.YLoc)));
		};
	};

	public:
		StrongPointAnalysis();
		StrongPointAnalysis(const vector<size_t> &xLocs, const vector<size_t> &yLocs, const vector<float> &dataVals,
				    const size_t &xSize, const size_t &ySize,
				    const float &upperSensitivity, const float &lowerSensitivity, const float &paddingLevel, const float &reach,
				    const int &clusterLevel = 0);
		StrongPointAnalysis(const Cluster &aClust, 
				    const size_t &xSize, const size_t &ySize, 
				    const float &upperSensitivity, const float &lowerSensitivity, const float &paddingLevel, const float &reach,
				    const int &clusterLevel = 0);


		void PrintBoard() const;

		bool LoadBoard(const Cluster &aClust, 
			       const size_t &xSize, const size_t &ySize,
			       const float &upperSensitivity, const float &lowerSensitivity, const float &paddingLevel, const float &reach);

		bool LoadBoard(const vector<size_t> &xLocs, const vector<size_t> &yLocs, const vector<float> &dataVals,
			       const size_t &xSize, const size_t &ySize,
			       const float &upperSensitivity, const float &lowerSensitivity, const float &paddingLevel, const float &reach);


		vector<Cluster> DoCluster() const;
		
	private:
		vector< vector< float > > myData;
		mutable vector< vector< PointLabel > > myPointLabels;

		size_t myXSize;
		size_t myYSize;

		float myStrongThreshold;
		float myWeakThreshold;
		float myWeakAssist;
		
		float myReach;
		float myUpperSensitivity;
		float myLowerSensitivity;
		float myPaddingLevel;

		int myClusterLevel;


		bool IsStrongPoint(const size_t &xLoc, const size_t &yLoc) const;
		bool IsWeakPoint(const size_t &xLoc, const size_t &yLoc) const;
		bool IsIgnorablePoint(const size_t &xLoc, const size_t &yLoc) const;
		bool BeenChecked(const size_t &xLoc, const size_t &yLoc) const;
		bool IsUninitialized(const size_t &xLoc, const size_t &yLoc) const;

		void FindStrongPoints(const size_t &Xindex, const size_t &Yindex, Cluster &newCluster) const;
		void PadCluster(Cluster &baseCluster) const;
		vector<Cluster> SubCluster(const Cluster &origCluster) const;

		void AnalyzeBoard();
		bool LoadData(const vector<size_t> &xLocs, const vector<size_t> &yLocs, const vector<float> &dataVals);
		void ResetBoard();

		double StrongPointsTouch(const size_t &XLoc, const size_t &YLoc) const;
		size_t GridSize() const;
		size_t GridPointsUsed() const;
};

#endif
