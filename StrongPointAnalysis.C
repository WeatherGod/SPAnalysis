using namespace std;

#include "StrongPointAnalysis.h"	// for enum PointLabel, PointLoc
#include "Cluster.h"			// for class Cluster, struct ClustMember

#include <iostream>
#include <vector>
#include <algorithm>			// for max(), max_element()
#include <cctype>			// for size_t

#include <cmath>			// for pow(), hypot(), NAN, isfinite(), sqrt()


StrongPointAnalysis::StrongPointAnalysis()
	:	myData(0),
		myPointLabels(0),
		myXSize(0),
		myYSize(0),
		myStrongThreshold(NAN),
		myWeakThreshold(NAN),
		myWeakAssist(0.0),
		myReach(0.0),
		myUpperSensitivity(0.0),
		myLowerSensitivity(0.0),
		myPaddingLevel(0.0)
{
}

StrongPointAnalysis::StrongPointAnalysis(const Cluster &aCluster, 
					 const size_t &xSize, const size_t &ySize, 
					 const float &upperSensitivity, const float &lowerSensitivity, const float &paddingLevel, const float &reach)
	:	myData(ySize, vector<float>(xSize, 0.0)),
		myPointLabels(ySize, vector<PointLabel>(xSize, UNINIT)),
		myXSize(xSize),
		myYSize(ySize),
		myStrongThreshold(NAN),
		myWeakThreshold(NAN),
		myWeakAssist(0.0),
		myReach(reach),
		myUpperSensitivity(upperSensitivity),
		myLowerSensitivity(lowerSensitivity),
		myPaddingLevel(paddingLevel)
{
	vector<size_t> xLocs(aCluster.MemberCount());
        vector<size_t> yLocs(aCluster.MemberCount());
        vector<float> dataVals(aCluster.MemberCount());

        const vector<ClustMember> theMembers = aCluster.GiveMembers();

        for (size_t memberIndex = 0; memberIndex < theMembers.size(); memberIndex++)
        {
                xLocs[memberIndex] = theMembers[memberIndex].XLoc;
                yLocs[memberIndex] = theMembers[memberIndex].YLoc;
                dataVals[memberIndex] = theMembers[memberIndex].memberVal;
        }

	if (LoadData(xLocs, yLocs, dataVals))
	{
		AnalyzeBoard();
	}
	else
	{
		ResetBoard();
	}
}


StrongPointAnalysis::StrongPointAnalysis(const Cluster &aCluster,
                                         const size_t &xSize, const size_t &ySize,
                                         const float &sensitivityLevel, const float &paddingLevel, const float &reach)
        :       myData(ySize, vector<float>(xSize, 0.0)),
                myPointLabels(ySize, vector<PointLabel>(xSize, UNINIT)),
                myXSize(xSize),
                myYSize(ySize),
                myStrongThreshold(NAN),
                myWeakThreshold(NAN),
                myWeakAssist(0.0),
                myReach(reach),
                myUpperSensitivity(2.0*sensitivityLevel),
                myLowerSensitivity(sensitivityLevel),
                myPaddingLevel(paddingLevel)
{
        vector<size_t> xLocs(aCluster.MemberCount());
        vector<size_t> yLocs(aCluster.MemberCount());
        vector<float> dataVals(aCluster.MemberCount());

        const vector<ClustMember> theMembers = aCluster.GiveMembers();

        for (size_t memberIndex = 0; memberIndex < theMembers.size(); memberIndex++)
        {
                xLocs[memberIndex] = theMembers[memberIndex].XLoc;
                yLocs[memberIndex] = theMembers[memberIndex].YLoc;
                dataVals[memberIndex] = theMembers[memberIndex].memberVal;
        }

        if (LoadData(xLocs, yLocs, dataVals))
        {
                AnalyzeBoard();
        }
        else
        {
                ResetBoard();
        }
}





StrongPointAnalysis::StrongPointAnalysis(const vector<size_t> &xLocs, const vector<size_t> &yLocs, const vector<float> &dataVals,
					 const size_t &xSize, const size_t &ySize,
					 const float &upperSensitivity, const float &lowerSensitivity, const float &paddingLevel, const float &reach)
	:	myData(ySize, vector<float>(xSize, 0.0)),
		myPointLabels(ySize, vector<PointLabel>(xSize, UNINIT)),
		myXSize(xSize),
		myYSize(ySize),
		myStrongThreshold(NAN),
		myWeakThreshold(NAN),
		myWeakAssist(0.0),
		myReach(0.0),
		myUpperSensitivity(upperSensitivity),
		myLowerSensitivity(lowerSensitivity),
		myPaddingLevel(paddingLevel)
{
	if (LoadData(xLocs, yLocs, dataVals))
	{
		AnalyzeBoard();
	}
	else
	{
		ResetBoard();
	}
}


StrongPointAnalysis::StrongPointAnalysis(const vector<size_t> &xLocs, const vector<size_t> &yLocs, const vector<float> &dataVals,
                                         const size_t &xSize, const size_t &ySize,
                                         const float &sensitivityLevel, const float &paddingLevel, const float &reach)
        :       myData(ySize, vector<float>(xSize, 0.0)),
                myPointLabels(ySize, vector<PointLabel>(xSize, UNINIT)),
                myXSize(xSize),
                myYSize(ySize),
                myStrongThreshold(NAN),
                myWeakThreshold(NAN),
                myWeakAssist(0.0),
                myReach(0.0),
                myUpperSensitivity(2.0 * sensitivityLevel),
                myLowerSensitivity(sensitivityLevel),
                myPaddingLevel(paddingLevel)
{
        if (LoadData(xLocs, yLocs, dataVals))
        {
                AnalyzeBoard();
        }
        else
        {
                ResetBoard();
        }
}




bool StrongPointAnalysis::LoadData(const vector<size_t> &xLocs, const vector<size_t> &yLocs, const vector<float> &dataVals)
{
	/* Validity checking:
	   1. Check to see if the vectors xLocs, yLocs, and dataVals are all the same size.
	   2. Check that there is at least one piece of data to enter into the cluster board.
	   3. Check that there are no locations that lie outside the given domain.
        */
	if (xLocs.size() == yLocs.size() &&
	    xLocs.size() == dataVals.size() &&
	    xLocs.size() != 0 &&
	    myXSize > *max_element(xLocs.begin(), xLocs.end()) &&
	    myYSize > *max_element(yLocs.begin(), yLocs.end()))
	{
		for (size_t index = 0; index < xLocs.size(); index++)
		{
			// Don't load any infinities or NaNs.
			if (isfinite(dataVals[index]))
			{
				myPointLabels[yLocs[index]][xLocs[index]] = UNCHECKED;
				myData[yLocs[index]][xLocs[index]] += dataVals[index];
			}
		}

		return(true);
	}
	else
	{
		return(false);
	}
}


bool StrongPointAnalysis::LoadBoard(const Cluster &aCluster,
				    const size_t &xSize, const size_t &ySize,
				    const float &upperSensitivity, const float &lowerSensitivity, const float &paddingLevel, const float &reach)
{
	vector<size_t> xLocs(aCluster.MemberCount());
	vector<size_t> yLocs(aCluster.MemberCount());
	vector<float> dataVals(aCluster.MemberCount());

	const vector<ClustMember> theMembers = aCluster.GiveMembers();

	for (size_t memberIndex = 0; memberIndex < theMembers.size(); memberIndex++)
	{
		xLocs[memberIndex] = theMembers[memberIndex].XLoc;
		yLocs[memberIndex] = theMembers[memberIndex].YLoc;
		dataVals[memberIndex] = theMembers[memberIndex].memberVal;
	}

	return(LoadBoard(xLocs, yLocs, dataVals, xSize, ySize, upperSensitivity, lowerSensitivity, paddingLevel, reach));
}

bool StrongPointAnalysis::LoadBoard(const Cluster &aCluster,
                                    const size_t &xSize, const size_t &ySize,
                                    const float &sensitivityLevel, const float &paddingLevel, const float &reach)
{
        vector<size_t> xLocs(aCluster.MemberCount());
        vector<size_t> yLocs(aCluster.MemberCount());
        vector<float> dataVals(aCluster.MemberCount());

        const vector<ClustMember> theMembers = aCluster.GiveMembers();

        for (size_t memberIndex = 0; memberIndex < theMembers.size(); memberIndex++)
        {
                xLocs[memberIndex] = theMembers[memberIndex].XLoc;
                yLocs[memberIndex] = theMembers[memberIndex].YLoc;
                dataVals[memberIndex] = theMembers[memberIndex].memberVal;
        }

        return(LoadBoard(xLocs, yLocs, dataVals, xSize, ySize, sensitivityLevel, paddingLevel, reach));
}




bool StrongPointAnalysis::LoadBoard(const vector<size_t> &xLocs, const vector<size_t> &yLocs, const vector<float> &dataVals,
			     const size_t &xSize, const size_t &ySize,
			     const float &upperSensitivity, const float &lowerSensitivity, const float &paddingLevel, const float &reach)
{
	ResetBoard();

	if (xLocs.size() == yLocs.size() &&
	    xLocs.size() == dataVals.size() &&
	    xLocs.size() != 0 &&
	    xSize > *max_element(xLocs.begin(), xLocs.end()) &&
	    ySize > *max_element(yLocs.begin(), yLocs.end()))
	{
		bool goodLoad = true;

		myPaddingLevel = paddingLevel;
		myUpperSensitivity = upperSensitivity;
		myLowerSensitivity = lowerSensitivity;
		myReach = reach;
		myXSize = xSize;
		myYSize = ySize;
		myData.resize(ySize, vector< float >(xSize, 0.0));
		myPointLabels.resize(ySize, vector< PointLabel >(xSize, UNINIT));

		if (LoadData(xLocs, yLocs, dataVals))
		{
			AnalyzeBoard();
			goodLoad = true;
		}
		else
		{
			ResetBoard();
			goodLoad = false;
		}

		return(goodLoad);
	}
	else
	{
		return(false);
	}
}


bool StrongPointAnalysis::LoadBoard(const vector<size_t> &xLocs, const vector<size_t> &yLocs, const vector<float> &dataVals,
                             const size_t &xSize, const size_t &ySize,
                             const float &sensitivityLevel, const float &paddingLevel, const float &reach)
{
        ResetBoard();

        if (xLocs.size() == yLocs.size() &&
            xLocs.size() == dataVals.size() &&
            xLocs.size() != 0 &&
            xSize > *max_element(xLocs.begin(), xLocs.end()) &&
            ySize > *max_element(yLocs.begin(), yLocs.end()))
        {
                bool goodLoad = true;

                myPaddingLevel = paddingLevel;
                myUpperSensitivity = 2.0 * sensitivityLevel;
                myLowerSensitivity = sensitivityLevel;
                myReach = reach;
                myXSize = xSize;
                myYSize = ySize;
                myData.resize(ySize, vector< float >(xSize, 0.0));
                myPointLabels.resize(ySize, vector< PointLabel >(xSize, UNINIT));

                if (LoadData(xLocs, yLocs, dataVals))
                {
                        AnalyzeBoard();
                        goodLoad = true;
                }
                else
                {
                        ResetBoard();
                        goodLoad = false;
                }

                return(goodLoad);
        }
        else
        {
                return(false);
        }
}




void StrongPointAnalysis::AnalyzeBoard()
{
	if (GridSize() <= 1)
	{
		return;
	}

	double theSum = 0.0;
	double sumOfSquares = 0.0;
	size_t pointCount = 0;

	for (size_t Y = 0; Y < myYSize; Y++)
        {
                for (size_t X = 0; X < myXSize; X++)
                {
			if (myPointLabels[Y][X] != UNINIT)
			{
                        	theSum += (double) myData[Y][X];
	                        sumOfSquares += pow((double) myData[Y][X], 2.0);
				pointCount++;
			}
                }
        }

        const double avgVal = theSum / (double) (pointCount);

	// Point Estimators of the parameters of the Gamma Distribution.
        const double BetaVal = (( sumOfSquares / (double) pointCount ) - pow(avgVal, 2.0)) / avgVal;
	const double AlphaVal = pow(avgVal, 2.0) / ((sumOfSquares / (double) pointCount) - pow(avgVal, 2.0));



        const double devVal = sqrt(AlphaVal*pow(BetaVal, 2.0));

	
        cout << "Stat -- Avg: " << avgVal << "   StdDeviation: " << devVal 
	     << "  theSum: " << theSum << "  sumOfSquares: " << sumOfSquares << "  pointCount: " << pointCount << '\n';
	cout << "      Alpha: " << AlphaVal << "    Beta: " << BetaVal << '\n';


	myStrongThreshold = (float) max(avgVal + (myUpperSensitivity * BetaVal * devVal), 0.0);
	myWeakThreshold = (float) max(avgVal + (log(myLowerSensitivity * BetaVal) * devVal), 0.0);

	myWeakAssist = myWeakThreshold * (myPaddingLevel / 10.0);
	cout << "Thresholds    STRONG: " << myStrongThreshold << "   WEAK: " << myWeakThreshold 
	     << "  WeakAssist: " << myWeakAssist << "  Reach: " << myReach << '\n';

	if (devVal < 1.0 && BetaVal < 1.0)
	{
		// Don't let clustering occur.  It would be pretty much useless.
		ResetBoard();
	}
}


size_t StrongPointAnalysis::GridPointsUsed() const
{
	size_t RunningSum = 0;

	for (size_t Y = 0; Y < myYSize; Y++)
	{
		for (size_t X = 0; X < myXSize; X++)
		{
			if (myPointLabels[Y][X] > IGNORABLE)
			{
				RunningSum++;
			}
		}
	}

	return(RunningSum);
}

size_t StrongPointAnalysis::GridSize() const
{
	return(myXSize * myYSize);
}


void StrongPointAnalysis::PrintBoard() const
// This only existed for debugging use.  It just so happened that the 
// domain was small enough to represent on the screen.  Don't use for regular use.
{
	if (myYSize == 0 || myXSize == 0)
	{
		cout << "No map to print..." << endl;
		return;
	}

	const size_t ColWidth = 2;

	cout << "Thresholds   STRONG: " << myStrongThreshold << "   WEAK: " << myWeakThreshold << "   WeakAssist: " << myWeakAssist << '\n'
	     << "X:";
	for (size_t X = 0; X < myXSize; X++)
	{
		printf("% *u ", ColWidth, X);
	}

	cout << "\n/-"
	     << string((ColWidth + 1) * myXSize, '-')
	     << "\\Y:\n";

	for (size_t Y = 0; Y < myYSize; Y++)
	{
		cout << "| ";
		for (size_t X = 0; X < myXSize; X++)
		{
			switch (myPointLabels[Y][X])
			{
			case UNINIT:
				cout << "   ";
				break;
			case UNCHECKED:
				cout << " + ";
				break;
			case IGNORABLE:
				cout << " . ";
				break;
			case WEAK:
				cout << " ~ ";
				break;
			case STRONG:
				cout << " * ";
				break;
			default:
				cout << " ? ";
				break;
			}
		}

		cout << '|' << Y << endl;
	}

	cout << "\\-"
	     << (string((ColWidth + 1) * myXSize, '-'))
	     << "/\n";
}

void StrongPointAnalysis::ResetBoard()
{
	for (size_t yIndex = 0; yIndex < myYSize; yIndex++)
	{
		myData[yIndex].clear();
		myPointLabels[yIndex].clear();
	}

	myData.clear();
	myPointLabels.clear();

	myXSize = 0;
	myYSize = 0;
	myStrongThreshold = NAN;
	myWeakThreshold = NAN;
	myWeakAssist = 0.0;
	myReach = 0.0;
	myPaddingLevel = 0.0;
	myUpperSensitivity = 0.0;
	myLowerSensitivity = 0.0;
}

bool StrongPointAnalysis::BeenChecked(const size_t &XLoc, const size_t &YLoc) const
{
	return( UNCHECKED < myPointLabels[YLoc][XLoc] );
}

bool StrongPointAnalysis::IsIgnorablePoint(const size_t &XLoc, const size_t &YLoc) const
{
	if ( BeenChecked(XLoc, YLoc) )
	{
		return( IGNORABLE == myPointLabels[YLoc][XLoc] );
	}
	else if (myWeakThreshold >= myData[YLoc][XLoc])
	{
		myPointLabels[YLoc][XLoc] = IGNORABLE;
		return(true);
	}
	else
	{
		return(false);
	}
}

bool StrongPointAnalysis::IsStrongPoint(const size_t &Xindex, const size_t &Yindex) const
/*
    The idea behind this method is that a "strong point" is most likely surrounded with other significant values
       So, we look and see if we can "draw" a line through the point at (Xindex, Yindex) without hitting gridpoints with significant values
       If we can draw that line, then that means that the point is most likely at the edge of a cluster or not on a cluster
       If we can not draw that line, then the point is surrounded with other grid points with decent values, making it a strong point of the cluster
     I also added a new rule that a point of a certain density, irregardless of surrounding values will test positive.
*/
{
	if ( BeenChecked(Xindex, Yindex) )
	{
		return( STRONG == myPointLabels[Yindex][Xindex] );
	}

	if ( IsIgnorablePoint(Xindex, Yindex) )
	{
		return(false);
	}

        if ( myData[Yindex][Xindex] >= myStrongThreshold )
	//  The if statement asks if the point is strong enough to stand on its own.
	//    if it isn't, then it goes to the following block where it checks the surroundings
	{
		// The point satisfied the Special Strong Point rule, which overrides the general rule.
		myPointLabels[Yindex][Xindex] = STRONG;
                return(true);
	}
	else
        {
		// The objective of the following algorithm is to prove that the point is a strong point.
		// To prove that it is a strong point, we try to draw a straight line through the point
		// with a radius of 1 without hitting any other points of significance.
		// If such a line can not be drawn, then the point is surrounded by 
		// enough significant points to be considered a strong point.
		// If such a line can be drawn, then it is likely that the point 
		// is on the edge of a cluster and shouldn't be considered a strong point.
                bool CanMakeLine = false;
                

		// I only need to check four possible lines:
		// \ * *      * | *      * * /     * * *
		// * \ *      * | *      * / *     - - -
		// * * \      * | *      / * *     * * *


		if (Xindex > 0 && Xindex + 1 < myXSize &&
		    Yindex > 0 && Yindex + 1 < myYSize)
		{
			// The center point is interior to the grid.
			CanMakeLine = (   (IsIgnorablePoint(Xindex - 1, Yindex - 1) && 		// diagonal, top left to bottom right
					   IsIgnorablePoint(Xindex + 1, Yindex + 1))
				       || (IsIgnorablePoint(Xindex, Yindex - 1) &&		// vertical
					   IsIgnorablePoint(Xindex, Yindex + 1))
				       || (IsIgnorablePoint(Xindex + 1, Yindex - 1) &&		// diagonal, top right to bottom left
					   IsIgnorablePoint(Xindex - 1, Yindex + 1))
				       || (IsIgnorablePoint(Xindex + 1, Yindex) &&		// horizontal
					   IsIgnorablePoint(Xindex - 1, Yindex)));
		}
		else
		{
			// The center point is on an edge of the grid.
			
			// any points outside the domain are considered ignorable points.
			// this does decrease the presence of strong points around the edge, but this is minimal as
			// the gridpoints can still be strong if there are significant points entirely on one side.
			// Besides, if it is on an edge, and is not surrounded by significant points,
			// then it is likely to be at the edge of a cluster and shouldn't be a strong point.

			// A center point at a corner can never be a strong point by this rule, 
			// because a line can ALWAYS be drawn (diagonal).
			// A center point on an edge can only be a strong point if the three
			// neighboring points in the next row (or column) are significant
			// and at least one of the two neighbor points are significant.

			if ((Xindex == 0 && (Yindex == 0 || Yindex + 1 == myYSize)) ||
			    (Xindex + 1 == myXSize && (Yindex == 0 || Yindex + 1 == myYSize)))
			{
				// The center point is on a corner of the domain.
				// No need to check, the diagonal on the corner can be drawn.
				CanMakeLine = true;
			}
			else
			{
				// The center point is on a edge of the domain.
				if (Xindex == 0)
				{
					CanMakeLine = (IsIgnorablePoint(1, Yindex - 1) ||
					    	       IsIgnorablePoint(1, Yindex) ||
					    	       IsIgnorablePoint(1, Yindex + 1) ||
					    	       (IsIgnorablePoint(0, Yindex - 1) && IsIgnorablePoint(0, Yindex + 1)));
				}
				else if (Xindex + 1 == myXSize)
				{
					CanMakeLine = (IsIgnorablePoint(Xindex - 1, Yindex - 1) ||
                                            	       IsIgnorablePoint(Xindex - 1, Yindex) ||
                                            	       IsIgnorablePoint(Xindex - 1, Yindex + 1) ||
                                            	       (IsIgnorablePoint(Xindex, Yindex - 1) && IsIgnorablePoint(Xindex, Yindex + 1)));
				}
				else if (Yindex == 0)
				{
					CanMakeLine = (IsIgnorablePoint(Xindex - 1, 1) ||
                                        	       IsIgnorablePoint(Xindex, 1) ||
                                            	       IsIgnorablePoint(Xindex + 1, 1) ||
                                            	       (IsIgnorablePoint(Xindex - 1, 0) && IsIgnorablePoint(Xindex + 1, 0)));
				}
				else if (Yindex + 1 == myYSize)
				{
					CanMakeLine = (IsIgnorablePoint(Xindex - 1, Yindex - 1) ||
                                        	       IsIgnorablePoint(Xindex, Yindex - 1) ||
                                        	       IsIgnorablePoint(Xindex + 1, Yindex - 1) ||
                                        	       (IsIgnorablePoint(Xindex - 1, Yindex) && IsIgnorablePoint(Xindex + 1, Yindex)));
				}
			}
		}

		if (!CanMakeLine)
		{
			myPointLabels[Yindex][Xindex] = STRONG;
		}

                return(!CanMakeLine);	// if no lines can be made, then the point is a strong point
        }
}



double StrongPointAnalysis::StrongPointsTouch(const size_t &XLoc, const size_t &YLoc) const
//  This function counts the number of gridpoints immediately surrounding (XLoc, YLoc) that are strong points
//    and it weights the count according to distance from (Xloc, Yloc)
{
	double StrongPointCount = 0.0;

	const int xStart = (XLoc > 0 ? -1 : 0);
	const int xEnd = (XLoc + 1 < myXSize ? 1 : 0);
	const int yStart = (YLoc > 0 ? -1 : 0);
	const int yEnd = (YLoc + 1 < myYSize ? 1 : 0);

	for (int yIndex = yStart; yIndex <= yEnd; yIndex++)
	{
                for (int xIndex = xStart; xIndex <= xEnd; xIndex++)
                {
			if (xIndex != 0 || yIndex != 0)
			//  only count points that are surrounding (XLoc, YLoc), not (XLoc, YLoc) itself...
                        {
                        	if (IsStrongPoint(XLoc + xIndex, YLoc + yIndex))
				{
					StrongPointCount += 1.0 / hypot((double) xIndex, 
									(double) yIndex);
                                }
                	}
                }
        }

        return(StrongPointCount);
}


bool StrongPointAnalysis::IsWeakPoint(const size_t &XLoc, const size_t &YLoc) const
{
	if ( BeenChecked(XLoc, YLoc) )
	{
		return( WEAK == myPointLabels[YLoc][XLoc] );
	}

	if ( myData[YLoc][XLoc] > myWeakThreshold )
	{
		myPointLabels[YLoc][XLoc] = WEAK;
		return(true);
	}
	else
	{
		const double assistedValue = myData[YLoc][XLoc] + (StrongPointsTouch(XLoc, YLoc) * myWeakAssist);

		if (assistedValue >= myWeakThreshold)
		{
			myPointLabels[YLoc][XLoc] = WEAK;
		}

		return(assistedValue >= myWeakThreshold);
	}
}



vector<Cluster>
StrongPointAnalysis::DoCluster() const
{
	vector<Cluster> theClusters(0);

	// Go to each grid location and start a recursive search for
	// strong points.  This will return a Cluster to be added to "theClusters".
	// Then move onto the next point
	for (size_t Yindex = 0; Yindex < myYSize; Yindex++)
	{
		for (size_t Xindex = 0; Xindex < myXSize; Xindex++)
		{
			// Note that if a spot has already been checked,
			// then it is either a strong point that exists
			// for a cluster that has already been found,
			// or it is either weak or ignorable, which
			// we don't care about anyway.
			if (!BeenChecked(Xindex, Yindex))
			{

				Cluster newCluster;
				FindStrongPoints(Xindex, Yindex, newCluster);

				if (!newCluster.IsEmpty())
				{		
					theClusters.push_back(newCluster);
				}
			}
		}
	}

	vector<Cluster> secIterClusts(0);
	for (vector<Cluster>::iterator aClust = theClusters.begin();
	     aClust != theClusters.end();
	     aClust++)
	{
		// NOTE:  Padding of clusters must be done AFTER all of the strong points have been
		// networked.  PadCluster() calls the function IsWeakPoint(), which can call
		// IsStrongPoint() for points off of the network.  Therefore, if it is called
		// while the main clusters are still being found, then points outside of the cluster
		// may become set as strong points, and then ignored when FindStrongPoints() looks for the
		// next network, because they are already set.
		PadCluster(*aClust);

		// Always returns at least the original cluster.
		const vector<Cluster> subClusters = SubCluster(*aClust);

		secIterClusts.insert(secIterClusts.end(), subClusters.begin(), subClusters.end());
	}

	cout << "Cluster Count: " << secIterClusts.size() << '\n';

	return(secIterClusts);
}


vector<Cluster>
StrongPointAnalysis::SubCluster(const Cluster &origCluster) const
{
	vector<Cluster> theClusters(0);

	cout << "\tMember Count: " << origCluster.MemberCount() << '\n';
	// Don't bother subclustering clusters with less than 6 datapoints.
	if (origCluster.MemberCount() >= 6 && origCluster.MemberCount() < GridPointsUsed())
	{
		StrongPointAnalysis newSPA(origCluster, myXSize, myYSize, myUpperSensitivity, myLowerSensitivity, myPaddingLevel, myReach);

		const vector<Cluster> subClusters = newSPA.DoCluster();

		// If I didn't get any division of clusters, then keep the original.
		if (!subClusters.empty())
		{
			theClusters.insert(theClusters.end(), subClusters.begin(), subClusters.end());	
		}
		else
		{
			theClusters.push_back(origCluster);
		}
	}
	else
	{
		theClusters.push_back(origCluster);
	}

	return(theClusters);
}


void
StrongPointAnalysis::FindStrongPoints(const size_t &Xindex, const size_t &Yindex, Cluster &newCluster) const
{
	if (IsStrongPoint(Xindex, Yindex))
	{
		newCluster.AddMember(Xindex, Yindex, myData[Yindex][Xindex]);

		const int startX = (Xindex > myReach ? (int) -myReach : -(int) Xindex);
		const int startY = (Yindex > myReach ? (int) -myReach : -(int) Yindex);
		const int endX = (Xindex < myXSize - (int) myReach ? (int) myReach : (int) myXSize - Xindex - 1);
		const int endY = (Yindex < myYSize - (int) myReach ? (int) myReach : (int) myYSize - Yindex - 1);

		for (int yOffset = startY; yOffset <= endY; yOffset++)
		{
			for (int xOffset = startX; xOffset <= endX; xOffset++)
			{
				if ((xOffset != 0 || yOffset != 0) &&
				    myReach > hypot((double) xOffset, (double) yOffset) &&
				    !BeenChecked(Xindex + xOffset, Yindex + yOffset))
				{
					// Don't Network from the same location as [Xindex, Yindex],
					// And, don't network from a spot that already has been checked,
					// because if it has been checked, then it is either a strong point
					// that I already know about, or it is a weak or ignorable point
					// that I don't care about.
					FindStrongPoints(Xindex + xOffset, Yindex + yOffset, newCluster);
				}
			}
		}
	}
}

void
StrongPointAnalysis::PadCluster(Cluster &baseCluster) const
// This function finds any neighbooring weak points and adds them to the base cluster.
// This must be done AFTER all of the strong points have already been found for the cluster.
{
	const vector<ClustMember> currMembers = baseCluster.GiveMembers();
	vector<PointLoc> strongLocs(currMembers.size());

	for (size_t index = 0; index < currMembers.size(); index++)
	{
		strongLocs[index] = PointLoc(currMembers[index].XLoc, currMembers[index].YLoc);
	}

	vector<PointLoc> clustDomain;
	
	for (vector<PointLoc>::const_iterator aStrongPoint = strongLocs.begin();
	     aStrongPoint != strongLocs.end();
	     aStrongPoint++)
	{
		const int startX = (aStrongPoint->XLoc > myReach ? (int) -myReach : -(int) aStrongPoint->XLoc);
                const int startY = (aStrongPoint->YLoc > myReach ? (int) -myReach : -(int) aStrongPoint->YLoc);
                const int endX = (aStrongPoint->XLoc < myXSize - (int) myReach ? (int) myReach : (int) myXSize - aStrongPoint->XLoc - 1);
                const int endY = (aStrongPoint->YLoc < myYSize - (int) myReach ? (int) myReach : (int) myYSize - aStrongPoint->YLoc - 1);

/*
		const size_t startX = (aStrongPoint->XLoc > 0 ? aStrongPoint->XLoc - 1 : 0);
		const size_t startY = (aStrongPoint->YLoc > 0 ? aStrongPoint->YLoc - 1 : 0);
		const size_t endX = ((aStrongPoint->XLoc + 1) < myXSize ? aStrongPoint->XLoc + 1 : aStrongPoint->XLoc);
		const size_t endY = ((aStrongPoint->YLoc + 1) < myYSize ? aStrongPoint->YLoc + 1 : aStrongPoint->YLoc);
*/

		for (int yOffset = startY; yOffset <= endY; yOffset++)
		{
			for (int xOffset = startX; xOffset <= endX; xOffset++)
			{
				if ((xOffset != 0 || yOffset != 0) &&
				    myReach > hypot((double) xOffset, (double) yOffset))
				{
					const PointLoc clustDomainPoint(aStrongPoint->XLoc + xOffset, 
									aStrongPoint->YLoc + yOffset);

					if (!binary_search(strongLocs.begin(), strongLocs.end(), clustDomainPoint) &&
					    !binary_search(clustDomain.begin(), clustDomain.end(), clustDomainPoint))
					{
						clustDomain.insert(lower_bound(clustDomain.begin(), clustDomain.end(), clustDomainPoint),
								   clustDomainPoint);
					}
				}
			}
		}
	}

	// At this point, clustDomain has all of the points that borders the strong points.
	// We will now check to see if any of them are weak points.
	// BUG: This quick-and-dirty algorithm still can't differentiate between a
	// strong point in the current cluster and a strong point in a neighboring cluster.
	// NOTE: This is now less-and-less of an issue now with the use of sub-clustering.
	for (vector<PointLoc>::const_iterator pointCheck = clustDomain.begin();
	     pointCheck != clustDomain.end();
	     pointCheck++)
	{
		if (IsWeakPoint(pointCheck->XLoc, pointCheck->YLoc))
		{
			baseCluster.AddMember(pointCheck->XLoc, pointCheck->YLoc, myData[pointCheck->YLoc][pointCheck->XLoc]);
		}
	}
}
