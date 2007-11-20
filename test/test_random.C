using namespace std;

#include <cstdlib>		// for rand()
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "StrongPointAnalysis.h"
#include "Cluster.h"


// g++ -g test.C -o test -L ./ -lSPAnalysis -lm

bool OutputClusters(const string &filename, const vector< vector<float> > &dataVals, const vector<Cluster> &theClusters,
                    const size_t xSize, const size_t ySize, 
                    const double deviationsAbove, const double deviationsBelow, const float paddingLevel);


int main()
{
	const size_t dataCount = 100000;
	const size_t xSize = 100;
	const size_t ySize = 100;
	const float thePower = 0.25;

	const size_t focusPoints = 45;
	const float maxRadius = 35.0;

	vector<float> xCenters(focusPoints);
	vector<float> yCenters(focusPoints);

	for (size_t index = 0; index < focusPoints; index++)
	{
		xCenters[index] = (((double) rand() / (double) RAND_MAX) * (xSize - (2.0 * maxRadius) - 1.0)) + maxRadius;
		yCenters[index] = (((double) rand() / (double) RAND_MAX) * (ySize - (2.0 * maxRadius) - 1.0)) + maxRadius;
	}

	vector<size_t> xLocs(dataCount);
	vector<size_t> yLocs(dataCount);
	vector<float> values(dataCount);

	vector< vector<float> > dataVals(ySize, vector<float>(xSize, 0.0));

	for (size_t index = 0; index < dataCount; index++)
	{
		const float randAngle = (((double) rand() / (double) RAND_MAX) * 2.0 * M_PI);
		const float randDist = (((double) rand() / (double) RAND_MAX) * maxRadius);
		const size_t randFocus = (size_t) (((double) rand() / (double) RAND_MAX) * (focusPoints - 1));

		xLocs[index] = (size_t) (xCenters[randFocus] + (randDist * cosf(randAngle)));
		yLocs[index] = (size_t) (yCenters[randFocus] + (randDist * sinf(randAngle)));
		values[index] = (float) (((double) rand() / (double) RAND_MAX) * thePower);

		dataVals[yLocs[index]][xLocs[index]] += values[index];
	}



	const double deviationsAbove = 10.0;
	const double deviationsBelow = -10.0;
	const float paddingLevel = 0.0;

	StrongPointAnalysis theSPA(xLocs, yLocs, values, xSize, ySize, deviationsAbove, deviationsBelow, paddingLevel);

	cerr << "Loaded...\n";

	vector<Cluster> theClusters = theSPA.DoCluster();

	cerr << "Cluster Count: " << theClusters.size() << endl;
/*
	vector<Cluster> secIterClusts(0);


	for (vector<Cluster>::const_iterator aClust = theClusters.begin();
	     aClust != theClusters.end();
	     aClust++)
	{
		if (aClust->MemberCount() >= 6)
		{
			theSPA.LoadBoard(*aClust, xSize, ySize, deviationsAbove, deviationsBelow, paddingLevel);

			const vector<Cluster> newClusts = theSPA.DoCluster();

			secIterClusts.insert(secIterClusts.end(), newClusts.begin(), newClusts.end());
		}
	}

	theClusters.insert(theClusters.end(), secIterClusts.begin(), secIterClusts.end());
*/
	

	if (!OutputClusters("output.txt", dataVals, theClusters, xSize, ySize, deviationsAbove, deviationsBelow, paddingLevel))
	{
		cerr << "Problem output to file...\n";
		return(1);
	}

	return(0);
}



bool OutputClusters(const string &filename, const vector< vector<float> > &dataVals, const vector<Cluster> &theClusters,
		    const size_t xSize, const size_t ySize, 
		    const double deviationsAbove, const double deviationsBelow, const float paddingLevel)
{

	ofstream outFile(filename.c_str());

	if (!outFile.is_open())
	{
		cerr << "Could not open file: " << filename << "\n";
		return(false);
	}

	outFile << xSize << ' ' << ySize << ' ' << deviationsAbove << ' ' << deviationsBelow << ' ' << paddingLevel << '\n';

	for (size_t yIndex = 0; yIndex < ySize; yIndex++)
	{
		outFile << dataVals[yIndex][0];
		for (size_t xIndex = 1; xIndex < xSize; xIndex++)
		{
			outFile << ' ' << dataVals[yIndex][xIndex];
		}
		outFile << '\n';
	}

	outFile << theClusters.size() << '\n';

	for (vector<Cluster>::const_iterator aClust = theClusters.begin();
	     aClust != theClusters.end();
	     aClust++)
	{
		outFile << aClust->size() << '\n';

		for (Cluster::const_iterator aMember = aClust->begin();
		     aMember != aClust->end();
		     aMember++)
		{
			outFile << aMember->XLoc + 1 << ' ' << aMember->YLoc + 1 << ' ' << aMember->memberVal << '\n';
		}
	}

	outFile.close();

	return(true);
}

