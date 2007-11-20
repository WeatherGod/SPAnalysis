using namespace std;

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "StrongPointAnalysis.h"
#include "Cluster.h"

#include <netcdfcpp.h>


// g++ -g test_radar.C -o test_radar -I /usr/include/netcdf-3 -L ./ -lSPAnalysis -L /usr/lib/netcdf-3 -l netcdf_c++ -l netcdf -lm

// g++ -g test_radar.C -o test_radar -I /usr/include/BUtils -L ./ -lSPAnalysis -l BUtils -lm

bool OutputClusters(const string &filename, const vector< vector<float> > &dataVals, const vector<Cluster> &theClusters,
                    const size_t xSize, const size_t ySize, 
                    const float upperSensitivity, const float lowerSensitivity, const float paddingLevel, const float reach);


int main(int argc, char* argv[])
{
	if (argc < 2)
	{
		cerr << "No filename given!\n";
		return(EXIT_FAILURE);
	}


	NcFile radarFile(argv[1]);

	if (!radarFile.is_valid())
	{
		cerr << "Could not open radar file: " << argv[1] << " for reading.\n";
		return(EXIT_FAILURE);
	}


	NcVar* reflectVar = radarFile.get_var("value");
	long* dataEdges = reflectVar->edges();	// [0] - time, [1] - lat, [2] - lon
	double* dataVals = new double[reflectVar->num_vals()];
	const size_t xSize = dataEdges[2];
	const size_t ySize = dataEdges[1];

	reflectVar->get(dataVals, dataEdges);

	vector<size_t> xLocs(0);
	xLocs.reserve(xSize * ySize);

	vector<size_t> yLocs(0);
	yLocs.reserve(xSize * ySize);

	vector<float> values(0);
	values.reserve(xSize * ySize);

	vector< vector<float> > theValues(ySize, vector<float>(xSize, NAN));

	cout << "XSize: " << xSize << "   YSize: " << ySize << endl;

	size_t dataIndex = 0;
	for (size_t yIndex = 0; yIndex < ySize; yIndex++)
	{
		for (size_t xIndex = 0; xIndex < xSize; xIndex++, dataIndex++)
		{
			if (isfinite(dataVals[dataIndex]))
			{
				xLocs.push_back(xIndex);
				yLocs.push_back(yIndex);
				values.push_back(dataVals[dataIndex]);
				theValues[yIndex][xIndex] = dataVals[dataIndex];
			}
		}
	}

	radarFile.close();

	delete [] dataEdges;
	delete [] dataVals;

	cout << "values.size(): " << values.size() << endl;


	const float upperSensitivity = 1.5;
	const float lowerSensitivity = -0.75;
	const float paddingLevel = 5.0;
	const float reach = 2.5;

	StrongPointAnalysis theSPA(xLocs, yLocs, values, 
				   xSize, ySize, 
				   upperSensitivity, lowerSensitivity, paddingLevel, reach);

	cerr << "Loaded...\n";

	vector<Cluster> theClusters = theSPA.DoCluster();

	cerr << "Cluster Count: " << theClusters.size() << endl;

	char* outfileName = "output.txt";
	
	if (!OutputClusters(outfileName, theValues, theClusters, xSize, ySize, upperSensitivity, lowerSensitivity, paddingLevel, reach))
	{
		cerr << "Problem outputing to file: " << outfileName << '\n';
		return(EXIT_FAILURE);
	}

	return(0);
}



bool OutputClusters(const string &filename, const vector< vector<float> > &dataVals, const vector<Cluster> &theClusters,
		    const size_t xSize, const size_t ySize, 
		    const float upperSensitivity, const float lowerSensitivity, const float paddingLevel, const float reach)
{

	ofstream outFile(filename.c_str());

	if (!outFile.is_open())
	{
		cerr << "Could not open file: " << filename << "\n";
		return(false);
	}

	outFile << xSize << ' ' << ySize << ' ' << upperSensitivity << ' ' << lowerSensitivity << ' ' << paddingLevel << '\n';


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

