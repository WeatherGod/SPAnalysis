using namespace std;

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <unistd.h>             // for optarg, opterr, optopt
#include <getopt.h>             // for getopt_long()

#include "StrongPointAnalysis.h"
#include "Cluster.h"

#include <netcdfcpp.h>


// g++ -g test_radar.C -o test_radar -I /usr/include/netcdf-3 -L ./ -lSPAnalysis -L /usr/lib/netcdf-3 -l netcdf_c++ -l netcdf -lm

// g++ -g test_radar.C -o test_radar -I /usr/include/BUtils -L ./ -lSPAnalysis -l BUtils -lm

bool OutputClusters(const string &filename, const vector< vector<float> > &dataVals, const vector<Cluster> &theClusters,
                    const size_t xSize, const size_t ySize, 
                    const float upperSensitivity, const float lowerSensitivity, const float paddingLevel, 
		    const float reach, const int subClustDepth);

void PrintHelp()
{
	cerr << "test_radar -i INPUT_FILE -o OUTPUT_FILE\n"
	     << "           -u UPPER_SENSITIVITY -l LOWER_SENSITIVITY\n"
	     << "           -p PADDING_LEVEL -r REACH -s SUBCLUST\n"
	     << "           [-h | --help]\n\n";
}


int main(int argc, char* argv[])
{
	float upperSensitivity = NAN;
	float lowerSensitivity = NAN;
	float paddingLevel = NAN;
	float reach = NAN;
	int subClustDepth = 0;

	string inputFilename = "";
	string outputFilename = "";

	int OptionIndex = 0;
	int OptionChar = 0;
	bool OptionError = false;
	opterr = 0;			// don't print out error messages, I will handle that.

	static struct option TheLongOptions[] = {
		{"input", 1, NULL, 'i'},
		{"output", 1, NULL, 'o'},
		{"upper", 1, NULL, 'u'},
		{"lower", 1, NULL, 'l'},
		{"padding", 1, NULL, 'p'},
		{"reach", 1, NULL, 'r'},
		{"subclust", 1, NULL, 's'},
		{"help", 0, NULL, 'h'},
		{0, 0, 0, 0}
	};

	while ((OptionChar = getopt_long(argc, argv, "i:o:u:l:p:r:s:h", TheLongOptions, &OptionIndex)) != -1)
	{
		switch (OptionChar)
		{
		case 'i':
			inputFilename = optarg;
			break;
		case 'o':
			outputFilename = optarg;
			break;
		case 'u':
			upperSensitivity = atof(optarg);
			break;
		case 'l':
			lowerSensitivity = atof(optarg);
			break;
		case 'p':
			paddingLevel = atof(optarg);
			break;
		case 'r':
			reach = atof(optarg);
			break;
		case 's':
			subClustDepth = atoi(optarg);
			break;
		case 'h':
			PrintHelp();
			return(1);
			break;
		case '?':
			cerr << "ERROR: Unknown argument: -" << (char) optopt << endl;
			OptionError = true;
			break;
		case ':':
			cerr << "ERROR: Missing value for argument: -" << (char) optopt << endl;
			OptionError = true;
			break;
		default:
			cerr << "ERROR: Programming error... Unaccounted option: -" << (char) OptionChar << endl;
			OptionError = true;
			break;
		}
	}

	if (OptionError)
	{
		PrintHelp();
		return(EXIT_FAILURE);
	}

	if  (inputFilename.empty() || outputFilename.empty()
	  || isnan(lowerSensitivity) || isnan(upperSensitivity)
	  || isnan(paddingLevel) || isnan(reach))
	{
		cerr << "Missing argument\n";
		PrintHelp();
		return(EXIT_FAILURE);
	}


	NcFile radarFile(inputFilename.c_str());

	if (!radarFile.is_valid())
	{
		cerr << "ERROR: Could not open radar file: " << inputFilename << " for reading.\n";
		return(EXIT_FAILURE);
	}


	
	NcVar* reflectVar = radarFile.get_var("value");

	if (reflectVar == NULL)
	{
		cerr << "ERROR: invalid data file.  No variable called 'value'!\n";
		radarFile.close();
		return(EXIT_FAILURE);
	}
		

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

	StrongPointAnalysis theSPA(xLocs, yLocs, values, 
				   xSize, ySize, 
				   upperSensitivity, lowerSensitivity, paddingLevel, reach, subClustDepth);

	cerr << "Loaded...\n";

	vector<Cluster> theClusters = theSPA.DoCluster();

	cerr << "Cluster Count: " << theClusters.size() << endl;
	
	if (!OutputClusters(outputFilename, theValues, theClusters, xSize, ySize, upperSensitivity, lowerSensitivity, 
										  paddingLevel, reach, subClustDepth))
	{
		cerr << "Problem outputing to file: " << outputFilename << '\n';
		return(EXIT_FAILURE);
	}

	return(EXIT_SUCCESS);
}



bool OutputClusters(const string &filename, const vector< vector<float> > &dataVals, const vector<Cluster> &theClusters,
		    const size_t xSize, const size_t ySize, 
		    const float upperSensitivity, const float lowerSensitivity, const float paddingLevel, 
		    const float reach, const int subClustDepth)
{

	ofstream outFile(filename.c_str());

	if (!outFile.is_open())
	{
		cerr << "Could not open file: " << filename << "\n";
		return(false);
	}

	outFile << xSize << ' ' << ySize << ' ' << upperSensitivity << ' ' << lowerSensitivity << ' ' 
		<< paddingLevel << ' ' << reach << ' ' << subClustDepth << '\n';

/*
	for (size_t yIndex = 0; yIndex < ySize; yIndex++)
	{
		outFile << dataVals[yIndex][0];
		for (size_t xIndex = 1; xIndex < xSize; xIndex++)
		{
			outFile << ' ' << dataVals[yIndex][xIndex];
		}
		outFile << '\n';
	}
*/

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

