#include "GeoPoint.h"
#include "Map.h"
#include <iostream>
using namespace std;

struct SampleType
{
	GeoPoint* x;
	GeoPoint* d;
	int rId;
	SampleType(GeoPoint* x, GeoPoint* d, int rId)
	{
		this->x = x;
		this->d = d;
		this->rId = rId;
	}
};

class Evaluation
{
public:
	void evaluateSMM();
	void evaluateSMMD();

	void filterTestData();

//private
	vector<SampleType> testDataSet;
};
/*
void Evaluation::evaluateSMM()
{
	int correctCount = 0;
	int allCount = testDataSet.size();
	int currentCount = 0;
	for each (SampleType testData in testDataSet)
	{
		//if (currentCount != )
		{
		}
	}
}

void Evaluation::evaluateSMMD()
{

}*/