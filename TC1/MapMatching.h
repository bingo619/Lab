#pragma once
#include <iostream>
#include <fstream>
#include <direct.h>
#include <io.h>
#include <vector>
#include "GeoPoint.h"
#include "Map.h"
using namespace std;

//��ͼƥ�����ò���
#define COEFFICIENT_FOR_EMISSIONPROB 140.2384599822997282786640971977//ԭʼֵΪ0.01402384599822997282786640971977��������10000��
#define COEFFICIENT_FOR_TRANSATIONPROB 93.1003342301998864175922391561//ԭʼֵΪ0.00931003342301998864175922391561��������10000��
//#define RANGEOFCANADIDATEEDGES 50.0 //��ѡ·��ѡȡ��Χ
#define MINPROB 1e-150 //������ʵ�����

extern Map map;
extern list<Edge*> MapMatching(list<GeoPoint*> &trajectory, double candidatesPickRange);