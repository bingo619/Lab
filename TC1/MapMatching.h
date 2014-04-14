/* 
 * Last Updated at [2014/4/14 12:49] by YKX
 */
#pragma once
#include <iostream>
#include <fstream>
#include <direct.h>
#include <io.h>
#include <vector>
#include <map>
#include "GeoPoint.h"
#include "Map.h"
using namespace std;

//��ͼƥ�����ò���
#define COEFFICIENT_FOR_EMISSIONPROB 140.2384599822997282786640971977//ԭʼֵΪ0.01402384599822997282786640971977��������10000��
#define COEFFICIENT_FOR_TRANSATIONPROB 0.31273997011//ԭʼֵΪ0.00931003342301998864175922391561��������10000��
//��ͼƥ��ͨ�ò���
#define MINPROB 1e-150 //������ʵ�����

extern Map roadNetwork;
extern list<Edge*> MapMatching(list<GeoPoint*> &trajectory, double rangeOfCandidateEdges = 50.0);
typedef list<GeoPoint*> Traj;