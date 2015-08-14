/* 
 * Last Updated at [2015/4/14 21:50] by wuhao
 */
#pragma once
#include "MapDrawer.h"
#include <iostream>
#include "GeoPoint.h"
#include <list>
#include <vector>
using namespace std;

typedef list<GeoPoint*> Traj;
class TrajDrawer
{
public:
	//////////////////////////////////////////////////////////////////////////
	///��һ��Ĺ켣�Ĺ��߷���
	///boolLine: �켣�Ƿ��ô��߻���Ĭ��Ϊfalse
	///drawLines: �Ƿ񻭳��켣������Ĭ��Ϊtrue
	///bigPoint: �켣���Ƿ���ʮ�ֵ㣬Ĭ��Ϊtrue
	///drawTrajPt: �켣�Ƿ񻭳��켣�㣬Ĭ��Ϊtrue,�켣����ɫΪ��ɫ
	//////////////////////////////////////////////////////////////////////////
	static void drawOneTraj(Traj* traj, MapDrawer& md, Gdiplus::Color color, bool boldLine = false, bool drawLines = true, bool bigPoint = true, bool drawTrajPts = true);
	static void drawTrajs(vector<Traj*>& trajs, MapDrawer& md, Gdiplus::Color color, bool boldLine = false, bool drawLines = true, bool bigPoint = true, bool drawTrajPts = true);
	static void drawTrajs(list<Traj*>& trajs, MapDrawer& md, Gdiplus::Color color, bool boldLine = false, bool drawLines = true, bool bigPoint = true, bool drawTrajPts = true);

	//////////////////////////////////////////////////////////////////////////
	///ר�û�ƥ���Ĺ켣
	///Ĭ��ƥ��ɹ��ĵ�Ϊ�̵㣬ʧ�ܵĵ�Ϊ���
	//////////////////////////////////////////////////////////////////////////
	static void drawOneMMTraj(Traj* traj, MapDrawer& md, Gdiplus::Color color, bool boldLine = false, bool drawLines = true, bool bigPoint = true, bool drawTrajPts = true);
	static void drawMMTrajs(vector<Traj*>& trajs, MapDrawer& md, Gdiplus::Color color, bool boldLine = false, bool drawLines = true, bool bigPoint = true, bool drawTrajPts = true);
	static void drawMMTrajs(list<Traj*>& trajs, MapDrawer& md, Gdiplus::Color color, bool boldLine = false, bool drawLines = true, bool bigPoint = true, bool drawTrajPts = true);

};