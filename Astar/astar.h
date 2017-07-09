#ifndef ASTAR_H
#define ASTAR_H

#define ZY_UINT32  unsigned int
#define ZY_ULONG64 unsigned long long
#include <stdio.h>
#include <iostream>
#include <list>
#include <set>
#include <map>
#include <numeric>
#include <algorithm>

using namespace std;

//PointItem
struct PointItem{
    ZY_UINT32 px;
    ZY_UINT32 py;

    PointItem(ZY_UINT32 x,ZY_UINT32 y):px(x),py(y){}
    PointItem():px(0),py(0){}
    PointItem(const PointItem& pos){this->px = pos.px; this->py = pos.py;}

    bool operator==(const PointItem &item1) const
    {
        if((item1.px == this->px)&&(item1.py == this->py))
            return true;
        else
            return false;
    }

    bool operator!=(const PointItem &item1) const
    {
        if((item1.px != this->px)||(item1.py != this->py))
            return true;
        else
            return false;
    }


    bool operator <(const PointItem &item1) const
    {
        if(this->px < item1.px)
            return true;
        else if(this->px == item1.px)
        {
            if(this->py < item1.py)
                return true;
            else
                return false;
        }
        else
            return false;
    }

    bool operator > (const PointItem &item1) const
    {
        if(this->px > item1.px)
            return true;
        else if(this->px == item1.px)
        {
            if(this->py > item1.py)
                return true;
            else
                return false;
        }
        else
            return false;
    }

    PointItem operator *(int x) const
    {
        PointItem tmp;
        tmp.px = this->px*x;
        tmp.py = this->py*x;
        return tmp;
    }

    PointItem operator +(const PointItem &item) const
    {
        PointItem tmp;
        tmp.px = this->px + item.px;
        tmp.py = this->py + item.py;
        return tmp;
    }
};

//路径点结构体
struct Point
{
    int x,y;
    int F,G,H;
    Point *parent;
    Point(int _x,int _y):x(_x),y(_y),F(0),G(0),H(0),parent(NULL)
    {
    }
};

//路径点key，用于排序
struct PointKey{
    int X,Y;
    PointKey(){}
    PointKey(int x,int y):X(x),Y(y){}
    bool operator <(const PointKey &item1) const
    {
        if(this->X > item1.X)
            return true;
        else if(this->X == item1.X)
        {
            if(this->Y > item1.Y)
                return true;
            else
                return false;
        }
        else
            return false;
    }
};

struct PointKey2{
    int X,Y,G;
    PointKey2(){}
    PointKey2(int x,int y,int g):X(x),Y(y),G(g){}
    bool operator <(const PointKey2 &item1) const
    {
        if(this->G < item1.G)
            return true;
        else if(this->G == item1.G)
        {
            if(this->X > item1.X)
                return true;
            else if(this->X == item1.X)
            {
                if(this->Y > item1.Y)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
        else
            return false;

    }
};

//地图中的时间点信息结构体
struct MapPoint{
      char PointType;
      std::set<ZY_UINT32> OccupyTime_set;
};

//A*算法类
class Astar
{
public:
    void InitAstar(std::vector<std::vector<MapPoint> > &_maze, float coef_x, float coef_y);          //A*算法数据初始化
    std::vector<PointItem> GetPath(PointItem startPoint, PointItem endPoint,ZY_UINT32 DelayTime);    //获取路径
    ~Astar();
private:
    Point *findPath(Point startPoint, Point endPoint, unsigned int DelayTime);                      //路径搜索
    std::vector<Point *> getSurroundPoints(const Point *point) const;                               //获取周边可达路径点
    bool isCanreach(const Point *point,const Point *target) const;                                  //检测是否可达
    float Coef_X;                                                                                   //X方向权重
    float Coef_Y;                                                                                   //Y方向权重
    float calcG(Point *temp_start,Point *point);                                                    //计算G值
    float calcH(Point *point,Point *end);
    float calcF(Point *point);
private:
    ZY_UINT32  Size_X;                                                                             //地图高度
    ZY_UINT32  Size_Y;                                                                             //地图宽度
    std::vector<std::vector<MapPoint> > maze;                                                      //地图数据
    map<PointKey,Point*> openList;                                                                 //开放列表
    map<PointKey,Point*> closeList;                                                                //关闭列表
    std::list<Point *> TrashList;                                                                  //垃圾列表，用于回收垃圾内存
};
#endif // ASTAR_H
