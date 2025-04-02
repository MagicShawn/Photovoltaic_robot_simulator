#ifndef _MY_GRIDNODE_H_
#define _MY_GRIDNODE_H_

#include <Eigen/Eigen>


#define inf 1>>20

class GridNode;
typedef GridNode* GridNodePtr;
typedef GridNodePtr *** GridMapPtr;

//Node class
class GridNode
{
public:
    int id;// 1--> open set, -1 --> closed set
    char dir;     // 'R'--> right, 'L'--> left, 'U'--> up, 'D'--> down, '0'--> no direction

    Eigen::Vector3d cord;
    Eigen::Vector3i index;

    double gcost,fcost;
    GridNodePtr parent;

    bool obsFlag;

public:

    GridNode(Eigen::Vector3d _cord,Eigen::Vector3i _index){
        id = 0;
        dir = '0';
        cord  = _cord;
        index = _index;

        gcost = inf;
        fcost = inf;
        parent = NULL;

        obsFlag = false;
    }

    GridNode(){};
    ~GridNode(){};
};



#endif
