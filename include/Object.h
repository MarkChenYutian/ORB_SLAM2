//
// Created by yutian on 3/8/23.
//

#ifndef ORB_SLAM2_OBJECT_H
#define ORB_SLAM2_OBJECT_H
#include <System.h>

namespace ORB_SLAM2 {
class ObjectBox {
public:
    enum ObjectType {
        CARE = 0,
        DONTCARE = -1,
    };

    int miTrackID;
    double mdlx, mdly, mdrx, mdry;
    ObjectType mObjectType;

    ObjectBox(int TrackID, double lx, double ly, double rx, double ry, const string &objectLabel);
    bool isCareObject();
};
}

#endif //ORB_SLAM2_OBJECT_H
