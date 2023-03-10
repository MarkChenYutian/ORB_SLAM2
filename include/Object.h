//
// Created by yutian on 3/8/23.
//

#ifndef ORB_SLAM2_OBJECT_H
#define ORB_SLAM2_OBJECT_H
#include <System.h>

namespace ORB_SLAM2 {
    class MapPoint;

    /*
     * ObjectObservation is a class that handles the object bounding box provided by the dataset
     */
    class ObjectObservation {
    public:
        enum ObjectType {
            CARE = 0,
            DONTCARE = -1,
        };

        int miTrackID;
        float mdlx, mdly, mdrx, mdry;
        ObjectType mObjectType;
        vector<MapPoint*> mvObjectObsPoints;

        ObjectObservation(int TrackID, float lx, float ly, float rx, float ry, const string &objectLabel);
        bool isCareObject() const;
    };
}

#endif //ORB_SLAM2_OBJECT_H
