//
// Created by yutian on 3/8/23.
//

#ifndef ORB_SLAM2_OBJECT_H
#define ORB_SLAM2_OBJECT_H
#include <System.h>

namespace ORB_SLAM2 {
    class MapPoint;

    enum ObjectType {
        CARE = 0,
        DONTCARE = -1,
    };

    class MapObject {
    public:
        int miTrackID;
        ObjectType mObjectType;
        vector<ObjectObservation*> mvObservations;

    public:
        MapObject(int TrackID, ObjectType ObjType);
    };

    /*
     * ObjectObservation is a class that handles the object bounding box provided by the dataset
     */
    class ObjectObservation {
    public:
        int miTrackID;
        ObjectType mObjectType;
        float mdlx, mdly, mdrx, mdry;
        vector<MapPoint*> mvObjectObsPoints;

        MapObject *mpMapObject;

        ObjectObservation* mpNextObservation;
        ObjectObservation* mpPrevObservation;
    public:
        ObjectObservation(int TrackID, float lx, float ly, float rx, float ry, const string &objectLabel);
        bool IsCareObject() const;
        tuple<int, int, int> GetColorForDisplay() const;

        ObjectObservation* NextObservation();
        ObjectObservation* PrevObservation();
    };
}

#endif //ORB_SLAM2_OBJECT_H
