#include "Object.h"

namespace ORB_SLAM2 {

ObjectObservation::ObjectObservation(int trackID, float lx, float ly, float rx, float ry, const string &objectLabel):
    miTrackID(trackID),
    mdlx(lx), mdly(ly), mdrx(rx), mdry(ry)
{
  if (objectLabel == "DontCare") {
    mObjectType = ObjectObservation::DONTCARE;
  } else {
    mObjectType = ObjectObservation::CARE;
  }
}

bool ObjectObservation::isCareObject() const {
  return mObjectType != ObjectObservation::DONTCARE;
}

}
