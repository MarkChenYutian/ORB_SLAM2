#include "Object.h"

namespace ORB_SLAM2 {

ObjectObservation::ObjectObservation(int trackID, float lx, float ly, float rx, float ry, const string &objectLabel):
    miTrackID(trackID), mdlx(lx), mdly(ly), mdrx(rx), mdry(ry),
    mpMapObject(nullptr), mpNextObservation(nullptr), mpPrevObservation(nullptr)
{
  if (objectLabel == "DontCare") {
    mObjectType = ObjectType::DONTCARE;
  } else {
    mObjectType = ObjectType::CARE;
  }
}

bool ObjectObservation::IsCareObject() const {
  return mObjectType != ObjectType::DONTCARE;
}

tuple<int, int, int> ObjectObservation::GetColorForDisplay() const {
  int r = 0, g = 0, b = 0;

  switch (mpMapObject->miTrackID % 6) {
    case 0 : {
      r = 200; break;
    } case 1 : {
      g = 200; break;
    } case 2 : {
      b = 200; break;
    } case 3 : {
      r = 200; g = 200; break;
    } case 4 : {
      r = 200; b = 200; break;
    } case 5 : {
      g = 200; b = 200; break;
    }
  }

  return make_tuple(r, g, b);
}

ObjectObservation* ObjectObservation::NextObservation() {
  return mpNextObservation;
}

ObjectObservation* ObjectObservation::PrevObservation() {
  return mpPrevObservation;
}

}
