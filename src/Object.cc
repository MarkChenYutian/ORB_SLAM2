#include <Object.h>

namespace ORB_SLAM2 {

ObjectBox::ObjectBox(int trackID, double lx, double ly, double rx, double ry, const string &objectLabel):
    miTrackID(trackID),
    mdlx(lx), mdly(ly), mdrx(rx), mdry(ry)
{
  if (objectLabel == "DontCare") {
    mObjectType = ObjectBox::DONTCARE;
  } else {
    mObjectType = ObjectBox::CARE;
  }
}

bool ObjectBox::isCareObject() {
  return mObjectType != ObjectBox::DONTCARE;
}

}
