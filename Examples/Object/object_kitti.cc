//
// Created by yutian on 1/6/23.
//

#include<iostream>
#include<algorithm>
#include<chrono>
#include<dirent.h>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strPathToSeqLeft, const string &strPathToSeqRight, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
  if(argc != 6)
  {
    cerr << endl << "Usage: ./stereo_kitti "
                    "path_to_vocabulary "
                    "path_to_settings "
                    "path_to_sequence_left "
                    "path_to_sequence_right "
                    "path_to_bbox_file" << endl;
    return 1;
  }

  // Retrieve paths to images
  vector<string> vstrImageLeft;
  vector<string> vstrImageRight;
  vector<double> vTimestamps;

  LoadImages(string(argv[3]), string(argv[4]), vstrImageLeft, vstrImageRight, vTimestamps);

  const int nImages = vstrImageLeft.size();

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << nImages << endl << endl;

  // Main loop
  cv::Mat imLeft, imRight;
  for(int ni=0; ni<nImages; ni++)
  {
    // Read left and right images from file
    imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
    imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);
    double tframe = vTimestamps[ni];

    if(imLeft.empty())
    {
      cerr << endl << "Failed to load image at: "
           << string(vstrImageLeft[ni]) << endl;
      return 1;
    }

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

    // Pass the images to the SLAM system
    SLAM.TrackObject(imLeft,imRight,tframe);

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

    double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    vTimesTrack[ni]=ttrack;

    // Wait to load the next frame
    double T=0;
    if(ni<nImages-1)
      T = vTimestamps[ni+1]-tframe;
    else if(ni>0)
      T = tframe-vTimestamps[ni-1];

    if(ttrack<T)
      // Note: I changed this since the KITTI tracking dataset does not contain
      // timestamp information, so just play all the stamps in constant time spacing
      usleep((T-ttrack)*5e4);
  }

  // Stop all threads
  SLAM.Shutdown();

  // Tracking time statistics
  sort(vTimesTrack.begin(),vTimesTrack.end());
  float totaltime = 0;
  for(int ni=0; ni<nImages; ni++)
  {
    totaltime+=vTimesTrack[ni];
  }
  cout << "-------" << endl << endl;
  cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
  cout << "mean tracking time: " << totaltime/nImages << endl;

  // Save camera trajectory
  SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

  return 0;
}

bool has_suffix(const std::string &str, const std::string &suffix)
{
  return str.size() >= suffix.size() &&
         str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

void LoadImages(const string &strPathToSeqLeft, const string &strPathToSeqRight, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
  DIR *dir;
  struct dirent *ent;

  vector<string> vstrImageName;

  if ((dir = opendir (strPathToSeqLeft.c_str())) != nullptr) {
    while ((ent = readdir (dir)) != nullptr) {
      string fileName = ent -> d_name;
      if (has_suffix(fileName, ".png")) {
        vstrImageName.emplace_back(ent->d_name);
      }
    }
    closedir (dir);
  } else {
    cerr << "Failed to read image files in the directory" << endl;
    exit(-1);
  }

  std::sort(vstrImageName.begin(), vstrImageName.end());

  for (const string &imageName : vstrImageName) {
    vstrImageLeft.emplace_back(strPathToSeqLeft + "/" + imageName);
    vstrImageRight.emplace_back(strPathToSeqRight + "/" + imageName);

    stringstream parser;
    double timeStamp;
    parser << imageName.substr(0, imageName.size() - 4);
    parser >> timeStamp;
    vTimestamps.emplace_back(timeStamp);
  }
}
