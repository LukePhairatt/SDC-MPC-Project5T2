#ifndef TRACK_H
#define TRACK_H

#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

class ThrottleMapping{
 public:
  ThrottleMapping();

  virtual ~ThrottleMapping();

  void ReadTrackData(string filepath);
  int GetLocation(double x, double y);
  vector<vector<double>> GetWayPoints(int index, int n_points);
  double GetTrackVelocity(int index);
  
  // data members
  vector<double> x_values;
  vector<double> y_values;
  vector<double> v_values;
  
};

#endif /* TRACK_H */
