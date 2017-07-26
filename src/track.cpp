#include "track.h"
#include <sstream>      //stringstream
#include <algorithm>    //minmax_element

//
// ThrottleMapping class definition implementation.
//
ThrottleMapping::ThrottleMapping() {}
ThrottleMapping::~ThrottleMapping() {}


void ThrottleMapping::ReadTrackData(string filepath)
{
  ifstream input_stream(filepath);
  string buffer;
  int rows = 0;
  vector<double> line_values;       				// xy data pairs
  
  if (input_stream.is_open()){
	  
	while(!input_stream.eof()){
		// read line data
		getline(input_stream,buffer,'\n');
		if(buffer == "EOF") break;

		// parsing line data - skip header
		if(rows > 0)
		{
			// parse data to vector
			stringstream ss(buffer);               // streaming data
			string line_value;                     // string data split by ','
			while(getline(ss, line_value, ','))
			{
				double num = stod(line_value);
				line_values.push_back(num);
				//std::cout << num << std::endl;
			}
		}
		// record indexing
		rows = rows + 1;
		//if(rows > 100) break;	
	}  
  }
  else{
	cout << "Can't open file: Check file path " << endl;
  }
	
  input_stream.close();	
  // extract x,y data points
  for(int i=0; i<line_values.size();i+=3)
  {
	  x_values.push_back(line_values[i]);
	  y_values.push_back(line_values[i+1]);
	  v_values.push_back(line_values[i+2]);	  
  }
}// end ReadTrackData
	
		
int ThrottleMapping::GetLocation(double x, double y){
   vector<double> dist;	
   for(int i=0;i<x_values.size();i++){
	 double dx = x_values[i] - x;
	 double dy = y_values[i] - y;
	 double r2 =  dx*dx + dy*dy; 
	 dist.push_back(r2); 	   
   }	
   
   // get closest match
   auto mm = minmax_element(dist.begin(), dist.end());
   int idx_min = distance(dist.begin(), mm.first);
   
   return idx_min;
   
}

// get the waypoints from the given index onward
vector<vector<double>> ThrottleMapping::GetWayPoints(int index, int n_points){
  vector<double> ptsx;
  vector<double> ptsy;
  vector<double> ptsv;
  int n = x_values.size();
  for(int i=0;i<n_points;i++){
	// python to cpp: (n + (a%n)) % n  (cyclic data)
	int a = index+i;
	int data_i = (n + (a%n)) % n;
	ptsx.push_back(x_values[data_i]);
	ptsy.push_back(y_values[data_i]);
	ptsv.push_back(v_values[data_i]);    	  
  }
  vector< vector<double> > xyv_waypoints;
  xyv_waypoints.push_back(ptsx);
  xyv_waypoints.push_back(ptsy);
  xyv_waypoints.push_back(ptsv);
  
  return xyv_waypoints;
}

double ThrottleMapping::GetTrackVelocity(int index){
  return v_values[index];
}
 
	


