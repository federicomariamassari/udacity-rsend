#include "refresh_rates.h"

using namespace std;


void print_header()
{
  vector<string> header = {"TIMESTAMP", "SENSOR", "MEASUREMENT", "INTERPOLATION (*)", "EXTRAPOLATION (^)", 
  "FILL-FORWARD"};

  cout << endl << string(120, '*') << endl;
  print_line(header);
  cout << string(120, '*') << endl;
}

template<typename T>
void print_line(vector<T> &line)
{
  for (size_t i = 0; i < line.size(); ++i)
  {
    switch(i)
    {
      case 0:
        cout << setw(18);
        break;

      case 1:
        cout << setw(10);
        break;

      default:
        cout << setw(23);
    }

    cout << left << line[i];
  }

  cout << endl;
}

string make_coordinate_pair(float &x, float &y)
{
  return "(x=" + to_string(x).substr(0, 5) + ", y=" + to_string(y).substr(0, 5) + ")";
}

void fill_forward(vector<pair<float, float>> &vec, pair<float, float> &result)
{
  result.first = vec[vec.size() - 1].first;
  result.second = vec[vec.size() - 1].second;
}

void extrapolate(vector<pair<float, float>> &vec, vector<uint64_t> &timestamps, float velocity, 
  pair<float, float> &result)
{
  uint64_t dt = timestamps[timestamps.size()-1] - timestamps[timestamps.size()-2];

  float dt_sec = dt / 1e6;  // Convert time to seconds

  result.first = vec[vec.size()-1].first + velocity * dt_sec;
  result.second = vec[vec.size()-1].second + velocity * dt_sec;
}

void interpolate(vector<pair<float, float>> &vec, vector<uint64_t> &timestamps, pair<float, float> &result)
{
  float x0 = vec[vec.size() - 2].first;
  float y0 = vec[vec.size() - 2].second;

  float x1 = vec[vec.size() - 1].first;
  float y1 = vec[vec.size() - 1].second;

  float dt0 = timestamps[timestamps.size()-2] - timestamps[timestamps.size()-3];
  float dt1 = timestamps[timestamps.size()-1] - timestamps[timestamps.size()-3];

  // Interpolated coordinates (https://en.wikipedia.org/wiki/Linear_interpolation)
  float x = x0 + (x1 - x0) * (dt0 / dt1);
  float y = y0 + (y1 - y0) * (dt0 / dt1);

  result.first = x;
  result.second = y;
}


int main(int argc, char** argv)
{
  // To hold simulated timestamps of incoming sensor measurements
  vector<uint64_t> timestamps;

  // To hold coordinates (x, y) of simulated sensor measurements
  pair<float, float> latest_pos, interpolated, extrapolated, filled_forward;
  vector<pair<float, float>> positions;

  float velocity = 0.1;  // Calibrated on simulated data distribution

  // Generate random position increment data (px, py)
  random_device rd;
  mt19937 gen(rd());
  uniform_real_distribution<float> dist(0, 0.5);

  cout << endl << "DATA IMPUTATION EXAMPLE" << endl << endl;
  cout << "(*): Interpolation data delayed one time period as two endpoints are required." << endl;
  cout << "(^): Extrapolation data model: meas_(t + delta_t) = meas_(t) + " << velocity << " * delta_t" << endl;

  print_header();

  for (int i = 0; i < 20; ++i)
  {
    // Simulate timestamp of incoming sensor measurements (suggested by Udacity GPT)
    auto measurement_in = chrono::high_resolution_clock::now().time_since_epoch();
    auto time_us = chrono::duration_cast<chrono::microseconds>(measurement_in).count();
    timestamps.push_back(time_us);

    // Will hold data to print
    vector<string> line;
    line.push_back(to_string(time_us));

    switch (i % 2)  // Alternate between LiDAR and radar
    {
      case (0):  // With LiDAR, simply push back the simulated measure

        line.push_back("L");

        if (positions.size() > 0)
        {
          // Add random increment to previous position
          pair<float, float> previous_pos = positions[positions.size()-1];
          latest_pos = make_pair(previous_pos.first + dist(gen), previous_pos.second + dist(gen));
        }

        else
          // Initial random position
          latest_pos = make_pair(dist(gen), dist(gen));

        line.push_back(make_coordinate_pair(latest_pos.first, latest_pos.second));
        positions.push_back(latest_pos);

        print_line(line);
        break;

      case (1):  // With radar, impute missing data for LiDAR

        line.push_back("R");

        // Skip measurement slot (data not yet available)
        line.push_back("N/A");

        if (positions.size() > 1)  // Interpolation requires two endpoints
        {
          // Interpolate based on previous two endpoints (delayed estimate)
          interpolate(positions, timestamps, interpolated);
          line.push_back(make_coordinate_pair(interpolated.first, interpolated.second));
        }

        else
          line.push_back("N/A");  // Not enough data to interpolate

        // Extrapolate future position on assumed Constant Velocity Model (CVM)
        extrapolate(positions, timestamps, velocity, extrapolated);
        line.push_back(make_coordinate_pair(extrapolated.first, extrapolated.second));

        fill_forward(positions, filled_forward);
        line.push_back(make_coordinate_pair(filled_forward.first, filled_forward.second));

        print_line(line);
        break;
    }

    this_thread::sleep_for(chrono::milliseconds(1000));  // Simulate measurement delay
  }
}