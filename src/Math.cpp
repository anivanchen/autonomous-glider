#include <iostream>
using namespace std;
#include <math.h>
float EARTHEQRADIUS = 6378137.0;
float EARTHPRADIUS = 6356752.3142;

int main() {
    float latitude = 18.91234654;
    float longitude = -19.02464236;
    float x = EARTHEQRADIUS * cos(latitude) * cos(longitude);
    float y = EARTHEQRADIUS * cos(latitude) * sin(longitude);
    float z = EARTHEQRADIUS * sin(latitude);

    string latString = std::to_string(latitude);
    string longString = std::to_string(longitude);
    string xString = std::to_string(x);
    string yString = std::to_string(y);
    string zString = std::to_string(z);

    cout << "Latitude: " + latString + "\n";
    cout << "Longitude:" + longString + "\n";
    cout << "XCoordinate: " + xString + "\n";
    cout << "YCoordinate: " + yString + "\n";
    cout << "ZCoordinate: " + zString + "\n";


    //return 0;
}