#include "globalmap.h"

using namespace cv;
using namespace std;
GlobalMap::GlobalMap()
{
    mapImage = cv::Mat(Size(WIDTH, HEIGHT), CV_8UC1);
    isValid = false;
}
