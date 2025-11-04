#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

using namespace std;
using namespace cv;

String vids[4] = {"first.mp4", "second.mp4", "third.mp4", "fourth.mp4"};
int N;

//Trackbar
int hueMin1, hueMin2, hueMax1, hueMax2, satMin, satMax, valMin, valMax;
const int HueMax = 180;
const int satValMax = 255;

int main(){

    cin >> N;

    VideoCapture cap(vids[N-1]);
    //VideoCapture cap(0);

    namedWindow("trackbar", WINDOW_AUTOSIZE);
    
    createTrackbar("Hue Min1", "trackbar", &hueMin1, HueMax);
    createTrackbar("Hue Max1", "trackbar", &hueMax1, HueMax);
    createTrackbar("Hue Min2", "trackbar", &hueMin2, HueMax);
    createTrackbar("Hue Max2", "trackbar", &hueMax2, HueMax);
    createTrackbar("Sat Min", "trackbar", &satMin, satValMax);
    createTrackbar("Sat Max", "trackbar", &satMax, satValMax);
    createTrackbar("Val Min", "trackbar", &valMin, satValMax);
    createTrackbar("Val Max", "trackbar", &valMax, satValMax);

    while (true){
        Mat frame, framehsv, maskHSV1, maskHSV2, maskHSV, resultHSV;

        if(!cap.read(frame)){
            break;
        }

        //UBAH
        cvtColor(frame, framehsv, COLOR_BGR2HSV);

        Scalar minHSV1 = Scalar(hueMin1, satMin, valMin);
        Scalar maxHSV1 = Scalar(hueMax1, satMax, valMax);
        inRange(framehsv, minHSV1, maxHSV1, maskHSV1);

        Scalar minHSV2 = Scalar(hueMin2, satMin, valMin);
        Scalar maxHSV2 = Scalar(hueMax2, satMax, valMax);
        inRange(framehsv, minHSV2, maxHSV2, maskHSV2);

        bitwise_or(maskHSV1, maskHSV2, maskHSV);

        bitwise_and(frame, frame, resultHSV, maskHSV);

        imshow("hasil", resultHSV);
        imshow("video", frame);
        if(waitKey(30) == 'a'){
            break;
        }
    }
    waitKey(0);
    destroyAllWindows();
}
