#include "include/common_include.h"
#include "include/Image.h"
#include "include/index.h"
#include "include/EKF.h"

using namespace std;

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        cout << "Input Video Path!" << endl;
    }

    cv::Mat currImage, currImageGray, lastImage;
    cv::VideoCapture capture;
    vector<cv::Mat> imageStore;
    vector<int> a;
    capture.open(argv[1]);
    EKF ekf;
    Image image;
    int count = 0;

    if (capture.isOpened())
    {

        while (true)
        {

            capture >> currImage;
            if (currImage.data == nullptr)
                break;

            Mat currImageProcessed;

            image.processImage(currImage, currImageProcessed);

            // Connect Image
            Mat connectImage(currImage.rows, 2 * currImage.cols, CV_8UC3);
            currImage.copyTo(connectImage(Rect(0, 0, currImage.cols, currImage.rows)));
            currImageProcessed.copyTo(connectImage(Rect(currImage.cols, 0, currImage.cols, currImage.rows)));
            if (connectImage.cols > 1920)
                resize(connectImage, connectImage, Size(connectImage.cols/2, connectImage.rows/2));
            cv::namedWindow("Camera");
            cv::imshow("Camera", connectImage);

            cv::waitKey(WAIT_TIME);

        }

    }
    else
    {
        cout << "ERROR: camera cannot open" << endl;
        return 0;
    }

    return 0;
}
