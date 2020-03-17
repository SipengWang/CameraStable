#ifndef IMAGE_H
#define IMAGE_H



#include "common_include.h"
#include "EKF.h"

using namespace std;
using namespace cv;

class Image
{
public:

    void imagePushBack(Mat imageGray, Mat imageColor);
    void calTrajectory();
    void trajectoryPushBack(Eigen::Vector3f trajectory);
    void transform();
    void fillUndefinedRegion(Mat& imageOutput);
    void stitcherImage();
    void processImage(Mat imageInput, Mat& imageOutput);
    Image();
    ~Image();


private:
    vector<Mat> imageGrayStore, imageColorStore;
    vector<Eigen::Vector3f> trajectory, trans;
    Eigen::Vector3f trajectoryAccu{0, 0, 0};
    Mat T, lastT;
    EKF ekf;
    Mat imageProcessed;
    ofstream out_trajectory, out_trajectory_smooth, out_transform_index;
    int count = 0;

};

Image::Image()
{
    out_trajectory.open("trajectory.txt");
    out_trajectory_smooth.open(("trajectory_smooth.txt"));
    out_transform_index.open("transform_index.txt");
}
Image::~Image()
{
    out_trajectory.close();
    out_trajectory_smooth.close();
    out_transform_index.close();
}

void Image::imagePushBack(Mat imageGray, Mat imageColor)
{
    imageGrayStore.push_back(imageGray);
    imageColorStore.push_back(imageColor);
}

void Image::calTrajectory()
{
    Mat currImage, refImage;
    vector<Point2f> currCorner, refCorner, currGoodCorner, refGoodCorner;
    vector<uchar> status;
    vector<float> err;
    currImage = imageGrayStore[imageGrayStore.size() - 1];
    refImage = imageGrayStore[std::max(int(imageGrayStore.size() - 1 - REFSTEP), 0)];

    goodFeaturesToTrack(refImage, refCorner, 200, 0.01, 30);
    calcOpticalFlowPyrLK(refImage, currImage, refCorner, currCorner, status, err);
    for (int i = 0; i < status.size(); i++)
    {
        if (status[i])
        {
            refGoodCorner.push_back(refCorner[i]);
            currGoodCorner.push_back(currCorner[i]);
        }
    }
    cout << "Good Corner Number: " << refGoodCorner.size() << endl;
    T = estimateRigidTransform(refGoodCorner, currGoodCorner, false);
//    T = estimateRigidTransform(currGoodCorner, refGoodCorner, false);
//    Mat T_ = findHomography(refGoodCorner, currGoodCorner, CV_RANSAC);
    if (T.data == NULL)
        lastT.copyTo(T);
    T.copyTo(lastT);

    auto dx = T.at<double>(0, 2);
    auto dy = T.at<double>(1, 2);
    auto da = atan2(T.at<double>(1, 0), T.at<double>(0, 0));
    trans.push_back(Vector3f(dx, dy, da));
    trajectoryAccu += Vector3f(dx, dy, da);
    trajectoryPushBack(trajectoryAccu);

    // filter
    Eigen::Vector3f filtered = ekf.filter(trajectory, FILTER_SMOOTH);

    // write file
    if (WRITE_FILE)
    {
        out_transform_index << count << " " << dx << " " << dy << " " << da << endl;
        out_trajectory << count << " " << trajectoryAccu[0] << " " <<
                          trajectoryAccu[1] << " " << trajectoryAccu[2] << endl;
        out_trajectory_smooth << count << " " << filtered[0] << " " << filtered[1] << " " << filtered[2] << endl;
    }

    dx = (filtered-trajectoryAccu)(0, 0) + dx;
    dy = (filtered-trajectoryAccu)(1, 0) + dy;
    da = (filtered-trajectoryAccu)(2, 0) + da;
    cout << "dx: " << dx << " dy: " << dy << " da: " << da << endl;

    T.at<double>(0, 0) = cos(da);
    T.at<double>(0, 1) = -sin(da);
    T.at<double>(1, 0) = sin(da);
    T.at<double>(1, 1) = cos(da);
    T.at<double>(0, 2) = dx;
    T.at<double>(1, 2) = dy;
}

void Image::trajectoryPushBack(Eigen::Vector3f trajectory)
{
    this->trajectory.push_back(trajectory);
}

void Image::transform()
{
    // warp affine transform
    Mat temp;

    /*
     * At first, I use such expression:
     * warpAffine(imageStore.back(), imageStore.back(), T, imageStore.back().size());
     * I wanted to get the last image in imagestore, affine it and replace the last image
     * with the affined image. But it did not work. It seems that the last image did not
     * changed at all. And the transform vector [dx, dy, da] is very small (about 1e-16)
     * When I use a temp as following, it works.
    */

    warpAffine(imageGrayStore.back(), imageProcessed, T, imageGrayStore.back().size());

    lastT = T;
}

void Image::fillUndefinedRegion(Mat& imageOutput)
{
      /*
       * warp and fill
       * Input: refImage(color), currImage(color), T
       * Output: filledImage(color)
       */
    Mat refImage = imageColorStore[max(int(imageColorStore.size() - 1 - REFSTEP), 0)];
    Mat currImage = imageColorStore[imageColorStore.size() - 1];
    warpAffine(currImage, imageOutput, T, currImage.size());

    for (int i = 0; i < currImage.rows; i++)
    {
        for ( int j = 0; j < currImage.cols; j++)
        {
            if (imageOutput.at<Vec3b>(i, j)[0] == 0 &&
                imageOutput.at<Vec3b>(i, j)[1] == 0 &&
                imageOutput.at<Vec3b>(i, j)[2] == 0)
                imageOutput.at<Vec3b>(i, j) = refImage.at<Vec3b>(i, j);
//            else
//            {
//                double temp_array[4]{double(i)/double(currImage.rows/2),
//                                     double(j)/double(currImage.cols/2),
//                                     double(currImage.rows - i)/double(currImage.rows/2),
//                                     double(currImage.cols - j)/double(currImage.cols/2)};
//                double ratio = *min_element(temp_array, temp_array + 3);
//                imageOutput.at<Vec3b>(i, j) = ratio * imageOutput.at<Vec3b>(i, j) +
//                                              (1 - ratio) * currImage.at<Vec3b>(i, j);
//            }
        }
    }


//    int vertBord = 20;
//    int horiBord = 20 * imageOutput.cols / imageOutput.rows;
//    imageOutput = imageOutput(Range(vertBord, imageOutput.rows - vertBord),
//                              Range(horiBord, imageOutput.cols - horiBord));
//    resize(imageOutput, imageOutput, currImage.size());
}

void Image::stitcherImage()
{
    // test: stitch 3 image
    vector<Mat>::iterator ite = imageColorStore.end();
    vector<Eigen::Vector3f>::iterator iteT = trans.end();
    if (imageColorStore.size() > 3)
    {
        Mat image1 = *(ite - 3), image2 = *(ite - 2), image3 = *(ite - 1);
        Eigen::Vector3f T1 = *(iteT - 2), T2 = *(iteT - 1);
    }
}

void Image::processImage(Mat imageInput, Mat& imageOutput)
{
    cout << "Frame Number: " << count++ << endl;
    // to gray and equalize
    Mat imageInputGray;
    cvtColor(imageInput, imageInputGray, CV_BGR2GRAY);
//    equalizeHist(imageInputGray, imageInputGray);

    // store gray and color image.
    imagePushBack(imageInputGray, imageInput);

    calTrajectory();

    // fill undefined region
    fillUndefinedRegion(imageOutput);

    if (imageGrayStore.size() > IMAGESTORE_MAX_SIZE)
    {
        imageGrayStore.erase(imageGrayStore.begin());
        imageColorStore.erase(imageColorStore.begin());
    }

    stitcherImage();

}

#endif
