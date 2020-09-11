#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;
string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}
int main(int argc, char** argv )
{
    /*
    if ( argc != 2 )
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }
    */
    Mat image;
    image = imread( "/home/ljj/code/APA/pathplannerOnGit/src/path_planner/maps/map_large.png", -1 );
    

    for (int i = 0; i < image.cols; i++) {
        for (int j = 0; j < image.rows; j++) {
            Vec3b &intensity = image.at<Vec3b>(j, i);
            std::cout << "ch1 : " << static_cast<unsigned>(intensity.val[0]) << "ch2 : " << static_cast<unsigned>(intensity.val[1]) << "ch3 : " << static_cast<unsigned>(intensity.val[2]) << std::endl;
        }
    }

    // set img and obstacles
    int row = 25 / 0.2;
    int col = 30 / 0.2;
    Mat myimg(row, col, CV_8UC3, cv::Scalar(255,255,255));
    for (int i = 20; i < 50; i++)
    {
        Vec3b &intensity = myimg.at<Vec3b>(i, 50);
        intensity.val[0]=0;
        intensity.val[1]=0;
        intensity.val[2]=0;

    }
    for (int i = 20; i < 50; i++)
    {
        Vec3b &intensity = myimg.at<Vec3b>(i, 65);
        intensity.val[0]=0;
        intensity.val[1]=0;
        intensity.val[2]=0;

    }
    for (int i = 50; i < 65; i++)
    {
        Vec3b &intensity = myimg.at<Vec3b>(21, i);
        intensity.val[0]=0;
        intensity.val[1]=0;
        intensity.val[2]=0;

    }

    int type = myimg.type();
    string ty =  type2str( type );

    imwrite("/home/ljj/code/APA/pathplannerOnGit/src/path_planner/maps/jj_map_parking.png",myimg);
    // show img
    printf("Matrix: %s %dx%d \n", ty.c_str(), myimg.cols, myimg.rows );
    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }
    namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", myimg);

    waitKey(0);

    return 0;
}