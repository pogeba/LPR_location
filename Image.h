#ifndef IMAGE
#define IMAGE
//#include "cv.h"
//#include<highgui.h>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
//using namespace cv;
class Image
{
public:
    Image(void);
    ~Image(void);
    static void cloneImage(IplImage *src,IplImage*dst);
    static void ZerosImage(IplImage * src);
    static void dilateImage(IplImage* src, IplImage *dst);
    static void erodeImage(IplImage* src,IplImage*dst);
    static void rgb2gray(IplImage* src, IplImage* dst);
    static void gray2bw(IplImage* src, IplImage* dst,unsigned char value);
};
#endif // IMAGE

