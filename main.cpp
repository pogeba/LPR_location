#include <QCoreApplication>
#include <cstdio>
#include <cstring>
#include <iostream>
#include<cv.h>
#include<highgui.h>
#include <cmath>
#include "Image.h"
#include "math.h"
//#include <QMessageBox>
#include <QString>
//#include <QMessageBox>
int sp = 0;//堆栈指针
#include <charpartition.h>
#include <charrecognition.h>

#define S(image,x,y) ((uchar*)(image->imageData + image->widthStep*(y)))[(x)]
#define COUNTER 5
#define ROW_THR 20
#define COL_THR 2
#define MODLESIZE 3

#define   FLAG 0
#define   IMAGE_WIDTH 600
#define   IMAGE_HEIGHT 600
#define   MAX_WIDTH 260
#define   MIN_WIDTH 100
#define   MYERROR 1.4
#define M_PI 3.14159265358979323846

#define   NUM 100    //车牌候选区域最大个数
//typedef struct myHist//直方图机构体
//{
//    int size; // 直方块个数
//    int ranges[2];//像素变化范围
//    int bin_max;//最大高度直方块坐标
//    int bin_min;//最小直高度直方块坐标
//    int max_value;//最大高度直方块的值
//    int min_value;//最小高度直方块的值
//    int bin[256];//直方图各方块的像素个数
//}myHist;
//typedef  struct Sta
//{
//    int x; /* X坐标, 通常以为基点*/
//    int y; /* y坐标, 通常以为基点*/
//}Sta;
Sta sta1[IMAGE_WIDTH *IMAGE_HEIGHT];

void push(int x, int y)
{
    sta1[sp].x = x;
    sta1[sp].y = y;
    sp++;
}

void  pop()
{
    sp--;
}
int myDiffProj(IplImage *src, IplImage *dst);
void myCalHist(myHist *histponter, IplImage * src, int size, int *ranges);
int myGetThreshold(IplImage *src);
void myScope(IplImage *scopecopy, int x, int y, int *pionter);
int myRemoveBorder(IplImage *src, int *post);
CharPartition charpartion;
CharRecognition charrecognition;
using namespace std;
using namespace cv;
const int Train = 109;
int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    const QString &srcImagePath="/home/wangjin/14.wmv";
  //  const QString &srcImagePath="/home/wangjin/QTProject/images/4.wmv";
    IplImage* srcImage = NULL,*image = NULL,*bwImage = NULL;
    IplImage *pImgSrc = NULL;
    IplImage *pImg8u = NULL;            //灰度图
    IplImage *pImg8uSmooth = NULL;     //中值滤波后的图
    IplImage *pImgCanny = NULL;        //边缘检测
    IplImage *pImgCanny1 = NULL;
    IplImage *pImgCanny2 = NULL;
    IplImage *mycopy = NULL;          //标记图像
    IplImage *pImgdst = NULL;     //形态学操作后图像
    IplImage *pImgResize = NULL; //粗定位车牌图像
    IplImage *pImgResize1 = NULL; //精确定位车牌图像
    IplImage *pImgdst1 = NULL;//去除上下边框
    IplImage *pImgThr = NULL;//二值化车牌
    CvPoint2D32f pt[4];
    char imageAName[12] = "1_dst.jpg";
    char imageBName[12] = "2_dst.jpg";
    char imageCName[12] = "3_dst.jpg";
    char imageDName[12] = "4_dst.jpg";
    char imageEName[12] = "5_dst.jpg";
    char imageFName[12] = "6_dst.jpg";
    char imageGName[12] = "7_dst.jpg";
//    cvNamedWindow("srcImage",1);
//    cvNamedWindow("bwImage1",1);
//      cvNamedWindow("bwImage2",1);
//        cvNamedWindow("bwImage3",1);
 //         cvNamedWindow("dst",1);
    //cvShowImage("srcImage",srcImage);

    namedWindow("src");
    namedWindow("foreground");


    int imageWidth,imageHeight;
    int maxDif = 50;
    //找到蓝色区域
    int i= 0,j = 0;
    unsigned char * pPixel = NULL;
    unsigned char   pixelR = 0,pixelG = 0,pixelB = 0;
    unsigned char R = 28,G = 63, B = 138;

    double length,area,rectArea;
    double rectDegree = 0.0; //矩形度
    double long2Short = 1.0; //体态比
    //计算边界序列的参数 长度 面积 矩形 最小矩形
    //并输出每个边界的参数
    CvRect rect;
    CvBox2D box;
    int imageCnt = 1;
    double axisLong = 0.0, axisShort = 0.0;
    double temp;
    Ptr<BackgroundSubtractorMOG2> mog = createBackgroundSubtractorMOG2(100, 25, false);
    //bgsubtractor->setVarThreshold(20);
    Mat foreGround;
    Mat backGround;
    int trainCounter = 0;
    bool dynamicDetect = true;
    int position_err1 = 0;
    int position_err2= 0;
    clock_t time;
    clock_t time1;
    bool first_frame=true;
    bool second_frame=false;

    double height=0.7;//
    double angle=double((50/180)*M_PI);
    double focus=0.04;
    int speed=0;
    double distance1=0;
    double distance2=0;
    double distance=0;
    double yc=0;
    double yp=0;

    double messure_time=0.0;
    double messure_time2=0.0;
  //  /**************************************
    std::string ImagePath;
     ImagePath = string((const char *)srcImagePath.toLocal8Bit());
    VideoCapture capture;
    capture.open(ImagePath);
    time1 = clock();

    double rate = capture.get(CV_CAP_PROP_FPS);//获取视频文件的帧率
    int delay = cvRound(1000.000 / rate);
    if (capture.isOpened()) {
       while(true){
        cv::Mat src;
        capture >> src;//读出每一帧的图像


//         cout<<"src_width"<<src.cols<<endl;
//          cout<<"src_width"<<src.rows<<endl;
  //     imshow("src",src);


        if (dynamicDetect)
        {
            mog->apply(src, foreGround, 0.005);
            //图像处理过程
            medianBlur(foreGround, foreGround, 3);
            dilate(foreGround, foreGround, Mat(), Point(-1, -1), 3);
            erode(foreGround, foreGround, Mat(), Point(-1, -1), 6);
            dilate(foreGround, foreGround, Mat(), Point(-1, -1), 3);
            imshow("foreground", foreGround);

            int x0 = src.cols *0;
            int x1 = src.cols ;
            int y0 = src.rows *5/8;
            int y1 = src.rows *5/8;
            //画线的坐标，起始坐标和终止坐标
            cv::Point p0 = cv::Point(x0,y0);
            cv::Point p1 = cv::Point(x1, y1);

            int x2 = src.cols *0;
            int x3 = src.cols ;
            int y2 = src.rows *7/8;
            int y3 = src.rows *7/8;
            //画线的坐标，起始坐标和终止坐标
            cv::Point p2 = cv::Point(x2,y2);
            cv::Point p3 = cv::Point(x3, y3);

            cv::line(src, p0, p1, cv::Scalar(0, 0, 255), 3, 4);
            cv::line(src, p2, p3, cv::Scalar(0, 0, 255), 3, 4);
       //     cout<<"y2:"<<y2<<endl;
       //     cout<<"sin(angle):"<<sin(3.14/90)<<endl;
            distance2 = height*(focus*cos(3.14/55)+(src.rows *3/8)*0.000002*sin(3.14/55))/(focus*sin(3.14/55)-(src.rows *3/8)*0.000002*cos(3.14/55));
            distance1 = height*(focus*cos(3.14/55)+(src.rows *1/8)*0.000002*sin(3.14/55))/(focus*sin(3.14/55)-(src.rows *1/8)*0.000002*cos(3.14/55));
     //       cout<<"distance1:"<<distance1<<endl;
            distance=distance2-distance1;
     //       cout<<"distance2:"<<distance2<<endl;
      //      cout<<"distance:"<<distance<<endl;


            if (trainCounter < Train)//训练期间所得结果为不准确结果，不应作为后续
            {
                Mat findc;
                foreGround.copyTo(findc);
                vector<vector<Point> > contours;
                cv::findContours(findc, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

                //targets.clear();
                const int maxArea = 800;
                size_t s = contours.size();
                for (size_t i = 0; i < s; i++)
                {
                    double area = abs(contourArea(contours[i]));
                    if (area > maxArea)
                    {
                        Rect mr = boundingRect(Mat(contours[i]));
                        rectangle(src, mr, Scalar(0, 0, 255), 2, 8, 0);
                        //targets.push_back(mr);
                    }
                }
                //string text;
                char text[50];
                sprintf(text, "background training -%d- ...", trainCounter);
                putText(src, text, Point(50, 50), 3, 1, Scalar(0, 255, 255), 2, 8, false);
                //delete[] text;
           //          cout<<"123"<<endl;

            }else
            {
                Mat findc;
                Mat findce;
                src.copyTo(findc);
                foreGround.copyTo(findce);
                vector<vector<Point> > contours;
         if(first_frame){


                IplImage imgTmp = findc;
                srcImage = cvCloneImage(&imgTmp);



                cv::findContours(findce, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

                //targets.clear();
                const int maxArea = 20000;
                size_t s = contours.size();
                for (size_t i = 0; i < s; i++)
                {
                    double area = abs(contourArea(contours[i]));
                    if (area > maxArea)
                    {
                        Rect mr = boundingRect(Mat(contours[i]));
                        rectangle(src, mr, Scalar(0, 0, 255), 2, 8, 0);
                        //targets.push_back(mr);
                        position_err1=abs(mr.br().y-p0.y);
//                        position_err2=abs(mr.br().y-p2.y);
//                        char text[50];
//                        sprintf(text, "error messure is -%d- ...", position_err1);
//                        putText(src, text, Point(450, 50), 3, 1, Scalar(0, 255, 255), 2, 8, false);

//                        cout<<"position_err1:  "<<position_err1<<endl;
//                        cout<<"position_err2:  "<<position_err2<<endl;
                    }
                }
                if(15<position_err1<=20)//当汽车与虚拟线圈的距离在15至20区间时，截取一帧图片用于识别车牌识别
                {
//                    time = clock();
//                 messure_time2=(double)(clock() - time1)/CLOCKS_PER_SEC;
//                 cout<<"the car meet the line at:"<<messure_time2<<"s"<<endl;




                          if (!srcImage) {
                cout<<"123"<<endl;
           //     QMessageBox::warning(NULL, "不能打开图片", "不能打开图片，请检查正确的路径");
                return NULL;
            }
         //     ***************************************/
        //    while ((srcImage = cvLoadImage(srcImagePath.toLatin1().data(),1)) != NULL)
        //    {
         //   srcImage = cvLoadImage(srcImagePath.toLatin1().data(),1);
        //        cvShowImage("srcImage",srcImage);
        //         cvWaitKey(0);
             //   cout<<imageName<<": "<<endl;
                imageWidth = srcImage->width;
                imageHeight = srcImage->height;


                pImg8u = cvCreateImage(cvSize(imageWidth,imageHeight),8,1);

                cvCvtColor(srcImage, pImg8u, CV_RGB2GRAY);

                //image = cvCloneImage(srcImage);
            //    Image::cloneImage(srcImage,image);

                bwImage = cvCreateImage(cvGetSize(srcImage),srcImage->depth,1);
                //cvZero(bwImage);
                Image::ZerosImage(bwImage);


                for (i = 0; i< imageHeight;i++)
                {
                    for (j = 0;j<imageWidth;j++)
                    {
                        pPixel = (unsigned char*)srcImage->imageData + i*srcImage->widthStep + j*3;
                        pixelB = pPixel[0];
                        pixelG = pPixel[1];
                        pixelR = pPixel[2];

                        if (abs(pixelB - B) < maxDif && abs(pixelG - G)< maxDif && abs(pixelR - R)< maxDif)
                        {
                            *((unsigned char*)bwImage->imageData + i*bwImage->widthStep + j) = 255;
                        }else {
                            *((unsigned char*)bwImage->imageData + i*bwImage->widthStep + j) = 0;
                        }
                    }
                }
            //    cvShowImage("bwImage1",bwImage);
          //      cvSaveImage(imageBwName,bwImage);
            //     cvWaitKey(0);
            //   imshow("after", bwImage);
                //膨胀
                //cvDilate(bwImage,bwImage,0,3);
                Image::dilateImage(bwImage,bwImage);
                Image::dilateImage(bwImage,bwImage);
                Image::dilateImage(bwImage,bwImage);

                //cvErode (bwImage,bwImage,0,3);
                Image::erodeImage(bwImage,bwImage);
                Image::erodeImage(bwImage,bwImage);
                Image::erodeImage(bwImage,bwImage);

          //      cvShowImage("bwImage2",bwImage);
          //      cvWaitKey(0);

                //新图，将轮廓绘制到dst
                IplImage *dst = cvCreateImage(cvGetSize(srcImage),8,3);
                //dst = cvCloneImage(srcImage);//赋值为0
                Image::cloneImage(srcImage,dst);
                //寻找轮廓
                CvMemStorage *storage = cvCreateMemStorage(0);
                CvSeq * seq = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint), storage);
                CvSeq * tempSeq = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint), storage);
                int cnt = cvFindContours(bwImage,storage,&seq);//返回轮廓的数目
          //      cout<<"number of contours   "<<cnt<<endl;
           //     cvShowImage("bwImage3",bwImage);    //难道使用cvFindContours会改变源图像？需要实现保存一下
          //      cvWaitKey(0);
                for (tempSeq = seq;tempSeq != NULL; tempSeq = tempSeq->h_next)
                {

                    length = cvArcLength(tempSeq);
                    area =  cvContourArea(tempSeq);
        //            cout<<"area:"<<area<<endl;
                    //筛选面积比较大的区域
                    if (area > 1400 && area < 50000)
                    {

                        //cout<<"Points:  "<<tempSeq->total<<endl;
                        //外接矩形
                        rect = cvBoundingRect(tempSeq,1);
                        //绘制轮廓和外接矩形
                        //cvDrawContours(dst,tempSeq,CV_RGB(255,0,0),CV_RGB(255,0,0),0);
                        //cvRectangleR(dst,rect,CV_RGB(0,255,0));
                        //cvShowImage("dst",dst);

                        //绘制外接最小矩形
                    //    CvPoint2D32f pt[4];
                        box = cvMinAreaRect2(tempSeq,0);
                        cvBoxPoints(box,pt);
                        //下面开始分析图形的形状特征
                        //长轴 短轴
                        axisLong = sqrt(pow(pt[1].x -pt[0].x,2) + pow(pt[1].y -pt[0].y,2));
                        axisShort = sqrt(pow(pt[2].x -pt[1].x,2) + pow(pt[2].y -pt[1].y,2));

                        if (axisShort > axisLong)
                        {
                            temp = axisLong;
                            axisLong = axisShort;
                            axisShort= temp;
                        }
                        rectArea = axisLong*axisShort;
                        rectDegree =  area/rectArea;
                        //体态比or长宽比 最下外接矩形的长轴和短轴的比值
                        long2Short = axisLong/axisShort;
               //         cout<<"long2Short:"<<long2Short<<endl;


                        if (long2Short> 2.2 && long2Short < 3.8 && rectDegree > 0.63 && rectArea > 1500 && rectArea <50000)
                        {
                            cout<<"the plate of car is deteced"<<endl;
//                           /* cout<<"Length:  "<<length<<endl;
//                            cout<<"Area  :  "<<area<<endl;
//                            cout<<"long axis ："<<axisLong<<endl;
//                            cout<<"short axis: "<<axisShort<<endl;
//                            cout<<"long2Short: "<<long2Short<<endl;
//                            cout<<"rectArea:  "<<rectArea<<endl;
//                            cout<<"rectDegree:  "<<rectDegree<<endl;*/

//                            for(int i = 0;i<4;++i){
//                                cvLine(dst,cvPointFrom32f(pt[i]),cvPointFrom32f(pt[((i+1)%4)?(i+1):0]),CV_RGB(255,0,0));
//                            }
                        }
                        //cvShowImage("dst",dst);
                        //cvWaitKey();
                    }
                }
        //        cvShowImage("dst",dst);
        //         cvWaitKey(0);
            //    imshow("after", dst);
           //     cvSaveImage(imageDstName,dst);


          //      imageCnt++;
         //       sprintf(imageName,"%d.jpg",imageCnt);
         //       sprintf(imageBwName,"%d_bw.jpg",imageCnt);
         //       sprintf(imageDstName,"%d_dst.jpg",imageCnt);
                cout<<"\n\n";
        //    }
//                cout<<"pt[0]"<<pt[0].x<<","<<pt[0].y<<endl;
//                cout<<"pt[1]"<<pt[1].x<<","<<pt[1].y<<endl;
//                cout<<"pt[2]"<<pt[2].x<<","<<pt[2].y<<endl;
//                cout<<"pt[3]"<<pt[3].x<<","<<pt[3].y<<endl;

           int min_x = 0,min_y=0;
           min_x = pt[1].x;
             min_y = pt[1].y;
           if(min_x>pt[2].x)
               min_x=pt[2].x;
             if(min_y>pt[2].y)
              min_y=pt[2].y;

                      CvRect ROI_rect3;                 //获得图片感兴趣区域
                      ROI_rect3.x = min_x;
                      ROI_rect3.y = min_y;
                      ROI_rect3.width = axisLong;
                      ROI_rect3.height = axisShort;

//                cout<<"ROI_rect3.x"<<ROI_rect3.x<<endl;
//                cout<<" ROI_rect3.y"<< ROI_rect3.y<<endl;
//                cout<<"ROI_rect3.width"<<ROI_rect3.width<<endl;
//                cout<<"ROI_rect3.height"<<ROI_rect3.height<<endl;





                    IplImage *pImg8uROI = NULL;         //感兴趣的图片
                //    cvSetImageROI(pImg8u, ROI_rect[0]);
                //    pImg8uROI = cvCreateImage(cvSize(ROI_rect[0].width, ROI_rect[0].height), IPL_DEPTH_8U, 1);
                    cvSetImageROI(pImg8u, ROI_rect3);
                    pImg8uROI = cvCreateImage(cvSize(ROI_rect3.width, ROI_rect3.height), IPL_DEPTH_8U, 1);
                    cvCopy(pImg8u, pImg8uROI);
                    cvResetImageROI(pImg8u);

                    /* 车牌精确定位 */
                    int nWidth = 409;//(409,90)分别为感兴趣图像的宽度与高度
                    int nHeight = 90;

                    pImgResize = cvCreateImage(cvSize(nWidth, nHeight), IPL_DEPTH_8U, 1);
                    cvResize(pImg8uROI, pImgResize, CV_INTER_LINEAR); //线性插值

                    pImgResize1 = cvCreateImage(cvSize(nWidth, nHeight), IPL_DEPTH_8U, 1);//精确定位部分
                    myDiffProj(pImgResize, pImgResize1);

                    int post[2] = { 0 };//上下边框位置数组

                    myRemoveBorder(pImgResize1, post);

//                    cout<<"post[0]:"<<post[0]<<endl;
//                    cout<<"post[1]:"<<post[1]<<endl;
                    CvRect ROI_rect1;                 //获得图片感兴趣区域
                    ROI_rect1.x = 0;
                    ROI_rect1.y = post[0];
                    ROI_rect1.width = pImgResize1->width;
                    ROI_rect1.height = abs(post[1] - post[0]);

                    pImgdst1 = cvCreateImage(cvSize(ROI_rect1.width, ROI_rect1.height), IPL_DEPTH_8U, 1);
                    cvSetImageROI(pImgResize1, ROI_rect1);
                    cvCopy(pImgResize1, pImgdst1);
                    cvResetImageROI(pImgResize1);

                    pImg8uSmooth = cvCreateImage(cvGetSize(pImgdst1), IPL_DEPTH_8U, 1);
                    pImgThr = cvCreateImage(cvGetSize(pImgdst1), IPL_DEPTH_8U, 1);
                    cvSmooth(pImgdst1, pImg8uSmooth, CV_GAUSSIAN, 3, 0, 0);//高斯滤波

                    int Thres = myGetThreshold(pImg8uSmooth);
                    cvThreshold(pImg8uSmooth, pImgThr, Thres, 255, CV_THRESH_BINARY);

                    //销毁无用图像
                    cvReleaseImage(&srcImage);
                    cvReleaseImage(&pImg8u);
                    cvReleaseImage(&pImg8uROI);
                    cvReleaseImage(&pImg8uSmooth);
                    cvReleaseImage(&pImgCanny);
                    cvReleaseImage(&pImgCanny1);
                    cvReleaseImage(&pImgCanny2);
                    cvReleaseImage(&mycopy);
                    cvReleaseImage(&pImgdst);
                    cvReleaseImage(&pImgResize);
                    cvReleaseImage(&pImgResize1);
                    cvReleaseImage(&pImgdst1);

//             cout<<"123456"<<endl;
                    cvShowImage("pImgThr",pImgThr);
            //            cvWaitKey(0);

             IplImage ** charImage_f= charpartion.partChar(pImgThr);
             IplImage* imagechar_0= charImage_f[0];
             IplImage* imagechar_1= charImage_f[1];
             IplImage* imagechar_2= charImage_f[2];
             IplImage* imagechar_3= charImage_f[3];
             IplImage* imagechar_4= charImage_f[4];
             IplImage* imagechar_5= charImage_f[5];
             IplImage* imagechar_6= charImage_f[6];
//             cvSaveImage(imageAName,imagechar_0);
//             cvSaveImage(imageBName,imagechar_1);
//             cvSaveImage(imageCName,imagechar_2);
//             cvSaveImage(imageDName,imagechar_3);
//             cvSaveImage(imageEName,imagechar_4);
//             cvSaveImage(imageFName,imagechar_5);
//             cvSaveImage(imageGName,imagechar_6);

          QString result=charrecognition.recognizeChar(charImage_f);
           printf("the result plate of the car is %s",result.toLatin1().data());
            cout<<endl;
//                cvShowImage("imagechar_s",imagechar_0);
//                cvWaitKey(0);
//                  cvShowImage("imagechar_s",imagechar_1);
//                  cvWaitKey(0);
//                    cvShowImage("imagechar_s",imagechar_2);
//                   cvWaitKey(0);
//                      cvShowImage("imagechar_s",imagechar_3);
//                      cvWaitKey(0);
//                       cvShowImage("imagechar_s",imagechar_4);
//                       cvWaitKey(0);
//                          cvShowImage("imagechar_s",imagechar_5);
//                          cvWaitKey(0);
//                            cvShowImage("imagechar_s",imagechar_6);
//                            cvWaitKey(0);
//                    cvWaitKey(0);

//                     if(delay != 0){
//                         waitKey(delay);
//                     }
                first_frame=false;
                second_frame=true;
                }

                }

               if(second_frame){
                cv::findContours(findce, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

                //targets.clear();
                const int maxArea = 20000;
                size_t s = contours.size();
                for (size_t i = 0; i < s; i++)
                {
                    double area = abs(contourArea(contours[i]));
                    if (area > maxArea)
                    {
                        Rect mr = boundingRect(Mat(contours[i]));
                        rectangle(src, mr, Scalar(0, 0, 255), 2, 8, 0);
                        //targets.push_back(mr);
                        position_err1=abs(mr.br().y-p0.y);
                        position_err2=abs(mr.br().y-p2.y);
                        char text[50];
                        sprintf(text, "error messure is -%d- ...", position_err1);
                        putText(src, text, Point(450, 50), 3, 1, Scalar(0, 255, 255), 2, 8, false);

//                        cout<<"position_err1:  "<<position_err1<<endl;
//                        cout<<"position_err2:  "<<position_err2<<endl;
                    }
                }
                if(position_err1<=5)
                { time = clock();
                 messure_time2=(double)(clock() - time1)/CLOCKS_PER_SEC;
                 cout<<"the car meet the line at:"<<messure_time2<<"s"<<endl;
                }
                if(position_err2<=10)
                {
                    messure_time=(double)(clock() - time)/CLOCKS_PER_SEC;
       //          cout << "messure _time:" << messure_time << endl;
                speed=distance*3.6/messure_time;
                cout<<"speed:"<<speed<<"km/h"<<endl;
                char text1[50];
                sprintf(text1, "the car speed is %d km/h", speed);
                putText(src, text1, Point(450, 100), 3, 1, Scalar(0, 255, 255), 2, 8, false);

                }

              }
            }



            trainCounter++;
        }






//                 cout<<"src_width"<<src.cols<<endl;
//                  cout<<"src_width"<<src.rows<<endl;


        imshow("src",src);
        cvWaitKey(20);




        }
       //.......
     }
         //   return pImgThr;
            return 0;
}

int myDiffProj(IplImage *src, IplImage *dst)
        {
            int i, j, r_max, r_max_value;//r_max为最大峰值所在行,r_max_value最大峰值的值
            int l_max, l_max_value;//l_max为最大峰值所在行,l_max_value最大峰值的值
            int r_sum[1600] = { 0 }, l_sum[1600] = { 0 }, sum = 0;//r_sum[i]为i行差分值之和,l_sum[i]为i列差分值之和,sum累加单元

            IplImage *pImg8uSmooth = NULL;       //高斯滤波后的图

            if (!src) {
    //            QMessageBox::warning(NULL, "警告", "不能载入源图像");
                return -1;
            } else {
                pImg8uSmooth = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
                cvSmooth(src, pImg8uSmooth, CV_GAUSSIAN, 3, 0, 0);//高斯滤波

                // 水平投影
                // 水平方向一阶差分
                for (j = 0; j<pImg8uSmooth->height; j++) {
                    sum = 0;
                    for (i = 0; i<pImg8uSmooth->width - 2; i++) {
                        sum += abs((S(pImg8uSmooth, i, j) - S(pImg8uSmooth, i + 1, j)));//差分，累加
                    }
                    r_sum[j] = sum;// 保存累加结果
                }
                for (j = 1; j<pImg8uSmooth->height - 2; j++) {
                    r_sum[j] = (int)((r_sum[j - 1] + r_sum[j] + r_sum[j + 1]) / 3);//平滑
                }

                r_max_value = r_sum[0];//寻找最大峰值
                for (i = 0; i<pImg8uSmooth->height; i++) {
                    if (r_max_value<r_sum[i]) {
                       r_max_value = r_sum[i];
                       r_max = i;
                    }
                }

                //row_min最大峰值两侧波谷的较小值所在行,row_min1=0峰值左侧波谷,row_min2=0峰值右侧波谷
                int row_start = -1, row_end = -1, row_min = 0, row_min1 = 0, row_min2 = 0;
                int row_min_value = 0;//最大峰值两侧波谷的较小值

                for (i = r_max; i>1; i--) {
                    if (r_sum[i]<r_sum[i - 1] && r_sum[i]<0.6*r_max_value) {
                        row_min1 = i;
                        break;
                    }//搜索峰值两侧的波谷

                }
                for (i = r_max; i<pImg8uSmooth->height - 1; i++) {
                    if (r_sum[i]<r_sum[i + 1] && r_sum[i]<0.6*r_max_value) {
                        row_min2 = i;
                        break;
                    }
                }
                //波谷的较小值
                if (r_sum[row_min1]<r_sum[row_min2]) {
                    row_min_value = r_sum[row_min1];
                    row_min = row_min1;
                } else {
                    row_min_value = r_sum[row_min2];
                    row_min = row_min2;
                }

                row_start = row_min;
                row_end = row_min;

                if (row_min > r_max) {
                    for (i = r_max; i>1; i--)
                    if (r_sum[i]<row_min_value&&r_sum[i]<0.6*r_max_value) {
                        row_start = i;
                        break;
                    }
                }

                // 没有找到满足条件的起始行
                if (i == 1) {
                    row_start = 0;
                }

                if (row_min < r_max) {
                    for (i = r_max; i<pImg8uSmooth->height - 1; i++)
                    if (r_sum[i]<row_min_value&&r_sum[i]<0.6*r_max_value) {
                        row_end = i;
                        break;
                    }
                }

                // 没有找到满足条件的结束行
                if (i == pImg8uSmooth->height - 1) {
                    row_end = pImg8uSmooth->height;
                }

                int col_start = -1, col_end = -1;

                //垂直投影
                for (i = 0; i<pImg8uSmooth->width; i++) {
                    sum = 0;
                    for (j = 0; j<pImg8uSmooth->height - 1; j++) {
                        sum = abs((S(pImg8uSmooth, i, j) - S(pImg8uSmooth, i, j + 1))) + sum;//差分
                    }
                    l_sum[i] = sum;
                }
                for (i = 1; i<pImg8uSmooth->width - 2; i++) {
                    l_sum[i] = (int)((l_sum[i - 1] + l_sum[i] + l_sum[i + 1]) / 3);//平滑
                }

                l_max_value = l_sum[0];//寻找最大峰值
                for (j = 0; j<pImg8uSmooth->width - 1; j++)
                if (l_max_value<l_sum[j]) {
                    l_max_value = l_sum[j];
                    l_max = j;
                }

                int flag = 1, rig_max = 0, last_max = 0;
                for (j = 1; j<pImg8uSmooth->width - 1; j++)//搜索满足条件的起始和结束峰值
                if (l_sum[j] >= l_sum[j - 1] && l_sum[j]>l_sum[j + 1] && l_sum[j]>0.5*l_max_value) {

                    if (flag) {
                        rig_max = j;
                        flag = 0;
                    }
                    else
                        last_max = j;
                }

                for (j = rig_max; j>1; j--) {
                    if (l_sum[j]<l_sum[j - 1]) {
                        col_start = j;
                        break;
                    }//起始列，结束列
                }

                // 没有找到满足条件的起始列
                if (j == 1) {
                    col_start = 0;
                }

                for (j = last_max; j<pImg8uSmooth->width - 1; j++) {
                    if (l_sum[j]<l_sum[j + 1]) {
                        col_end = j;
                        break;
                    }
                }

                // 没有找到满足条件的结束列
                if (j == pImg8uSmooth->width - 1) {
                    col_end = pImg8uSmooth->width;
                }

                CvRect ROI_rect;                 //获得图片感兴趣区域
                ROI_rect.x = col_start;
                ROI_rect.y = row_start;
                ROI_rect.width = col_end - col_start;
                ROI_rect.height = row_end - row_start;
                IplImage *pImg8uROI = NULL;         //感兴趣的图片

                cvSetImageROI(pImg8uSmooth, ROI_rect);
                pImg8uROI = cvCreateImage(cvSize(ROI_rect.width, ROI_rect.height), IPL_DEPTH_8U, 1);
                cvCopy(pImg8uSmooth, pImg8uROI);
                cvResetImageROI(pImg8uSmooth);

                cvResize(pImg8uROI, dst, CV_INTER_LINEAR); //线性插值

                return 0;
            }

            return -1;
        }

void myScope(IplImage *scopecopy, int x, int y, int *pionter)
        {
            S(scopecopy, x, y) = FLAG;//标记
            pionter[0] = x;
            pionter[1] = x;
            pionter[2] = y;
            pionter[3] = y;
            push(x, y);
            while (sp) {
                pop();
                x = sta1[sp].x;
                y = sta1[sp].y;
                if ((x - 1)>0) // 负X方向搜索
                    if (S(scopecopy, x - 1, y)>250) {
                        push(x - 1, y);
                        S(scopecopy, x - 1, y) = FLAG;//标记
                        if (pionter[0]>x - 1)
                            pionter[0] = x - 1;
                    }

                if ((x + 1)<scopecopy->width) // 正X方向搜索
                    if (S(scopecopy, x + 1, y)>250) {
                        push(x + 1, y);
                        S(scopecopy, x + 1, y) = FLAG;//标记
                        if (pionter[1]<x + 1)
                            pionter[1] = x + 1;
                    }

                if ((y - 1)>0) // 正Y方向搜索
                    if (S(scopecopy, x, y - 1)>250) {
                        push(x, y - 1);
                        S(scopecopy, x, y - 1) = FLAG;//标记
                        if (pionter[2]>y - 1)
                            pionter[2] = y - 1;
                    }

                if ((y + 1)<scopecopy->height) //负Y方向搜索
                    if (S(scopecopy, x, y + 1)>250) {
                        push(x, y + 1);
                        S(scopecopy, x, y + 1) = FLAG;//标记
                        if (pionter[3]<y + 1)
                            pionter[3] = y + 1;
                    }
            }
        }

int myRemoveBorder(IplImage *src, int *post)
        {
            int i, j, sum = 0;
            int r_sum[1000] = { 0 };//, l_sum[1000] = { 0 };

            IplImage *pImg8uSmooth;
            pImg8uSmooth = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
            cvSmooth(src, pImg8uSmooth, CV_GAUSSIAN, 3, 0, 0);//高斯滤波

            //水平投影
            for (j = 0; j<pImg8uSmooth->height; j++) {
                sum = 0;
                for (i = 0; i<pImg8uSmooth->width - 2; i++) {
                    sum = abs((S(pImg8uSmooth, i, j) - S(pImg8uSmooth, i + 1, j))) + sum;//差分
                }
                r_sum[j] = sum;
            }
            for (j = 1; j<pImg8uSmooth->height - 2; j++) {
                r_sum[j] = (int)((r_sum[j - 1] + r_sum[j] + r_sum[j + 1]) / 3);//平滑
            }

            int r_max_value = r_sum[0], r_max = 0;//寻找最大峰值r_max_value,寻找最大峰值所在行r_max
            for (j = 0; j<pImg8uSmooth->height - 1; j++)
            if (r_max_value<r_sum[j]){ r_max_value = r_sum[j]; r_max = j; }

            int flag = 1, rig_max = 0, last_max = 0, row_start = 0, row_end = 0;
            for (j = 1; j<pImg8uSmooth->height - 1; j++)//搜索满足条件的起始和结束峰值
            if (r_sum[j] >= r_sum[j - 1] && r_sum[j]>r_sum[j + 1]) {

                if (flag) {
                    rig_max = j; flag = 0;
                }
                else
                    last_max = j;
            }

            for (j = rig_max; j<pImg8uSmooth->height - 3; j++) {
                if (r_sum[j]<r_sum[j + 1] && r_sum[j + 1]<r_sum[j + 2] && r_sum[j])
                {
                    row_start = j;
                    break;
                }//起始行
            }
            if (r_sum[row_start]>0.5*r_max_value || j == pImg8uSmooth->height - 3) {
                row_start = 0;
            }//满足条件起始峰下侧没有到波谷
            for (j = last_max; j>3; j--) {
                if (r_sum[j]<r_sum[j - 1] && r_sum[j - 1]<r_sum[j - 2] && r_sum[j]){ row_end = j; break; }//结束行

            }
            if (r_sum[row_end]>0.5*r_max_value || j == 3) {
                row_end = pImg8uSmooth->height - 1;
            }//满足条件结束峰上侧没有到波谷

            post[0] = row_start;
            post[1] = row_end;

            return 0;
        }

int myGetThreshold(IplImage *src)
        {
            IplImage *pImg8uSmooth = NULL;       //高斯滤波后的图

            pImg8uSmooth = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
            cvSmooth(src, pImg8uSmooth, CV_GAUSSIAN, 3, 0, 0);//高斯滤波
            int size = 125, i = 0;
            int ranges[2] = { 0, 255 };
            int THRESHOLD = 0;
            myHist hist;
            myCalHist(&hist, pImg8uSmooth, size, ranges);

            for (i = hist.bin_max; i<hist.size - 4; i++) {
                if (hist.bin[i]<hist.bin[i + 1] && hist.bin[i + 1]<hist.bin[i + 2] && hist.bin[i + 2]<hist.bin[i + 3])
                    break;
            }

            if (i<hist.size - 4)
                THRESHOLD = (hist.ranges[1] - hist.ranges[0]) / hist.size*i;
            else
                THRESHOLD = (hist.ranges[1] - hist.ranges[0]) / hist.size*hist.bin_max;

            return THRESHOLD;

        }

void myCalHist(myHist *histponter, IplImage * src, int size, int *ranges)//
        {
            int i = 0, j = 0, k = 0, counter = 0;//计数器，记录某一范围中像素的个数

            int bin_wid = (ranges[1] - ranges[0]) / size;//每个所表示的直方块像素范围

            while (k<size) {
                counter = 0;
                histponter->bin[k] = 0;
                for (i = 0; i<src->width; i++)
                for (j = 0; j<src->height; j++) {
                    if (S(src, i, j)>k*bin_wid&&S(src, i, j)<(k + 1)*bin_wid)
                        counter++;
                }
                histponter->bin[k++] = counter;
            }

            histponter->size = size;
            histponter->ranges[0] = ranges[0];
            histponter->ranges[1] = ranges[1];

            histponter->max_value = histponter->bin[0];
            histponter->bin_max = 0;
            for (i = 0; i<k; i++) {
                if (histponter->bin[i]>histponter->max_value){ histponter->max_value = histponter->bin[i]; histponter->bin_max = i; }
            }

            histponter->min_value = histponter->bin[0];
            histponter->bin_min = 0;
            for (i = 0; i<k; i++) {
                if (histponter->bin[i]<histponter->min_value) {
                    histponter->min_value = histponter->bin[i];
                    histponter->bin_min = i;
                }
            }
        }
