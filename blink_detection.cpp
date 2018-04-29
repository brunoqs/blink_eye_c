// OpenCV Sample Application: blink_detction.cpp
// Include header files

// includes for blink_detection
#include "cv.h"
#include "highgui.h" 

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>
#include <iostream>

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// includes do templateMatch opencv
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/core/internal.hpp"
#include <math.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <float.h>

using namespace cv;

// funcoes retiradas do opencv
void crossCor( const Mat& img, const Mat& _templ, Mat& corr,
                Size corrsize, int ctype,
                Point anchor, double delta, int borderType )
{
    const double blockScale = 4.5;
    const int minBlockSize = 256;
    std::vector<uchar> buf;

    Mat templ = _templ;
    int depth = img.depth(), cn = img.channels();
    int tdepth = templ.depth(), tcn = templ.channels();
    int cdepth = CV_MAT_DEPTH(ctype), ccn = CV_MAT_CN(ctype);

    CV_Assert( img.dims <= 2 && templ.dims <= 2 && corr.dims <= 2 );

    if( depth != tdepth && tdepth != std::max(CV_32F, depth) )
    {
        _templ.convertTo(templ, std::max(CV_32F, depth));
        tdepth = templ.depth();
    }

    CV_Assert( depth == tdepth || tdepth == CV_32F);
    CV_Assert( corrsize.height <= img.rows + templ.rows - 1 &&
               corrsize.width <= img.cols + templ.cols - 1 );

    CV_Assert( ccn == 1 || delta == 0 );

    corr.create(corrsize, ctype);

    int maxDepth = depth > CV_8S ? CV_64F : std::max(std::max(CV_32F, tdepth), cdepth);
    Size blocksize, dftsize;

    blocksize.width = cvRound(templ.cols*blockScale);
    blocksize.width = std::max( blocksize.width, minBlockSize - templ.cols + 1 );
    blocksize.width = std::min( blocksize.width, corr.cols );
    blocksize.height = cvRound(templ.rows*blockScale);
    blocksize.height = std::max( blocksize.height, minBlockSize - templ.rows + 1 );
    blocksize.height = std::min( blocksize.height, corr.rows );

    dftsize.width = std::max(getOptimalDFTSize(blocksize.width + templ.cols - 1), 2);
    dftsize.height = getOptimalDFTSize(blocksize.height + templ.rows - 1);
    if( dftsize.width <= 0 || dftsize.height <= 0 )
        CV_Error( CV_StsOutOfRange, "the input arrays are too big" );

    // recompute block size
    blocksize.width = dftsize.width - templ.cols + 1;
    blocksize.width = MIN( blocksize.width, corr.cols );
    blocksize.height = dftsize.height - templ.rows + 1;
    blocksize.height = MIN( blocksize.height, corr.rows );

    Mat dftTempl( dftsize.height*tcn, dftsize.width, maxDepth );
    Mat dftImg( dftsize, maxDepth );

    int i, k, bufSize = 0;
    if( tcn > 1 && tdepth != maxDepth )
        bufSize = templ.cols*templ.rows*CV_ELEM_SIZE(tdepth);

    if( cn > 1 && depth != maxDepth )
        bufSize = std::max( bufSize, (blocksize.width + templ.cols - 1)*
            (blocksize.height + templ.rows - 1)*CV_ELEM_SIZE(depth));

    if( (ccn > 1 || cn > 1) && cdepth != maxDepth )
        bufSize = std::max( bufSize, blocksize.width*blocksize.height*CV_ELEM_SIZE(cdepth));

    buf.resize(bufSize);

    // compute DFT of each template plane
    for( k = 0; k < tcn; k++ )
    {
        int yofs = k*dftsize.height;
        Mat src = templ;
        Mat dst(dftTempl, Rect(0, yofs, dftsize.width, dftsize.height));
        Mat dst1(dftTempl, Rect(0, yofs, templ.cols, templ.rows));

        if( tcn > 1 )
        {
            src = tdepth == maxDepth ? dst1 : Mat(templ.size(), tdepth, &buf[0]);
            int pairs[] = {k, 0};
            mixChannels(&templ, 1, &src, 1, pairs, 1);
        }

        if( dst1.data != src.data )
            src.convertTo(dst1, dst1.depth());

        if( dst.cols > templ.cols )
        {
            Mat part(dst, Range(0, templ.rows), Range(templ.cols, dst.cols));
            part = Scalar::all(0);
        }
        dft(dst, dst, 0, templ.rows);
    }

    int tileCountX = (corr.cols + blocksize.width - 1)/blocksize.width;
    int tileCountY = (corr.rows + blocksize.height - 1)/blocksize.height;
    int tileCount = tileCountX * tileCountY;

    Size wholeSize = img.size();
    Point roiofs(0,0);
    Mat img0 = img;

    if( !(borderType & BORDER_ISOLATED) )
    {
        img.locateROI(wholeSize, roiofs);
        img0.adjustROI(roiofs.y, wholeSize.height-img.rows-roiofs.y,
                       roiofs.x, wholeSize.width-img.cols-roiofs.x);
    }
    borderType |= BORDER_ISOLATED;

    // calculate correlation by blocks
    for( i = 0; i < tileCount; i++ )
    {
        int x = (i%tileCountX)*blocksize.width;
        int y = (i/tileCountX)*blocksize.height;

        Size bsz(std::min(blocksize.width, corr.cols - x),
                 std::min(blocksize.height, corr.rows - y));
        Size dsz(bsz.width + templ.cols - 1, bsz.height + templ.rows - 1);
        int x0 = x - anchor.x + roiofs.x, y0 = y - anchor.y + roiofs.y;
        int x1 = std::max(0, x0), y1 = std::max(0, y0);
        int x2 = std::min(img0.cols, x0 + dsz.width);
        int y2 = std::min(img0.rows, y0 + dsz.height);
        Mat src0(img0, Range(y1, y2), Range(x1, x2));
        Mat dst(dftImg, Rect(0, 0, dsz.width, dsz.height));
        Mat dst1(dftImg, Rect(x1-x0, y1-y0, x2-x1, y2-y1));
        Mat cdst(corr, Rect(x, y, bsz.width, bsz.height));

        for( k = 0; k < cn; k++ )
        {
            Mat src = src0;
            dftImg = Scalar::all(0);

            if( cn > 1 )
            {
                src = depth == maxDepth ? dst1 : Mat(y2-y1, x2-x1, depth, &buf[0]);
                int pairs[] = {k, 0};
                mixChannels(&src0, 1, &src, 1, pairs, 1);
            }

            if( dst1.data != src.data )
                src.convertTo(dst1, dst1.depth());

            if( x2 - x1 < dsz.width || y2 - y1 < dsz.height )
                copyMakeBorder(dst1, dst, y1-y0, dst.rows-dst1.rows-(y1-y0),
                               x1-x0, dst.cols-dst1.cols-(x1-x0), borderType);

            dft( dftImg, dftImg, 0, dsz.height );
            Mat dftTempl1(dftTempl, Rect(0, tcn > 1 ? k*dftsize.height : 0,
                                         dftsize.width, dftsize.height));
            mulSpectrums(dftImg, dftTempl1, dftImg, 0, true);
            dft( dftImg, dftImg, DFT_INVERSE + DFT_SCALE, bsz.height );

            src = dftImg(Rect(0, 0, bsz.width, bsz.height));

            if( ccn > 1 )
            {
                if( cdepth != maxDepth )
                {
                    Mat plane(bsz, cdepth, &buf[0]);
                    src.convertTo(plane, cdepth, 1, delta);
                    src = plane;
                }
                int pairs[] = {0, k};
                mixChannels(&src, 1, &cdst, 1, pairs, 1);
            }
            else
            {
                if( k == 0 )
                    src.convertTo(cdst, cdepth, 1, delta);
                else
                {
                    if( maxDepth != cdepth )
                    {
                        Mat plane(bsz, cdepth, &buf[0]);
                        src.convertTo(plane, cdepth);
                        src = plane;
                    }
                    add(src, cdst, cdst);
                }
            }
        }
    }
}



/*****************************************************************************************/

void matchTemp( InputArray _img, InputArray _templ, OutputArray _result, int method )
{
    CV_Assert( CV_TM_SQDIFF <= method && method <= CV_TM_CCOEFF_NORMED );

    int numType = method == CV_TM_CCORR || method == CV_TM_CCORR_NORMED ? 0 :
                  method == CV_TM_CCOEFF || method == CV_TM_CCOEFF_NORMED ? 1 : 2;
    bool isNormed = method == CV_TM_CCORR_NORMED ||
                    method == CV_TM_SQDIFF_NORMED ||
                    method == CV_TM_CCOEFF_NORMED;

    Mat img = _img.getMat(), templ = _templ.getMat();
    if( img.rows < templ.rows || img.cols < templ.cols )
        std::swap(img, templ);

    CV_Assert( (img.depth() == CV_8U || img.depth() == CV_32F) &&
               img.type() == templ.type() );

    CV_Assert( img.rows >= templ.rows && img.cols >= templ.cols);

    Size corrSize(img.cols - templ.cols + 1, img.rows - templ.rows + 1);
    _result.create(corrSize, CV_32F);
    Mat result = _result.getMat();

#ifdef HAVE_TEGRA_OPTIMIZATION
    if (tegra::matchTemplate(img, templ, result, method))
        return;
#endif

    int cn = img.channels();
    crossCor( img, templ, result, result.size(), result.type(), Point(0,0), 0, 0);

    if( method == CV_TM_CCORR )
        return;

    double invArea = 1./((double)templ.rows * templ.cols);

    Mat sum, sqsum;
    Scalar templMean, templSdv;
    double *q0 = 0, *q1 = 0, *q2 = 0, *q3 = 0;
    double templNorm = 0, templSum2 = 0;

    if( method == CV_TM_CCOEFF )
    {
        integral(img, sum, CV_64F);
        templMean = mean(templ);
    }
    else
    {
        integral(img, sum, sqsum, CV_64F);
        meanStdDev( templ, templMean, templSdv );

        templNorm = CV_SQR(templSdv[0]) + CV_SQR(templSdv[1]) +
                    CV_SQR(templSdv[2]) + CV_SQR(templSdv[3]);

        if( templNorm < DBL_EPSILON && method == CV_TM_CCOEFF_NORMED )
        {
            result = Scalar::all(1);
            return;
        }

        templSum2 = templNorm +
                     CV_SQR(templMean[0]) + CV_SQR(templMean[1]) +
                     CV_SQR(templMean[2]) + CV_SQR(templMean[3]);

        if( numType != 1 )
        {
            templMean = Scalar::all(0);
            templNorm = templSum2;
        }

        templSum2 /= invArea;
        templNorm = sqrt(templNorm);
        templNorm /= sqrt(invArea); // care of accuracy here

        q0 = (double*)sqsum.data;
        q1 = q0 + templ.cols*cn;
        q2 = (double*)(sqsum.data + templ.rows*sqsum.step);
        q3 = q2 + templ.cols*cn;
    }

    double* p0 = (double*)sum.data;
    double* p1 = p0 + templ.cols*cn;
    double* p2 = (double*)(sum.data + templ.rows*sum.step);
    double* p3 = p2 + templ.cols*cn;

    int sumstep = sum.data ? (int)(sum.step / sizeof(double)) : 0;
    int sqstep = sqsum.data ? (int)(sqsum.step / sizeof(double)) : 0;

    int i, j, k;

    for( i = 0; i < result.rows; i++ )
    {
        float* rrow = (float*)(result.data + i*result.step);
        int idx = i * sumstep;
        int idx2 = i * sqstep;

        for( j = 0; j < result.cols; j++, idx += cn, idx2 += cn )
        {
            double num = rrow[j], t;
            double wndMean2 = 0, wndSum2 = 0;

            if( numType == 1 )
            {
                for( k = 0; k < cn; k++ )
                {
                    t = p0[idx+k] - p1[idx+k] - p2[idx+k] + p3[idx+k];
                    wndMean2 += CV_SQR(t);
                    num -= t*templMean[k];
                }

                wndMean2 *= invArea;
            }

            if( isNormed || numType == 2 )
            {
                for( k = 0; k < cn; k++ )
                {
                    t = q0[idx2+k] - q1[idx2+k] - q2[idx2+k] + q3[idx2+k];
                    wndSum2 += t;
                }

                if( numType == 2 )
                {
                    num = wndSum2 - 2*num + templSum2;
                    num = MAX(num, 0.);
                }
            }

            if( isNormed )
            {
                t = sqrt(MAX(wndSum2 - wndMean2,0))*templNorm;
                if( fabs(num) < t )
                    num /= t;
                else if( fabs(num) < t*1.125 )
                    num = num > 0 ? 1 : -1;
                else
                    num = method != CV_TM_SQDIFF_NORMED ? 0 : 1;
            }

            rrow[j] = (float)num;
        }
    }
}


// Create memory for calculations
static CvMemStorage* storage = 0;

// Create a new Haar classifier
static CvHaarClassifierCascade* cascade = 0;

// Function prototype for detecting and drawing an object from an image
bool detect_and_draw( IplImage* image ,CvHaarClassifierCascade* cascade);

// Create a string that contains the cascade name
/*    "eyes.xml*/
const char *cascade_name[1]={"eyes.xml"};

cv::Mat roiImg;

int threshold_value = 200;
int threshold_type = 3;;
int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;

int hough_thr = 35;
cv::Mat src_gray, dst;

using namespace cv;

Mat img1; Mat img2; Mat templ; Mat result;
const char* image_window = "Source Image";
const char* result_window = "Result window";

int match_method=0;
int max_Trackbar = 5;

int eye_open=0;
int eye_close=0;
/*
**
 * @function MatchingMethod
 * @brief Trackbar callback
 
*/

//Matching with 2 images ,eye closed or open
void MatchingMethod(cv::Mat templ,int id){
  /// Source image to display
  cv::Mat img_display;
  roiImg.copyTo(img_display);

  /// Create the result matrix
  int result_cols = roiImg.cols - templ.cols + 1;
  int result_rows = roiImg.rows - templ.rows + 1;

  result.create(result_cols, result_rows, CV_32FC1);

  /// Do the Matching and Normalize
  matchTemp(roiImg, templ, result, match_method);
  cv::normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());

  /// Localizing the best match with minMaxLoc
  double minVal; double maxVal; Point minLoc; Point maxLoc;
  cv::Point matchLoc;

  cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

  ///Justing checkin the match template value reaching the threashold
  if(id == 0 && (minVal < 0)){
    eye_open = eye_open + 1;
    if(eye_open == 10){
      std::cout<<"Eye Open"<<std::endl;
      eye_open=0;
      eye_close=0;
    }
  }
  else if(id == 1 && (minVal < 0))
	eye_close=eye_close+1;
  if(eye_close == 10){
    std::cout<<"Eye Closed"<<std::endl;
    eye_close=0;
    system("python send_arduino.py");
  }

  /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
  if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED ){
    matchLoc = minLoc;
  } else {
    matchLoc = maxLoc;
  }

  /// Show me what you got
  cv::rectangle(img_display, matchLoc, Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows), Scalar::all(0), 2, 8, 0 );
  cv::rectangle(result, matchLoc, Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows), Scalar::all(0), 2, 8, 0);

  cv::imshow(image_window, img_display);
  cv::imshow(result_window, result);

}


void detect_blink(cv::Mat roi){
	try{  	
       MatchingMethod(img1, 0);
       MatchingMethod(img2, 1);
    }
    catch(cv::Exception& e){
      std::cout << "An exception occued" << std::endl;
    }
}

// Main function, defines the entry point for the program.
int main(int argc, char** argv){
    if(argc <= 1){
       std::cout << "\n Help " << std::endl;
       std::cout << "\n ------------------------------------\n" << std::endl;
       std::cout << "./blink_detect open_eye.jpg close_eye.jpg\n" << std::endl;
       std::cout << "Eg :: ./blink_detect 2.jpg 3.jpg\n" << std::endl;
       std::cout << "\n ------------------------------------\n" << std::endl;
       exit(0);
    }

    // Structure for getting video from camera or avi
    CvCapture* capture = 0;

    // Images to capture the frame from video or camera or from file
    IplImage *frame, *frame_copy = 0;

    // Used for calculations
    int optlen = strlen("--cascade=");

    // Input file name for avi or image file.
    const char* input_name;

    img1 = imread(argv[1], 1);
    img2 = imread(argv[2], 1);

    // Load the HaarClassifierCascade
    /// Create windows
    cv::namedWindow(image_window, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(result_window, CV_WINDOW_AUTOSIZE);

    // Allocate the memory storage
    storage = cvCreateMemStorage(0);

    // resolution 
    capture = cvCaptureFromCAM(0);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 100);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 100);
    // Create a new named window with title: result
    cvNamedWindow("original_frame", 1);

    // If loaded succesfully, then:
    if(capture){
        // Capture from the camera.
        for(;;){
            // Capture the frame and load it in IplImage
            if( !cvGrabFrame( capture )) break;
            frame = cvRetrieveFrame( capture );

            // If the frame does not exist, quit the loop
            if(!frame) break;

            // Allocate framecopy as the same size of the frame
            if(!frame_copy){
                frame_copy = cvCreateImage(cvSize(frame->width,frame->height),
                IPL_DEPTH_8U, frame->nChannels);
            }
            // Check the origin of image. If top left, copy the image frame to frame_copy. 
            if(frame->origin == IPL_ORIGIN_TL) cvCopy(frame, frame_copy, 0);
            // Else flip and copy the image

            //for(int i = 0; i < 1; i++){
                cascade = (CvHaarClassifierCascade*)cvLoad( cascade_name[0], 0, 0, 0 );

     	        // Check whether the cascade has loaded successfully. Else report and error and quit
                if(!cascade){
                    fprintf(stderr, "ERROR: Could not load classifier cascade\n");
                    return -1;
                }
                // Call the function to detect and draw the face
                detect_and_draw(frame_copy, cascade);
            //}
            // Wait for a while before proceeding to the next frame
            if(cvWaitKey(1) >= 0) break;
        }

        // Release the images, and capture memory
        cvReleaseHaarClassifierCascade(&cascade);
        cvReleaseImage( &frame_copy );
        cvReleaseCapture( &capture );
        cvReleaseMemStorage(&storage);
    }
    return 0;
}

// Function to detect and draw any faces that is present in an image
bool detect_and_draw( IplImage* img,CvHaarClassifierCascade* cascade )
{
    int scale = 1;

    // Create a new image based on the input image
    IplImage* temp = cvCreateImage( cvSize(img->width/scale,img->height/scale), 8, 3 );

    // Create two points to represent the face locations
    CvPoint pt1, pt2;
    int i;

    // Clear the memory storage which was used before
    cvClearMemStorage( storage );

    // Find whether the cascade is loaded, to find the faces. If yes, then:
    if( cascade )
    {

        // There can be more than one face in an image. So create a growable sequence of faces.
        // Detect the objects and store them in the sequence
        CvSeq* faces = cvHaarDetectObjects( img, cascade, storage,
            1.1, 8, CV_HAAR_DO_CANNY_PRUNING,
            cvSize(40, 40) );

        // Loop the number of faces found.
        for( i = 0; i < (faces ? faces->total : 0); i++ )
        {
           // Create a new rectangle for drawing the face
            CvRect* r = (CvRect*)cvGetSeqElem( faces, i );

            // Find the dimensions of the face,and scale it if necessary
            pt1.x = r->x*scale;
            pt2.x = (r->x+r->width)*scale;
            pt1.y = r->y*scale;
            pt2.y = (r->y+r->height)*scale;

            // Draw the rectangle in the input image
            cvRectangle( img, pt1, pt2, CV_RGB(255,0,0), 3, 8, 0 );

            Mat image = cv::cvarrToMat(img);
            cv::Rect rect;

            rect = cv::Rect(pt1.x,pt1.y,(pt2.x-pt1.x),(pt2.y-pt1.y));

            roiImg = image(rect);
            cv::imshow("roi",roiImg);	


///Send to arduino

            detect_blink(roiImg);

        }
    }


    // Show the image in the window named "result"
    cvShowImage( "original_frame", img );


    if(i  > 0)
      return 1;
  else
      return 0;


    // Release the temp image created.


  cvReleaseImage( &temp );


}
