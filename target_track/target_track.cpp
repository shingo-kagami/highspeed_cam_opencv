/**
 @file target_track.cpp
 @brief A template-matching exmaple with a separate GUI thread.
 @author Shingo W. Kagami swk(at)ic.is.tohoku.ac.jp
 */

#include <string.h>
#include "opencv2/opencv.hpp"
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include "FC1/PGRFlyCapture.h"
#include "FC1/PGRFlyCapturePlus.h"
#include "msglink.hpp"

#define TEMPLATE_SIZE 64
#define SEARCH_SIZE 72
#define NHISTORY 32

/**
 * A subsclass of cv::VideoCapture that encapsulates PGRFlyCapture
 * API.  For simplicity, this implementation assumes that both the
 * camera and the output image are in monochrome 8-bit format (see the
 * comment in retrieve() method).
 *
 * Usage:
 *    VideoCaptureFlyCap cap(n); // n: device index of FlyCapture camera
 * 
 *  Once cap is obtained, it can be used just like cv::VideoCapture
 *  object (see main() function for a sample usage).
 */
class VideoCaptureFlyCap : public cv::VideoCapture {
public:
    VideoCaptureFlyCap(int device) {
        open(device);
    }
    ~VideoCaptureFlyCap() {
        flycaptureStop(flycapture);
        flycaptureDestroyContext(flycapture);
    }
    bool isOpened() const {
        return (error == FLYCAPTURE_OK)? true: false;
    }
    bool open(int device) {
        flycaptureCreateContext(&flycapture);
        error = flycaptureInitialize(flycapture, device);
        if (error != FLYCAPTURE_OK) {
            return false;
        }
        error = flycaptureStart(flycapture, FLYCAPTURE_VIDEOMODE_ANY,
                                FLYCAPTURE_FRAMERATE_ANY);
        return isOpened();
    }
    bool grab() {
        flycaptureLockLatest(flycapture, &buffer);
        return true;
    }
    bool retrieve(cv::Mat& image, int channel = 0) {
        /*
         * We assume that the camera takes a monochrome 8-bit image
         * whose pData can be simply copied to cv::Mat image data in
         * CV_8U format.  For more general image formats, demosaicing
         * functions such as flycaptureConvertImage() are needed
         * instead of just memcpy().
         */
        if (image.rows == 0) {
            image = cv::Mat(buffer.image.iRows, buffer.image.iCols, CV_8U);
        }
        memcpy((uchar *)image.data, (uchar *)buffer.image.pData,
               image.rows * image.cols);
        flycaptureUnlock(flycapture, buffer.uiBufferIndex);
        return true;
    }
private: 
    FlyCaptureContext flycapture;
    FlyCaptureError error;
    FlyCaptureImagePlus buffer;
};

/*
 * A message for displaying the results, containing the image, a ring
 * buffer for the trajectory of the tracked center (recent NHISTORY
 * points), and the index to the latest point in the ring buffer.  The
 * ring buffer and the index need to be copied between the sender's
 * buffer and the intermediate buffer because their previous values
 * are used for frame-by-frame updates, while the image need not.
 */
class DispMsg : public MsgData {
public:
    DispMsg() {
        for (int i = 0; i < NHISTORY; i++) {
            center[i].x = center[i].y = -1;
        }
        index = -1;
    }
    void copyTo(DispMsg *dst) {
        for (int i = 0; i < NHISTORY; i++) {
            dst->center[i] = center[i];
        }
        dst->index = index;
    }
    cv::Mat image;
    cv::Point center[NHISTORY];
    int index;
};

/*
 * A message for UI, namely used for mouse event notification.  The
 * mouse pointer position is contained.
 */
class UiMsg : public MsgData {
public:
    cv::Point mpos;
};

void
onMouse(int event, int x, int y, int flags, void *param)
{
    MsgLink<UiMsg> *lu = (MsgLink<UiMsg> *)param;

    if (event == CV_EVENT_LBUTTONDOWN) {
        // Mouse position (x, y) is set as a message
        UiMsg *mu = lu->prepareMsg();
        mu->mpos.x = x;
        mu->mpos.y = y;
        lu->send();
    }
}

void
drawTrackRect(cv::Mat& image, cv::Point& center, int thickness)
{
    cv::rectangle(image, 
                  cv::Point(center.x - TEMPLATE_SIZE / 2,
                            center.y - TEMPLATE_SIZE / 2),
                  cv::Point(center.x + TEMPLATE_SIZE / 2 - 1,
                            center.y + TEMPLATE_SIZE / 2 - 1),
                  CV_RGB(0, 0, 255), thickness);
}

void
drawTrackResults(cv::Mat& image, DispMsg *md)
{
    drawTrackRect(image, md->center[md->index], 3);
    for (int i = 0; i < NHISTORY; i++) {
        if (md->center[i].x >= 0) {
            cv::circle(image, md->center[i], 3, CV_RGB(255, 0, 0), 1);
        }
    }
}

void
dispThread(MsgLink<DispMsg> *ld, MsgLink<UiMsg> *lu)
{
    cv::namedWindow("disp", CV_WINDOW_AUTOSIZE);

    // Delegate the UI message link to the mouse callback function
    cv::setMouseCallback("disp", onMouse, lu);

    // for displaying the tracking results in color
    cv::Mat dispimg;

    while (1) {
        DispMsg *md = ld->receive();
        if (md != NULL) { // If a display message is received
            cv::cvtColor(md->image, dispimg, CV_GRAY2BGR);
            drawTrackResults(dispimg, md);
            cv::imshow("disp", dispimg);
        }
        if (cv::waitKey(30) > 0) {
            break;
        }
    }
    ld->close();
}

void
setTemplate(cv::Mat& frame, cv::Mat& templ, cv::Point& center)
{
    center.x = (int)MAX(center.x, TEMPLATE_SIZE / 2);
    center.y = (int)MAX(center.y, TEMPLATE_SIZE / 2);
    center.x = (int)MIN(center.x, frame.cols - 1 - TEMPLATE_SIZE / 2);
    center.y = (int)MIN(center.y, frame.rows - 1 - TEMPLATE_SIZE / 2);
    
    cv::getRectSubPix(frame, cv::Size(TEMPLATE_SIZE, TEMPLATE_SIZE),
                      center, templ);
}

void
trackTemplate(cv::Mat& frame, cv::Mat& templ, cv::Point& center)
{
    cv::Point stl, sbr;
    stl.x = (int)MAX(center.x - SEARCH_SIZE / 2, 0);
    stl.y = (int)MAX(center.y - SEARCH_SIZE / 2, 0);
    sbr.x = (int)MIN(center.x + SEARCH_SIZE / 2 - 1, frame.cols - 1);
    sbr.y = (int)MIN(center.y + SEARCH_SIZE / 2 - 1, frame.rows - 1);
    cv::Mat search(frame, cv::Rect(stl.x, stl.y,
                                   sbr.x - stl.x + 1, sbr.y - stl.y + 1));
    cv::Mat result;
    cv::matchTemplate(search, templ, result, CV_TM_SQDIFF_NORMED);
    cv::Point minloc;
    cv::minMaxLoc(result, NULL, NULL, &minloc);
    center.x = stl.x + minloc.x + TEMPLATE_SIZE / 2;
    center.y = stl.y + minloc.y + TEMPLATE_SIZE / 2;
}

int
main()
{
    // 0-th FlyCap camera
    VideoCaptureFlyCap cap(0);

    // MsgLink to display thread
    MsgLink<DispMsg> linkd, *ld = &linkd;

    // MsgLink from display thread
    MsgLink<UiMsg> linku, *lu = &linku;

    // Display thread is created
    boost::thread th(boost::bind(dispThread, ld, lu));

    cv::Mat frame, img, templ;

    // Template image is sampled from around the center of the first frame
    cap >> frame;
    cv::Point center(frame.cols / 2, frame.rows / 2);
    setTemplate(frame, templ, center);

    while (1) {
        // Captured image is directly stored in the message to be sent
        DispMsg *md = ld->prepareMsg();
        cap >> md->image;

        // Tracking results are written into the ring buffer in the message
        trackTemplate(md->image, templ, center);
        md->index = (md->index + 1) % NHISTORY;
        md->center[md->index] = center;

        ld->send();
        if (ld->isClosed()) { // Check if the peer has finished
            break;
        }

        UiMsg *mu = lu->receive();
        if (mu != NULL) { // If the mouse button pressed
            // Template image is sampled from around the mouse position
            center = mu->mpos;
            setTemplate(md->image, templ, center);
        }
    }

    return 0;
}
