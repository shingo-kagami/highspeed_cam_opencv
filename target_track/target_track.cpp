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

class VideoCapturePGR : public cv::VideoCapture {
public:
    VideoCapturePGR(int device) {
        open(device);
    }
    ~VideoCapturePGR() {
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
        if (image.rows == 0) {
            image = cv::Mat(buffer.image.iRows, buffer.image.iCols, CV_8U);
        }
        memcpy((uchar *)image.data, (uchar *)buffer.image.pData, image.rows * image.cols);
        flycaptureUnlock(flycapture, buffer.uiBufferIndex);
        return true;
    }
private: 
    FlyCaptureContext flycapture;
    FlyCaptureError error;
    FlyCaptureImagePlus buffer;
};

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

class UiMsg : public MsgData {
public:
    cv::Point mpos;
};

void
onMouse(int event, int x, int y, int flags, void *param)
{
    MsgLink<UiMsg> *lu = (MsgLink<UiMsg> *)param;

    if (event == CV_EVENT_LBUTTONDOWN) {
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
                  cv::Point(center.x - TEMPLATE_SIZE/2,
                            center.y - TEMPLATE_SIZE/2),
                  cv::Point(center.x + TEMPLATE_SIZE/2 - 1,
                            center.y + TEMPLATE_SIZE/2 - 1),
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
    cv::setMouseCallback("disp", onMouse, lu);
    cv::Mat dispimg;

    while (1) {
        DispMsg *md = ld->receive();
        if (md != NULL) {
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
    center.x = (int)MAX(center.x, TEMPLATE_SIZE/2);
    center.y = (int)MAX(center.y, TEMPLATE_SIZE/2);
    center.x = (int)MIN(center.x, frame.cols - 1 - TEMPLATE_SIZE/2);
    center.y = (int)MIN(center.y, frame.rows - 1 - TEMPLATE_SIZE/2);
    
    cv::getRectSubPix(frame, cv::Size(TEMPLATE_SIZE, TEMPLATE_SIZE),
                      center, templ);
}

void
trackTemplate(cv::Mat& frame, cv::Mat& templ, cv::Point& center)
{
    cv::Point stl, sbr;
    stl.x = (int)MAX(center.x - SEARCH_SIZE/2, 0);
    stl.y = (int)MAX(center.y - SEARCH_SIZE/2, 0);
    sbr.x = (int)MIN(center.x + SEARCH_SIZE/2 - 1, frame.cols - 1);
    sbr.y = (int)MIN(center.y + SEARCH_SIZE/2 - 1, frame.rows - 1);
    cv::Mat search(frame, cv::Rect(stl.x, stl.y,
                                   sbr.x - stl.x + 1, sbr.y - stl.y + 1));
    cv::Mat result;
    cv::matchTemplate(search, templ, result, CV_TM_SQDIFF_NORMED);
    cv::Point minloc;
    cv::minMaxLoc(result, NULL, NULL, &minloc);
    center.x = stl.x + minloc.x + TEMPLATE_SIZE/2;
    center.y = stl.y + minloc.y + TEMPLATE_SIZE/2;
}

int
main()
{
    VideoCapturePGR cap(0);
    MsgLink<DispMsg> linkd, *ld = &linkd;
    MsgLink<UiMsg> linku, *lu = &linku;
    boost::thread th(boost::bind(dispThread, ld, lu));

    cv::Mat frame, img, templ;
    cap >> frame;
    cv::Point center(frame.cols / 2, frame.rows / 2);
    setTemplate(frame, templ, center);

    while (1) {
        DispMsg *md = ld->prepareMsg();
        cap >> md->image;
        trackTemplate(md->image, templ, center);
        md->index = (md->index + 1) % NHISTORY;
        md->center[md->index] = center;
        ld->send();
        if (ld->isClosed()) { break; }

        UiMsg *mu = lu->receive();
        if (mu != NULL) {
            center = mu->mpos;
            setTemplate(md->image, templ, center);
        }
    }
    return 0;
}
