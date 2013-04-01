/**
 @file multi_threaded_cap.cpp
 @brief A simplest sample code of using MsgLink.
 @author Shingo W. Kagami swk(at)ic.is.tohoku.ac.jp
 */

#include "opencv2/opencv.hpp"
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include "msglink.hpp"

class DispMsg : public MsgData {
public:
    cv::Mat image;
};

void
dispThread(MsgLink<DispMsg> *ld)
{
    cv::namedWindow("disp", CV_WINDOW_AUTOSIZE);
    while (1) {
        DispMsg *md = ld->receive();
        if (md != NULL) {
            cv::imshow("disp", md->image);
        }
        if (cv::waitKey(30) > 0) {
            break;
        }
    }
    ld->close();
}

int
main()
{
    cv::VideoCapture cap(0);
    MsgLink<DispMsg> linkd, *ld = &linkd;
    boost::thread th(boost::bind(dispThread, ld));
    while (1) {
        DispMsg *md = ld->prepareMsg();
        cap >> md->image;
        // do_some_processing(md->image);
        ld->send();
        if (ld->isClosed()) {
            break;
        }
    }
    return 0;
}
