#ifndef _MSGLINK_HPP_
#define _MSGLINK_HPP_

#include <boost/thread.hpp>

class MsgData {
    template<class T> friend class MsgLink;
public:
    MsgData() : seqno(0) {};
    virtual ~MsgData() {};
    virtual void copyTo(MsgData *dst) {}
private:
    unsigned int seqno;
};

template<class CustomMsgData>
class MsgLink {
public:
    MsgLink() : master_seqno(0), closed(false) {
        md_snd = new CustomMsgData();
        md_med = new CustomMsgData();
        md_rcv = new CustomMsgData();
    }
    ~MsgLink() {
        delete md_snd;
        delete md_med;
        delete md_rcv;
    }

    CustomMsgData *prepareMsg() const {
        return md_snd;
    }

    void send() {
        master_seqno++;
        boost::mutex::scoped_lock lock(mt);
        md_snd->seqno = master_seqno;
        swapPtr(&md_snd, &md_med);
        md_med->copyTo(md_snd);
    }

    bool isUpdated() const {
        return md_med->seqno > md_rcv->seqno;
    }

    CustomMsgData *receive() {
        if (!isUpdated()) {
            return NULL;
        }
        boost::mutex::scoped_lock lock(mt);
        swapPtr(&md_med, &md_rcv);
        return md_rcv;
    }

    bool isClosed() const {
        return closed;
    }
    
    void close() {
        closed = true;
    }
    
private:
    void swapPtr(CustomMsgData **p1, CustomMsgData **p2) {
        CustomMsgData *tmp;
        tmp = *p1;
        *p1 = *p2;
        *p2 = tmp;
    }
    
    CustomMsgData *md_snd;
    CustomMsgData *md_med;
    CustomMsgData *md_rcv;
    boost::mutex mt;
    unsigned int master_seqno;
    bool closed;
};

#endif /* _MSGLINK_HPP_ */
