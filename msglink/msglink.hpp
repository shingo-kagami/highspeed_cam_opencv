/**
 @file msglink.hpp
 @brief A simple message passing utility with minimized copying overhead.
 @author Shingo W. Kagami swk(at)ic.is.tohoku.ac.jp
 */

#ifndef _MSGLINK_HPP_
#define _MSGLINK_HPP_

#include <boost/thread.hpp>

/**
 * An interface class for the message data to be passed by MsgLink.
 * Data members are to be added in the subclass of this. 
 */
class MsgData {
    template<class T> friend class MsgLink;
public:

    /**
     * Constructor of the message data to be overridden if needed.
     */
    MsgData() : seqno(0) {};

    /**
     * Destructor of the message data to be overridden if needed.
     */
    virtual ~MsgData() {};

    /**
     * Method to be overridden when a part of the message data needs
     * actual copy (By default, the pointers to the message data are
     * exchanged instead of actual data copy.  If this pointer
     * exchange is the desired behavior, this method does not neeed to
     * be overridden).
     * 
     * @param dst Pointer to the destination message data
     */
    virtual void copyTo(MsgData *dst) {}

private:
    unsigned int seqno;
};


/**
 * Class template of the message passing link. 
 *
 * @tparam CustomMsgData A subclass of MsgData passed by the link.
 */
template<class CustomMsgData>
class MsgLink {
public:
    /**
     * Constructor, in which the three CustomMsgData, namely for the
     * sender's buffer, intermediate bufer and the receiver's buffer,
     * are initialized.
     */
    MsgLink() : master_seqno(0), closed(false) {
        md_snd = new CustomMsgData();
        md_med = new CustomMsgData();
        md_rcv = new CustomMsgData();
    }

    /**
     * Destructor, in which the three CustomMsgData are deleted.
     */
    ~MsgLink() {
        delete md_snd;
        delete md_med;
        delete md_rcv;
    }

    /**
     * Returns the pointer to the sender's CustomMsgData buffer so
     * that the message data to be sent can be put into it.
     */
    CustomMsgData *prepareMsg() const {
        return md_snd;
    }

    /**
     * Send the message by exchanging the pointers to the sender's
     * buffer and the intermediate buffer, followed by calling the
     * copyTo() method.
     */
    void send() {
        master_seqno++;
        boost::mutex::scoped_lock lock(mt);
        md_snd->seqno = master_seqno;
        swapPtr(&md_snd, &md_med);
        md_med->copyTo(md_snd);
    }

    /**
     * Returns true iff the message in the intermediate buffer is
     * newer than the one in the receiver's buffer.
     */
    bool isUpdated() const {
        return md_med->seqno > md_rcv->seqno;
    }

    /**
     * Receive the message by exchanging the pointers to the
     * intermediate buffer and the receiver's buffer.  If not
     * isUpdated(), the pointers are not exchanged. 
     * Returns the pointer to the receiver's buffer after exchange if
     * isUpdated(); NULL otherwise.
     */
    CustomMsgData *receive() {
        if (!isUpdated()) {
            return NULL;
        }
        boost::mutex::scoped_lock lock(mt);
        swapPtr(&md_med, &md_rcv);
        return md_rcv;
    }

    /**
     * Returns true iff the link has been closed.
     */
    bool isClosed() const {
        return closed;
    }

    /**
     * Close the link.
     */
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
