//standard includes

#include <iostream>
#include <errno.h>
#include <stdio.h>

#include <stdlib.h>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>

// Other includes
#ifndef _HVHMCLESSCMD_H
    #include "HvHmcLessCmd.H"
#endif

using namespace std;

/** Return zero on success */
int writeMessage(int fd, HvHmcLessCmd *outbound)
{
    int rc;
    do {
        rc = write(fd ,(char*)outbound , outbound->getLength());
    } while ((rc == -1) && ((errno == EBUSY) || (errno == EAGAIN)));

    if (rc == -1) {
        cout << "write failed errno: " << errno << "  " << perror << endl;
        return -1;
    } else if (rc != outbound->getLength()) {
        cout << "Write failed msglen " << rc << " != " << outbound->getLength() << endl;
        return -2;
    }
    return 0;
}

int readMessage(int fd, HvHmcLessCmd *inbound)
{
    int rc;
    do {
        rc = read (fd , (char*)inbound, sizeof(*inbound) );
    } while ((rc == -1) && ((errno == EBUSY) || (errno == EAGAIN)));

    if (rc == -1) {
        cout << "Read failed errno" << errno<< "  " << perror << endl;
        return -1;
    } else if (rc != inbound->getLength()) {
        cout << "Read failed msglen " << rc << " != " << inbound->getLength() << endl;
        return -2;
    }
    return 0;
}

struct vmc_query_struct
{
    int have_vmc;
    int state;
};

int query(int fd)
{
    int rc = 0;
    struct vmc_query_struct query_struct;
    memset(&query_struct, 0, sizeof(query_struct));

    do {
        rc = ioctl(fd, 2, &query_struct);
        cout << "ioctl called: rc=" << rc << endl;
    } while ((rc == -1) && ((errno == EBUSY) || (errno == EAGAIN)));

    cout << "query_struct: have_vmc=" << query_struct.have_vmc
	<< " state=" << query_struct.state << endl;

    return rc;
}

int main(int argc, char *argv[])
{
    const char* devname = "/dev/ibmvmc" ;// put something here
    int loopcount = 100;
    int fd;
    int rc;
    HvHmcLessCmd* inbound;
    HvHmcLessCmd* outbound;
    char hmc_id[32] = "hmcless id";

    cout << "calling open" << endl;

    do {
        fd = open (devname , O_RDWR);
        cout << "open called: fd=" << fd << endl;
    } while ((fd == -1) && ((errno == EBUSY) || (errno == EAGAIN)));

    if (fd == -1) {
        cout << "error opening file   " << errno << "  " << perror  << endl ;
        exit (1);
    }
    cout << "open complete" << endl;

    rc = query(fd);
    if (rc == -1) {
	cout << "error querying state" << endl;
	exit(1);
    }

    do {
        rc = ioctl(fd, 1, hmc_id);
        cout << "ioctl called: rc=" << rc << endl;
    } while ((rc == -1) && ((errno == EBUSY) || (errno == EAGAIN)));

    if (rc == -1) {
        cout << "error opening vmc for my hmc_id " << errno << "  " << perror  << endl ;
        exit (2);
    }

    inbound = new HvHmcLessCmd(HvHmcLessCmd::CatHmcRtr);
    outbound = new HvHmcLessCmd(HvHmcLessCmd::CatHmcRtr);
    outbound->setOpcode(HvHmcLessCmd::ExchangeOps);
    outbound->setParmsByte(1,1);
    outbound->setDataLen(6);

    cout << "sending the open command" << endl;
    outbound->dumpHex();

    if (writeMessage(fd,outbound) != 0) {
        exit(3);
    }


    if (readMessage(fd,inbound) != 0) {
        exit(4);
    }

    cout << "got the open response" << endl;
    inbound->dumpHex();

    cout << "cmd router session is now open" << endl;

    outbound->setOpcode(HvHmcLessCmd::cmdRtrEcho);
    for (int count=0 ; count <loopcount ; count++) {
        outbound->setParmsByte(count, count+1);
        outbound->setDataLen(count+1); // increase the parm length for each msg
	cout << "ping: " << count << endl;
//	outbound->dumpHex();
        if (writeMessage(fd,outbound) != 0) {
            exit(5);
        }


        if (readMessage(fd,inbound) != 0) {
            exit(4);
        }

	cout << "ping response" << endl;
//	inbound->dumpHex();

    }

    outbound->setDataLen(0);
    outbound->setOpcode(HvHmcLessCmd::Close);

    cout << "sending the close command" << endl;
    outbound->dumpHex();
    if (writeMessage(fd,outbound) != 0) {
        exit(3);
    }

    if (readMessage(fd,inbound) != 0) {
        exit(4);
    }

    cout << "got the close response" << endl;
    inbound->dumpHex();
    cout << "cmd router session is now closed" << endl;

    if (close (fd) == -1) {
        cout << "error closing file   " << errno << "  " << perror << "\n" ;
        exit (1);
    }


}
