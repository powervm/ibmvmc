/* Copyright 2015, IBM
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
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

/* ioctl numbers */
#define VMC_BASE		0xCC
#define VMC_IOCTL_SETHMCID      _IOW(VMC_BASE, 0x00, unsigned char *)
#define VMC_IOCTL_QUERY         _IOR(VMC_BASE, 0x01, struct ibmvmc_ioctl_query_struct)
#define VMC_IOCTL_REQUESTVMC    _IOR(VMC_BASE, 0x02, unsigned int)

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


int request_vmc(int fd)
{
    int rc = 0;
    int vmc_index = 0;

    do {
        rc = ioctl(fd, VMC_IOCTL_REQUESTVMC, &vmc_index);
        cout << "requestvmc ioctl called: rc=" << rc << endl;
    } while ((rc == -1) && ((errno == EBUSY) || (errno == EAGAIN)));

    if (rc == -1) {
	cout << "vmc request failed errno: " << errno << "  " << perror << endl;
    }

    cout << "vmc_index=" << vmc_index << endl;

    return rc;
};

struct ibmvmc_ioctl_query_struct
{
    int have_vmc;
    int state;
    int vmc_drc_index;
};

int query(int fd, int &have_vmc)
{
    int rc = 0;
    struct ibmvmc_ioctl_query_struct query_struct;
    memset(&query_struct, 0, sizeof(query_struct));

    do {
        rc = ioctl(fd, VMC_IOCTL_QUERY, &query_struct);
        cout << "query ioctl called: rc=" << rc << endl;
    } while ((rc == -1) && ((errno == EBUSY) || (errno == EAGAIN)));

    if (rc == -1) {
	    cout << "query ioctl failed errno: " << errno << "  " << perror << endl;
    }

    cout << "query_struct: have_vmc=" << query_struct.have_vmc
	<< " state=" << query_struct.state
	<< " vmc_drc_index=" << query_struct.vmc_drc_index << endl;

    if (!rc) {
	have_vmc = query_struct.have_vmc;
    }

    return rc;
}

int main(int argc, char *argv[])
{
    const char* devname = "/dev/ibmvmc" ;// put something here
    int loopcount = 100;
    int fd;
    int rc;
    int have_vmc;
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

    // Perform initial query for VMC
    rc = query(fd, have_vmc);
    if (rc == -1) {
	cout << "error querying state" << endl;
	exit(1);
    }

    // If no VMC found, try and request one from phyp
    if (!have_vmc) {
	rc = request_vmc(fd);
	if (rc == -1) {
		cout << "error requesting vmc" << endl;
		exit(1);
	}

	// TODO: DLPAR add the VMC device here
	// Requery to check for VMC
	rc = query(fd, have_vmc);
	if (rc == -1) {
	    cout << "error querying state" << endl;
	    exit(1);
	}
	// Fail out if VMC still not available
	if (!have_vmc) {
	    cout << "No VMC.  Done." << endl;
	    exit(1);
	}
    }

    do {
        rc = ioctl(fd, VMC_IOCTL_SETHMCID, hmc_id);
        cout << "sethmcid ioctl called: rc=" << rc << endl;
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
