#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

int vmc_fd;

// DRENG need to add more EBUSY checking

int vmc_open() {
	int rc;

	do {
		vmc_fd = open("/dev/ibmvmc", O_RDWR);
		printf("vmc_open: vmc_fd = 0x%x, rc = 0x%x, errno = 0x%x\n", vmc_fd, rc, errno);
	} while ((rc == -1) && (errno == EBUSY));

	printf("open: complete.\n");
	return vmc_fd;
}

int vmc_send_hmc_id() {
	char hmc_id[32] = "i am an hmc id";
	int rc;

	do {
		rc = ioctl(vmc_fd, 1, hmc_id);
	} while ((rc == -1) && (errno == EBUSY));

	printf("vmc_send_hmc_id: rc = 0x%x\n", rc);
}

int vmc_send_msg() {
	char msg[32] = "this is an hmc msg";
	int rc;
	int c = 0;

	do {
		rc = write(vmc_fd, msg, 32);
#if 0
                if(c == 0)
			printf("vmc_send_msg: rc = 0x%x, errno = %d\n", rc, errno);
#endif
		c++;
		if(c == 100) c = 0;
	} while ((rc == -1) && (errno == EBUSY));

	printf("vmc_send_msg: complete.\n");
}

main () {
	printf("Starting VMC test\n");

	if(vmc_open() >= 0) {
		vmc_send_hmc_id();
		vmc_send_msg();
	} else {
		printf("vmc open failed: %d\n", errno);
	}
}

