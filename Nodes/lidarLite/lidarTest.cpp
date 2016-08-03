#include "lidarLite.h"
#include <time.h>

int er(int argc,char *argv[]){
	
	int fd, res, del;
	unsigned char st;
	// First arg is delay in ms (default is 1000)
	if (argc > 1){ 
	   del = (uint)(atoi(argv[1])*1000);
   }
	else{
		 del=2000;
	}
    fd = lidar_init(false);
    if (fd == -1) {
        printf("initialization error\n");
        }
    else {
        for (;;) {
            res = lidar_read(fd);
            st = lidar_status(fd);
            //ver = lidar_version(fd);
            printf("\n");
            printf("%3.0d cm \n", res);
            lidar_status_print(st);
			printf("\n");
			usleep(del);
        }
	}
	return 0;
}
