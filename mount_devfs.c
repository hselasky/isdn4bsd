#include <stdio.h>
#include <sys/param.h>
#include <sys/mount.h>
#include <errno.h>

int main()
{
    int error;

    error = mkdir("/cdev", 0700);
    if(error && (errno != EEXIST))
    {
        printf("mkdir error: %s!\n", 
	       strerror(errno));
    }

    error = mount("devfs", "/cdev", 0, NULL);
    if(error)
    {
        printf("mount error: %s!\n",
	       strerror(errno));
    }

    return 0;
}
