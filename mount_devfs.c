#include <stdio.h>
#include <sys/param.h>
#include <sys/mount.h>
#include <errno.h>

int main()
{
    int error;

/*    error = mkdir("/cdev", 0700);
    if(error && (error != EEXIST))
    {
        printf("mkdir error: %s!\n", 
	       strerror(error));
    }
*/
    error = mount("devfs", "/cdev", 0, NULL);
    if(error)
    {
        printf("mount error: %s!\n",
	       strerror(error));
    }

    return 0;
}
