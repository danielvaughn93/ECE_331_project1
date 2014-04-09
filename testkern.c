int main(int argc, char *argv[])
{

        int fd;

        //test opening kernal
        fd = open("/dev/lcd", O_RDONLY);

        //Error Checking
        if (fd < 0) {
                perror("Failed init: ");
                return -1;
        }
        printf("%d\n", fd);
        //Test closing kernal if opening worked.
        close(fd);
        return 0;
}