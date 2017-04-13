#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netdb.h>
#include <fcntl.h>
#include <errno.h>
#include <netinet/in.h>
#include <arpa/inet.h>
//#include <strings.h>
//#include <sys/wait.h>

#include "60t.h"
#include "60t-usb.h"

#ifdef verbose
#include <syslog.h>
    #define log(...) syslog(LOG_INFO, __VA_ARGS__)
    #define debug_printf(...) syslog(LOG_INFO, __VA_ARGS__)
#else
    #define log(...) 
    #define debug_printf(...)
#endif

#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif

#define server_id "Server: 60t\r\n"

#if openwrtbuild
    #define wwwdir "/www/"
#else
    #define wwwdir "./"
#endif

// todo: on fatal do cleanup

unsigned short process_request(int);
void fatal(const char *);
void send_file(int, const char *, const char*);

int lock_fd;
libusb_device_handle* dev = 0;

/* cmd */
unsigned char cmd = cmd_set;
int pwm_left = 0;
int pwm_right = 0;
unsigned char pl1 = 0;
unsigned char pl2 = 0;
unsigned char pr1 = 0;
unsigned char pr2 = 0;
unsigned char buffer[32];

libusb_device_handle* get_usb_device() { 
    
    int res;
    libusb_device_handle* handle = 0;

    res = libusb_init(0);
    if (res != 0) {
        printf("Error initialising libusb.\n");
        return 0;
    }

    handle = libusb_open_device_with_vid_pid(0, shared_vid, shared_pid);
    if (!handle) {
        printf("Unable to open device\n");
        return 0;
    }

#if 1
    if (libusb_kernel_driver_active(handle, 0)) {
        res = libusb_detach_kernel_driver(handle, 0);
        if (res != 0) {
          return 0;
        }
    }
#endif

    res = libusb_claim_interface(handle, 0);
    if (res != 0) {
        printf("Error claiming interface.\n");
        libusb_close(handle);
        return 0;
    }

    return handle;
}

int usb_receive (libusb_device_handle * handle,
        char * buffer,
        size_t size) {
    return libusb_control_transfer(
        handle, 
        usb_rq_in, 
        cmd_get,
        0,
        0,
        (char *)buffer,
        size,
        usb_timeout);
}

int usb_send (libusb_device_handle * handle,
        char * buffer,
        size_t size) {
    return libusb_control_transfer(
        handle, 
        usb_rq_out, 
        cmd_set,
        0,
        0,
        (char*) buffer,
        size,
        usb_timeout);
}

void close_usb_device(libusb_device_handle* handle) {
    libusb_close(handle);
}

void reset_usb_device() {
    if (dev) {
        close_usb_device(dev);
    }
    dev = get_usb_device();
}

void compose_command() {
    if (pwm_left < 0) {
        pwm_left = -pwm_left;
        pl1 = 0;
        pl2 = 1;
    } else if (pwm_left > 0) {
        pl1 = 1;
        pl2 = 0;
    } else {
        pl1 = 0;
        pl2 = 0;
    }

    if (pwm_right < 0) {
        pwm_right = -pwm_right;
        pr1 = 0;
        pr2 = 1;
    } else if (pwm_right > 0) {
        pr1 = 1;
        pr2 = 0;
    } else {
        pr1 = 0;
        pr2 = 0;
    }

    pwm_left &= 0xFF;
    pwm_right &= 0xFF;

    if (cmd_get == cmd) {
        // todo: ???
        buffer[0] = 0;
    } else if (cmd_set == cmd) {
        buffer[0] = 1;
        buffer[1] = pwm_right;
        buffer[2] = pr1;
        buffer[3] = pr2;
        buffer[4] = 2;
        buffer[5] = pwm_left;
        buffer[6] = pl1;
        buffer[7] = pl2;
    }
}

void usb_transaction() {
    int bytes_read;
    if (cmd_set == cmd) {
        bytes_read = usb_send(dev, buffer, 8);
        log("%d bytes sent", bytes_read);
    } else if (cmd_get == cmd) {
        bytes_read = usb_receive(dev, buffer, report_size);
        log("%d bytes got\n", bytes_read);
        log("GOT THIS: %x %x %x %x\n",
                buffer[0], buffer[1], buffer[2], buffer[3]);
    }
}

// 0 - no command to execute
// 1 - there is a command
unsigned short process_request(int client) {
    char buf[1024];
    char request[128];
    char *query_string;
    unsigned short command_ready = 0;
    char * lines;

    if (read(client, buf, 1024) == -1) {
        fatal("recive");
    }

    lines = strtok(buf, "\n\r");
    
    do {
        // GET /sdfsdf/?sfasfasf=45555 HTTP/1.1
        if (0 == strncasecmp(lines, "GET", 3)) {
            strncpy(request, lines, 128);
            request[127] = '\0';
        }
        debug_printf("-->%s\n", lines);
        lines = strtok(NULL, "\n\r");
    } while (lines);

    debug_printf("R = %s\n", request);

    if (!request[0]) {
        goto html;
    }
    
    // split GET request
    lines = strtok(request, " ");
    if (!lines) {
        goto html;
    }
    
    // expecting resource line
    lines = strtok(NULL, " ");
    if (!lines) {
        goto html;
    }
    
    // checking query string
    query_string = strstr(lines, "?");

    if (!query_string) {
        // check if .js requested:
        int len = strlen(lines);
        if (len > 3 && !strcmp(lines + len - 3, ".js")) {
            debug_printf("javascript: %s\n", lines);
            goto javascript;
        }
        goto html;
    }

    debug_printf("Q = %s\n", &(query_string[1]));

    lines = &(query_string[1]);

    char *current_qry, *current_token, *token, *key, *value;
    char *saveptr1, *saveptr2;
    int j;

    for (current_qry = lines; ; current_qry = NULL) {
        
        token = strtok_r(current_qry, "&", &saveptr1);
        if (token == NULL)
            break;

        current_token = token;

        key = strtok_r(current_token, "=", &saveptr2);
        
        if (key == NULL) {
            continue;
        }
        
        value = strtok_r(NULL, "=", &saveptr2);
        
        if (key == NULL) {
            continue;
        }

        debug_printf ("k=%s  v=%s\n", key, value);

        if ( 0 == strncmp(key,"cmd",3)) {
            if (0 == strncmp(value, "get", 3)) {
                debug_printf(" command = get\n");
                cmd = cmd_get;
            } else if (0 == strncmp(value, "set", 3)) {
                debug_printf(" command = set\n");
                cmd = cmd_set;
            } else if (0 == strncmp(value, "rst", 3)) {
                debug_printf(" command = reset\n");
                cmd = cmd_rst;
            }
        }
        else if (0 == strcmp(key, "r")) {
            pwm_right = atoi(value);
        }
        else if (0 == strcmp(key, "l")) {
            pwm_left = atoi(value);
        }
    }

//ajax:
    //todo
    sprintf(buf, "HTTP/1.0 200 OK\r\n" server_id);
    send(client, buf, strlen(buf), 0);
    sprintf(buf, "Content-Type: text/plain\r\n\r\n60t\n");
    send(client, buf, strlen(buf), 0);
    command_ready = 1;
    goto end;

// todo: absolute paths ???
javascript:
    send_file(client, wwwdir "60t.js", "Content-type: application/javascript\r\n");
    goto end;

html:
    send_file(client, wwwdir "60t.html" , "Content-Type: text/html\r\n");

end:
    close(client);
    return command_ready;
}

void fatal(const char *msg) {
    printf(msg);
    exit(1);
}

void send_file(int client, const char *filename, const char* type) {
    FILE *resource = NULL;
    char buf[1024];

    sprintf(buf, "HTTP/1.0 200 OK\r\n" server_id);
    send(client, buf, strlen(buf), 0);
    sprintf(buf, type);
    send(client, buf, strlen(buf), 0);
    sprintf(buf, "\r\n");
    send(client, buf, strlen(buf), 0);

    resource = fopen(filename, "r");
    if (resource == NULL) {
        printf("error fopen\n");
        return;
    }

    fgets(buf, sizeof(buf), resource);
    while (!feof(resource)) {
        send(client, buf, strlen(buf), 0);
        fgets(buf, sizeof(buf), resource);
    }

    fclose(resource);
}

void httpd_loop(void) {

    int server_sock = -1;
    u_short port = httpd_port;
    int client_sock = -1;
    struct sockaddr_in client_name;
    socklen_t client_name_len = sizeof(client_name);

    struct sockaddr_in name;

    server_sock = socket(PF_INET, SOCK_STREAM, 0);
    if (server_sock == -1)
        fatal("socket");

    memset(&name, 0, sizeof(name));
    name.sin_family = AF_INET;
    name.sin_port = htons(port);
    name.sin_addr.s_addr = htonl(INADDR_ANY);
    
    if (bind(server_sock, (struct sockaddr *)&name, sizeof(name)) < 0)
        fatal("bind");

    if (listen(server_sock, 5) < 0)
        fatal("listen");
    
    while (1) {
        
        client_sock = accept(
                server_sock,
                (struct sockaddr *) &client_name,
                &client_name_len);
        
        if (client_sock == -1)
            fatal("accept");

        if (process_request(client_sock)) {
            if (cmd == cmd_rst) {
                reset_usb_device();
            } else {
                compose_command();
                usb_transaction();
            }
        }
    
    }

    close(server_sock);

}

void kill_previous_instance () {
    int lock_fd;
    char pid_str[10];
    int pid, i;
    errno = 0;
    lock_fd = open(lock_file, O_RDONLY);
    if (0 < lock_fd) {
        /* lock file exists */
        if (0 != lockf(lock_fd, F_TEST, 0)) {
            /* lock file exists + locked */
            /* read PID of locking process */
            memset(pid_str, 0, sizeof(pid_str));
            i = read(lock_fd, pid_str, sizeof(pid_str));
            if (0 < i) {
                pid = atoi(pid_str);
                if (pid) {
                    (void)kill(pid, SIGTERM);
                    /* waiting 7 seconds for previous instance ended */
                    i = 7;    
                    while (0 < i) {
                        if (0 == lockf(lock_fd, F_TEST, 0)) {
                            close(lock_fd);
                            return;
                        }
                        sleep(1);
                        i -= 1;
                    }
                    close(lock_fd);
                    log("Error killing previous instance %d\nPossibly started by other user\n", pid);
                    exit(1);
                }
            }
            /* garbage in file */
            close(lock_fd);
            log("Error reading lock file %s\nTerminating\n", lock_file);
            exit(1);
    }
    else {
            /* lock file exists + NOT locked */
        }
    }
}

int create_pid_file() {
    char pid_str[10];
    int lock_fd;
    lock_fd = open(lock_file, O_RDWR|O_CREAT|O_TRUNC, S_IRUSR|S_IWUSR|S_IROTH|S_IWOTH|S_IRGRP|S_IWGRP);
    if (0 == lockf(lock_fd, F_TLOCK, 0)) {
        memset(pid_str, 0, sizeof(pid_str));
        sprintf(pid_str, "%d", (int)getpid());
        if (0 < write(lock_fd, pid_str, strlen(pid_str)))
            return lock_fd;
    }
    close(lock_fd);
    return 0;
}

void signal_handler(int sgn) {
    //todo: close sockets and usb device
     if (0 == lockf(lock_fd, F_ULOCK, 0)) {
        close(lock_fd);
     }
     exit(0);
}

int main(int argc, char **argv) {
	
    int fd, fd_server;
    int bytes_read;
    int sz = client_buffer_size;
    char buffer[client_buffer_size];

    
    pid_t pid, sid;
    int op, result;

    while (-1 != (op = getopt (argc, argv, "x"))) {
        switch (op) {
        case 'x':
            kill_previous_instance();
            return 0;
            break;
        default:
            break;
        }
    }

    pid = fork();
    if (pid < 0) {
        exit(1);
    }
    if (pid > 0) {
        exit(0);
    }
    sid = setsid();
    if (sid < 0) {
        exit(2);
    }

    umask(0);

    kill_previous_instance();

    lock_fd = create_pid_file();
    if (!lock_fd) {
        return 1;
    }

    signal(SIGTERM, signal_handler);

#if 1
    close(STDIN_FILENO);
    close(STDOUT_FILENO);
    close(STDERR_FILENO);
#endif

    dev = get_usb_device();

    httpd_loop();

    return 0;
}

