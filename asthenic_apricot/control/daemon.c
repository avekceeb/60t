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

#include "60t.h"
#include "60t-usb.h"

/* web stuff */
const char * mimehtml = "text/html";
const char * mimecss = "text/css";
const char * mimejs = "application/javascript";
const char * mimejson = "application/json";
const char * mimetext = "text/plain"; /* let it be the default */

#define welcome_page "60t.html"

#define server_id "Server: 60t\r\n"

#if openwrtbuild
    #define wwwdir "/www"
#else
    #define wwwdir "."
#endif

unsigned short process_request(int);
int send_file(int, const char *);
void send_json(int);

#if demonize
    #include <syslog.h>
    #define log(...) syslog(LOG_INFO, __VA_ARGS__)
    #define fatal(...) while(1){syslog(LOG_ERR, __VA_ARGS__);exit(3);}
#else
    #define log(...) printf(__VA_ARGS__)
    #define fatal(...) while(1){printf(__VA_ARGS__);exit(3);}
#endif

#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif

int server_sock = -1;
int client_sock = -1;

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
// todo : structure
unsigned char usb_buffer[32];

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
        usb_buffer[0] = 0;
    } else if (cmd_set == cmd) {
        usb_buffer[0] = 1;
        usb_buffer[1] = pwm_right;
        usb_buffer[2] = pr1;
        usb_buffer[3] = pr2;
        usb_buffer[4] = 2;
        usb_buffer[5] = pwm_left;
        usb_buffer[6] = pl1;
        usb_buffer[7] = pl2;
    }
}

/********* usb ********************/

libusb_device_handle* get_usb_device() { 
    
    int res;
    libusb_device_handle* handle = 0;

    res = libusb_init(0);
    if (res != 0) {
        log("Error initialising libusb.\n");
        return 0;
    }

    handle = libusb_open_device_with_vid_pid(0, shared_vid, shared_pid);
    if (!handle) {
        log("Unable to open device\n");
        return 0;
    }

    if (libusb_kernel_driver_active(handle, 0)) {
        res = libusb_detach_kernel_driver(handle, 0);
        if (res != 0) {
            log("Unable to detach kernel driver\n");
          return 0;
        }
    }

    res = libusb_claim_interface(handle, 0);
    if (res != 0) {
        log("Error claiming interface.\n");
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

void usb_transaction() {
    
    int bytes_read;
    
    if (!dev) {
        return;
    }

    log("===>\n");
    
    if (cmd_set == cmd) {
    
        bytes_read = usb_send(dev, usb_buffer, 8);
        log("%d bytes sent\n", bytes_read);
    
    } else if (cmd_get == cmd) {
        
        bytes_read = usb_receive(dev, usb_buffer, report_size);
        log("%d bytes got\n", bytes_read);
        log("buffer: %x %x %x %x\n",
                usb_buffer[0], usb_buffer[1], usb_buffer[2], usb_buffer[3]);
    
    }

    log("===>\n\n");
}

/*******  web ***************/

const char * suggest_mime_type(const char* hint) {
    if (!hint) {
        return 0;
    }
    int len = strlen(hint);
    if (len > 5 && !strcmp(hint + len - 5, ".html")) {
        return mimehtml;
    }
    if (len > 4 && !strcmp(hint + len - 4, ".css")) {
        return mimecss;
    }
    if (len > 3 && !strcmp(hint + len - 3, ".js")) {
        return mimejs;
    }
    return 0; // text???
}

// 0 - no command to execute
// 1 - there is a command
unsigned short process_request(int client) {
    char buf[1024];
    char request[128];
    char *query_string;
    unsigned short command_ready = 0;
    char * lines;

    log("--->\n");

    if (read(client, buf, 1024) == -1) {
        fatal("fatal: receive\n");
    }

    // GET /blah/blah?x=4&y=-10&z=yes HTTP/1.1
    lines = strtok(buf, "\n\r");
    
    do {
        if (0 == strncasecmp(lines, "GET", 3)) {
            strncpy(request, lines, 128);
            request[127] = '\0';
        }
        lines = strtok(NULL, "\n\r");
    } while (lines);

    if (!request || !request[0]) {
        goto welcome;
    }
    
    log("request: %s\n", request);

    // split GET request
    lines = strtok(request, " ");
    if (!lines) {
        goto welcome;
    }
    
    // expecting resource line
    lines = strtok(NULL, " ");
    if (!lines) {
        goto welcome;
    }

    log("resource requested: %s\n", lines);

#if 1
    if (!strcmp(lines, "/jsondata")) {
        send_json(client);
        goto end;
    }
#endif

    if (send_file(client, lines)) {
        goto end;
    }

    // checking query string
    query_string = strstr(lines, "?");

    if (!query_string || 2 > strlen(query_string)) {
        goto welcome;
    }

    log("query: %s\n", &(query_string[1]));

    lines = &(query_string[1]);

    char *current_qry, *current_token, *token, *key, *value;
    char *saveptr1, *saveptr2;
    int j;

    for (current_qry = lines; ; current_qry = NULL) {
        
        token = strtok_r(current_qry, "&", &saveptr1);
        if (!token)
            break;

        current_token = token;

        key = strtok_r(current_token, "=", &saveptr2);
        
        if (!key) {
            continue;
        }
        
        value = strtok_r(NULL, "=", &saveptr2);
        
        if (!key) {
            continue;
        }

        log("parameter: %s = %s\n", key, value);

        if (0 == strncmp(key, "cmd", 3)) {
            if (0 == strncmp(value, "get", 3)) {
                log("command = get\n");
                cmd = cmd_get;
            } else if (0 == strncmp(value, "set", 3)) {
                log("command = set\n");
                cmd = cmd_set;
            } else if (0 == strncmp(value, "rst", 3)) {
                log("command = reset\n");
                cmd = cmd_rst;
            }
        }
        else if (0 == strcmp(key, "r")) {
            pwm_right = atoi(value);
        }
        else if (0 == strcmp(key, "l")) {
            pwm_left = atoi(value);
        }
        // todo: ???
        command_ready = 1;
    }

welcome:
    if (!send_file(client, welcome_page)) {
        // last resort:
        sprintf(buf, "HTTP/1.0 200 OK\r\nContent-Type: text/plain\r\n\r\n60t\n");
        send(client, buf, strlen(buf), 0);
        goto end;
    }

end:
    close(client);
    log("<---\n\n");
    return command_ready;
}

int send_file(int client, const char *filename) {
    FILE *resource = 0;
    char buf[1024];

    const char * mime = suggest_mime_type(filename);
    if (!mime) {
        return 0;
    }

    sprintf(buf, wwwdir);
    // todo: check "/" at the beginning
    strcat(buf, filename);

    resource = fopen(buf, "r");
    if (!resource) {
        log("error fopen\n");
        return 0;
    }

    log("file = %s\n", buf);
    
    memset(buf, 0, 1024);

    sprintf(buf, "HTTP/1.0 200 OK\r\n" server_id);
    send(client, buf, strlen(buf), 0);
    sprintf(buf, "Content-type: %s\r\n\r\n", mime);

    send(client, buf, strlen(buf), 0);

    fgets(buf, sizeof(buf), resource);
    while (!feof(resource)) {
        send(client, buf, strlen(buf), 0);
        fgets(buf, sizeof(buf), resource);
    }

    fclose(resource);

    return 1;
}

#define jsoncount 128
void send_json(int client) {

    int i;
    int data[jsoncount];
    char buf[4096];

    for (i=0; i<jsoncount; i++) {
        data[i] = (i-(jsoncount>>1)) * (i-(jsoncount>>1));
    }
    
    sprintf(buf, "HTTP/1.0 200 OK\r\nContent-Type: %s\r\n\r\n", mimejson);
    send(client, buf, strlen(buf), 0);
    
    sprintf(buf, "[\n{\"id\":-100,\"name\":\"Parabola\",\"data\":[\n");
    send(client, buf, strlen(buf), 0);

    for (i=0; i<jsoncount; i++) {
        sprintf(buf, "{\"x\":%d,\"y\":%d}%s\n", i, data[i], i==jsoncount-1 ? "":",");
        send(client, buf, strlen(buf), 0);
    }
    sprintf(buf, "]\n}\n]");
    send(client, buf, strlen(buf), 0);
}

void httpd_loop(void) {

    u_short port = httpd_port;
    struct sockaddr_in client_name;
    socklen_t client_name_len = sizeof(client_name);

    struct sockaddr_in name;

    server_sock = socket(PF_INET, SOCK_STREAM, 0);
    if (server_sock == -1)
        fatal("ERROR: socket\n");

    memset(&name, 0, sizeof(name));
    name.sin_family = AF_INET;
    name.sin_port = htons(port);
    name.sin_addr.s_addr = htonl(INADDR_ANY);
    
    if (bind(server_sock, (struct sockaddr *)&name, sizeof(name)) < 0)
        fatal("ERROR: bind\n");

    if (listen(server_sock, 5) < 0)
        fatal("ERROR: listen\n");
    
    while (1) {
        
        client_sock = accept(
                server_sock,
                (struct sockaddr *) &client_name,
                &client_name_len);
        
        if (client_sock == -1)
            fatal("ERROR: accept\n");

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

/*********** daemon *******************/

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
    log("got signal %d\n", sgn);
    close(server_sock);
    close(client_sock);
    if (0 == lockf(lock_fd, F_ULOCK, 0)) {
        close(lock_fd);
    }
    exit(5);
}

/************************************/

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

#if demonize
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
#endif

    lock_fd = create_pid_file();
    if (!lock_fd) {
        return 1;
    }

    signal(SIGTERM, signal_handler);
#if ! demonaize
    signal(SIGINT, signal_handler);
#endif

#if demonize
    close(STDIN_FILENO);
    close(STDOUT_FILENO);
    close(STDERR_FILENO);
#endif

    dev = get_usb_device();

    httpd_loop();

    return 0;
}

