// commands to avr
#define cmd_set 0xA0
#define cmd_get 0xB0
// commands to wrt
#define cmd_rst 0xC0

#define report_size 4 
#define client_buffer_size 64

#define lsnr_fifo "/tmp/60t.lsnr"
#define rply_fifo "/tmp/60t.rply"
#define lock_file "/tmp/60t.lock"

#define httpd_port 60607
