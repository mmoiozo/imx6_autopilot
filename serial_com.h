
extern void uart_init();
extern void uart_init_nc();
extern void uart_send();
extern void uart_read();
extern void uart_read_simple();
extern void uart_read_nc(char *received);
void esp8266_init();
void send_string(char out_buffer[]);
void esp8266_send(int length);
extern int uart0_filestream;
char gain_recv;
void gain_send();

void uart_read_old(char *received);

extern int16_t x_com;
extern int16_t y_com;
extern int16_t t_com;
extern int16_t r_com;
extern int16_t rec_com;

extern int debug_send(int16_t x_angle, int16_t y_angle, int16_t alt, int16_t loop_rate, int16_t connected);