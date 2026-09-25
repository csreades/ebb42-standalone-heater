#ifndef USB_H_
#define USB_H_
void usb_init(void);
void usb_task(void);
void usb_write(const char *s);
int  usb_read_char(void);            /* -1 if nothing pending */
void usb_request_bootloader(void);
void usb_check_bootloader_request(void);
#endif
