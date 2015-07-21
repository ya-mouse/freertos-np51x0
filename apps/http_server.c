#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/sys.h"
#include <stdio.h>
#include <string.h>
#include "http_server.h"

void http_server_serve(int sd)
{
        int read_len,j;
        int RECV_BUF_SIZE = 1400;
        char recv_buf[1400];

        /* read in the request */
        simple_printf("Server request\n");
        if ((read_len = read(sd, recv_buf, RECV_BUF_SIZE)) < 0) {
                close(sd);
#ifdef OS_IS_FREERTOS
                vTaskDelete(NULL);
#endif
                return;
        }

        /* respond to request */
        strcpy(recv_buf, "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n");
        strcpy(recv_buf, "<html><body><h1>NP51x0: It works!</h1></body></html>\n");
//        generate_response(sd, recv_buf, read_len);
        lwip_write(sd, recv_buf, strlen(recv_buf));
        j = 0;
        while(j < 5000)
           j++;

        /* close connection */
        close(sd);
#ifdef OS_IS_FREERTOS
        vTaskDelete(NULL);
#endif
        return;
}

void http_server_netconn_thread()
{ 
    int sock;
    struct sockaddr_in address, remote;
    int size;

    if ((sock = lwip_socket(AF_INET, SOCK_STREAM, 0)) < 0) {
	simple_printf("lwip_socket failed\n");
	vTaskDelete(NULL);
        return;
    }

    address.sin_family = AF_INET;
    address.sin_port = htons(80);
    address.sin_addr.s_addr = INADDR_ANY;

    if (lwip_bind(sock, (struct sockaddr *)&address, sizeof(address)) < 0) {
	simple_printf("lwip_bind failed\n");
        vTaskDelete(NULL);
        return;
    }

    lwip_listen(sock, 0);

    size = sizeof(remote);
    while (1) {
        int new_sd = lwip_accept(sock, (struct sockaddr *)&remote, (socklen_t *)&size);

        /* serve connection */
        http_server_serve(new_sd);
    }
}