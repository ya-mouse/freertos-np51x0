#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"

void http_server_serve(int sd);
void http_server_netconn_thread();