
/* ------------------------ FreeRTOS includes ----------------------------- */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "http_server.h"

/* ------------------------ lwIP includes --------------------------------- */
#include "lwip/api.h"
#include "lwip/tcpip.h"

/* ------------------------ Project includes ------------------------------ */
#include "ftmac100.h"

static void vDemoAppsTask( void *pvParameters );

/**
 * Add a network interface
 */
void apps_init( void )
{
	vSerialPutString("INIT\n");
	vSerialPutString("...\n");
	if( sys_thread_new( "demo-apps", vDemoAppsTask, NULL, 1024, (tskIDLE_PRIORITY + 1) ) == NULL )
		printf( "apps_init: create task failed!\n");

	vSerialPutString("OK\n");
}


static void vDemoAppsTask( void *pvParameters )
{
	struct ip_addr  xIpAddr, xNetMast, xGateway;
	static struct netif ftmac100_if;

	vSerialPutString("TASK\n");
	/* The parameters are not used in this function. */
	( void ) pvParameters;

#if 1
	/* Init lwip library */
	tcpip_init( NULL, NULL );

	/* Create and configure the EMAC interface. */
	IP4_ADDR( &xIpAddr, 192, 168, 68, 70 );
	IP4_ADDR( &xNetMast, 255, 255, 255, 0 );
	IP4_ADDR( &xGateway, 192, 168, 68, 66 );
	netif_add( &ftmac100_if, &xIpAddr, &xNetMast, &xGateway, NULL, ftMac100_init, tcpip_input );

	/* make it the default interface */
	netif_set_default( &ftmac100_if );

	/* bring it up */
	netif_set_up( &ftmac100_if );

	/* link is up */
	netif_set_link_up( &ftmac100_if );

//        lwip_init();
#endif
//	if( sys_thread_new( "httpd", http_server_netconn_thread, NULL, 256, ( tskIDLE_PRIORITY + 2 ) ) == NULL )
//		printf( "apps_init: create task failed!\n");
	vSerialPutString("loop\n");
	for ( ;; )
	{
		/* do nothing, delay very long time, let other tasks
		   to run */
		vTaskDelay( 0xffff );
	}
}
