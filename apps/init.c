
/* ------------------------ FreeRTOS includes ----------------------------- */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

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
	if( sys_thread_new( "demo-apps", vDemoAppsTask, NULL, 1024, (tskIDLE_PRIORITY + 1) ) == NULL )
		printf( "apps_init: create task failed!\n");
}


static void vDemoAppsTask( void *pvParameters )
{
	struct ip_addr  xIpAddr, xNetMast, xGateway;
	static struct netif ftmac100_if;

	/* The parameters are not used in this function. */
	( void ) pvParameters;

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

	for ( ;; )
	{
		/* do nothing, delay very long time, let other tasks
		   to run */
		vTaskDelay( 0xffff );
	}
}
