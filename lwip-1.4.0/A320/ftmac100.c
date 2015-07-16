/*
    FreeRTOS V7.4.2 - Copyright (C) 2013 Real Time Engineers Ltd.

    BingYao, Luo - Modified for Faraday FTMAC100. @2013-June

    FEATURES AND PORTS ARE ADDED TO FREERTOS ALL THE TIME.  PLEASE VISIT
    http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.

    >>>>>>NOTE<<<<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
    details. You should have received a copy of the GNU General Public License
    and the FreeRTOS license exception along with FreeRTOS; if not it can be
    viewed here: http://www.freertos.org/a00114.html and also obtained by
    writing to Real Time Engineers Ltd., contact details for whom are available
    on the FreeRTOS WEB site.

    1 tab == 4 spaces!

    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?"                                     *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************


    http://www.FreeRTOS.org - Documentation, books, training, latest versions,
    license and Real Time Engineers Ltd. contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, and our new
    fully thread aware and reentrant UDP/IP stack.

    http://www.OpenRTOS.com - Real Time Engineers ltd license FreeRTOS to High 
    Integrity Systems, who sell the code with commercial support,
    indemnification and middleware, under the OpenRTOS brand.
    
    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.
*/
#include <string.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* lwIP includes. */
#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include <lwip/stats.h>
#include <lwip/snmp.h>
#include "netif/etharp.h"

/* BSP includes. */
#include "ftmac100.h"

/* Define those to better describe your network interface. */
#define IFNAME0 'e'
#define IFNAME1 'l'

#define FTMAC100_RX_TASK_PRIORITY       ( configMAX_PRIORITIES - 1 )
#define FTMAC100_RX_STACK_SIZE          2048

/* When a packet is ready to be sent, if it cannot be sent immediately then
 * the task performing the transmit will block for netifTX_BUFFER_FREE_WAIT
 * milliseconds.  It will do this a maximum of netifMAX_TX_ATTEMPTS before
 * giving up.
 */
#define netifTX_BUFFER_FREE_WAIT	( ( portTickType ) 2UL / portTICK_RATE_MS )
#define netifMAX_TX_ATTEMPTS		( 5 )

#define RX_QUEUE_ENTRIES	128	/* must be power of 2 */
#define TX_QUEUE_ENTRIES	2	/* must be power of 2 */

/*
 * Ethernet maximum payload 1500 bytes, plus header total 1542 bytes.
 */
#define MAX_PKT_SIZE		1520
#define RX_BUF_SIZE		2032	/* must be smaller than 0x7ff */

#if MAX_PKT_SIZE > 0x7ff
#error invalid MAX_PKT_SIZE
#endif

#define netifMAX_MTU MAX_PKT_SIZE

/*
 * Tx and Rx descriptors
 */
struct ftmac100_rxdes gRxDes[ RX_QUEUE_ENTRIES ] __attribute__((section(".nbuf"),aligned(16)));
struct ftmac100_txdes gTxDes[ TX_QUEUE_ENTRIES ] __attribute__((section(".nbuf"),aligned(16)));

/*
 * Data Buffers
 */
static unsigned char rx_buf[ RX_BUF_SIZE * RX_QUEUE_ENTRIES ] __attribute__((section(".nbuf"),aligned(16)));
static unsigned char tx_buf[ MAX_PKT_SIZE ] __attribute__((section(".nbuf"),aligned(16)));

struct xFtmac100If {
	char hwaddr[ETHARP_HWADDR_LEN];

	unsigned int rx_pointer;
	unsigned int tx_clean_pointer;
	unsigned int tx_pointer;
	unsigned int tx_pending;

	struct netif *netIf;

	sys_sem_t tx_sem;     /* Control access to transmitter. */
	sys_sem_t rx_sem;     /* Semaphore to signal receive thread. */
};

struct xFtmac100If *pgMac100If;

/*
 * Interrupt handler.
 */
void ftMac100_InterruptHandler( void );
static void vFtMac100_ISR_Wrapper( void );

/*
 * Copy the received data into a pbuf.
 */
static struct pbuf *prvLowLevelInput( const unsigned char * const pucInputData, unsigned short usDataLength );

/*
 * Send data from a pbuf to the hardware.
 */
static err_t prvLowLevelOutput( struct netif *pxNetIf, struct pbuf *p );

/*
 * Perform any hardware and/or driver initialisation necessary.
 */
static void prvLowLevelInit( struct xFtmac100If *priv );

/*
 * Functions to rx packets.
 */
static void ftMac100_rx_task( void *arg);

/*-----------------------------------------------------------*/

static void ftMac100_alloc_buffers( void )
{
	int i;

	memset( gRxDes, 0, (sizeof(struct ftmac100_rxdes) * RX_QUEUE_ENTRIES) );

	/* initialize RX ring */
	gRxDes[RX_QUEUE_ENTRIES - 1].rxdes1 |= FTMAC100_RXDES1_EDORR;

	for (i = 0; i < RX_QUEUE_ENTRIES; i++) {
		struct ftmac100_rxdes *rxdes = &gRxDes[i];

		rxdes->rxdes2 = (int) &rx_buf[ RX_BUF_SIZE * i ];
		rxdes->rxdes1 &= FTMAC100_RXDES1_EDORR;
		rxdes->rxdes1 |= FTMAC100_RXDES1_RXBUF_SIZE(RX_BUF_SIZE);
		rxdes->rxdes0 = FTMAC100_RXDES0_RXDMA_OWN;
	}

	/* initialize TX ring */
	memset( gTxDes, 0, (sizeof(struct ftmac100_txdes) * TX_QUEUE_ENTRIES) );

	gTxDes[TX_QUEUE_ENTRIES - 1].txdes1 = FTMAC100_TXDES1_EDOTR;
}

static err_t ftMac100_reset_hw( void )
{
	/* NOTE: reset clears all registers */
	FTMAC100_OFFSET_MACCR = FTMAC100_MACCR_SW_RST;

	while( FTMAC100_OFFSET_MACCR & FTMAC100_MACCR_SW_RST ) { ; }

	return ERR_OK;
}

#define MACCR_ENABLE_ALL	(FTMAC100_MACCR_XMT_EN	| \
				 FTMAC100_MACCR_RCV_EN	| \
				 FTMAC100_MACCR_XDMA_EN	| \
				 FTMAC100_MACCR_RDMA_EN	| \
				 FTMAC100_MACCR_CRC_APD	| \
				 FTMAC100_MACCR_FULLDUP	| \
				 FTMAC100_MACCR_RX_RUNT	| \
				 FTMAC100_MACCR_RX_BROADPKT)

#define INT_MASK_ALL_ENABLED	(FTMAC100_INT_RPKT_FINISH	| \
				 FTMAC100_INT_NORXBUF		| \
				 FTMAC100_INT_RPKT_LOST		| \
				 FTMAC100_INT_AHB_ERR		| \
				 FTMAC100_INT_PHYSTS_CHG)
/**
 * In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param pxNetIf the already initialized lwip network interface structure
 *		for this etherpxNetIf
 */
static void prvLowLevelInit( struct xFtmac100If *priv )
{
	err_t xStatus;
	unsigned portBASE_TYPE uxOriginalPriority;
	unsigned int maddr, laddr;

	/* Hardware initialisation can take some time, so temporarily lower the
	task priority to ensure other functionality is not adversely effected.
	The priority will get raised again before this function exits. */
	uxOriginalPriority = uxTaskPriorityGet( NULL );
	vTaskPrioritySet( NULL, tskIDLE_PRIORITY );

	/* Reset the mac */
	xStatus = ftMac100_reset_hw();

	if( xStatus == ERR_OK )
	{
		/* setup ring buffer base registers */
		FTMAC100_OFFSET_RXR_BADR = (int) gRxDes;
		FTMAC100_OFFSET_TXR_BADR = (int) gTxDes;

		FTMAC100_OFFSET_APTC = FTMAC100_APTC_RXPOLL_CNT(1);

		/* Set mac address */
		maddr = priv->hwaddr[0] << 8 | priv->hwaddr[1];
		laddr = priv->hwaddr[2] << 24 | priv->hwaddr[3] << 16 |
			priv->hwaddr[4] << 8 | priv->hwaddr[5];
		FTMAC100_OFFSET_MAC_MADR = maddr;
		FTMAC100_OFFSET_MAC_LADR = laddr;

		FTMAC100_OFFSET_MACCR = MACCR_ENABLE_ALL;

		/* Set Rx, Tx interrupt handlers */
		VICVectAddr25 = ( long ) vFtMac100_ISR_Wrapper;

		/* Enable Rx, Tx interrupts */
		FTMAC100_OFFSET_IMR = INT_MASK_ALL_ENABLED;
		VICIRQMask |= ( 1 << 25 );
		VICIRQMode &= ~( 1 << 25 );
		VICIRQLevel &= ~( 1 << 25 );

	}

	/* Reset the task priority back to its original value. */
	vTaskPrioritySet( NULL, uxOriginalPriority );

	configASSERT( xStatus == ERR_OK );
}

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param pxNetIf the lwip network interface structure for this etherpxNetIf
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *		 an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *	   strange results. You might consider waiting for space in the DMA queue
 *	   to become availale since the stack doesn't retry to send a packet
 *	   dropped because of memory failure (except for the TCP timers).
 */

static err_t prvLowLevelOutput( struct netif *pxNetIf, struct pbuf *p )
{

	/* This is taken from lwIP example code and therefore does not conform
	to the FreeRTOS coding standard. */

	struct pbuf *q;
	unsigned char *pucBuffer = tx_buf;
	unsigned char *pucChar;
	u16_t usTotalLength = p->tot_len - ETH_PAD_SIZE;
	err_t xReturn = ERR_OK;
	long x;
	struct ftmac100_txdes *txdes;

	( void ) pxNetIf;

	#if defined(LWIP_DEBUG) && LWIP_NETIF_TX_SINGLE_PBUF
	LWIP_ASSERT("p->next == NULL && p->len == p->tot_len", p->next == NULL && p->len == p->tot_len);
	#endif

	/* Initiate transfer. */
	if( p->len == p->tot_len )
	{
		memcpy( pucBuffer, &( ( unsigned char * ) p->payload )[ ETH_PAD_SIZE ], usTotalLength );
	}
	else
	{
		/* pbuf chain, copy into contiguous tx_buf. */
		if( p->tot_len >= sizeof( tx_buf ) )
		{
#if LINK_STATS
			LINK_STATS_INC( link.lenerr );
			LINK_STATS_INC( link.drop );
#endif
			xReturn = ERR_BUF;
		}
		else
		{
			pucChar = tx_buf;

			for( q = p; q != NULL; q = q->next )
			{
				/* Send the data from the pbuf to the interface, one pbuf at a
				time. The size of the data in each pbuf is kept in the ->len
				variable. */
				/* send data from(q->payload, q->len); */
				LWIP_DEBUGF( NETIF_DEBUG, ( "NETIF: send pucChar %p q->payload %p q->len %i q->next %p\n", 
							    pucChar, q->payload, ( int ) q->len, ( void* ) q->next ) );
				if( q == p )
				{
					memcpy( pucChar, &( ( char * ) q->payload )[ ETH_PAD_SIZE ], q->len - ETH_PAD_SIZE );
					pucChar += q->len - ETH_PAD_SIZE;
				}
				else
				{
					memcpy( pucChar, q->payload, q->len );
					pucChar += q->len;
				}
			}
		}
	}

	if( xReturn == ERR_OK )
	{
		sys_sem_wait( &pgMac100If->tx_sem );

		xReturn = ERR_BUF;

#if 0 /* Only has 1 tx descriptor */
		txdes = &gTxDes[0];
#else
		txdes = &gTxDes[pgMac100If->tx_pointer];
		pgMac100If->tx_pointer = (pgMac100If->tx_pointer + 1) & (TX_QUEUE_ENTRIES - 1);;
#endif
		/* setup TX descriptor */
		txdes->txdes2 = (int) pucBuffer;

		txdes->txdes1 &= FTMAC100_TXDES1_EDOTR;

		if (usTotalLength < 64)
			usTotalLength = 64;
		txdes->txdes1 |= ( FTMAC100_TXDES1_FTS | FTMAC100_TXDES1_LTS |
				   FTMAC100_TXDES1_TXBUF_SIZE( usTotalLength ) );

		/* Descriptor owned by FTMAC */
		txdes->txdes0 = FTMAC100_TXDES0_TXDMA_OWN;

		/* start transmit */
		FTMAC100_OFFSET_TXPD = 1;
		for( x = 0; x < netifMAX_TX_ATTEMPTS; x++ )
		{
			if( !(txdes->txdes0 & FTMAC100_TXDES0_TXDMA_OWN) )
			{
				xReturn = ERR_OK;
#if LINK_STATS
				LINK_STATS_INC( link.xmit );
#endif
				break;
			}
			else
			{
				vTaskDelay( netifTX_BUFFER_FREE_WAIT );
			}
		}

		if( xReturn != ERR_OK )
		{
#if LINK_STATS
			LINK_STATS_INC( link.memerr );
			LINK_STATS_INC( link.drop );
#endif
		}

		/* reset the descriptor */
		txdes->txdes1 &= ~( FTMAC100_TXDES1_FTS | FTMAC100_TXDES1_LTS |
				    FTMAC100_TXDES1_TXBUF_SIZE( 0x7ff ) );
		sys_sem_signal( &pgMac100If->tx_sem );
	}

	return xReturn;
}

/**
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param pxNetIf the lwip network interface structure for this etherpxNetIf
 * @return a pbuf filled with the received packet (including MAC header)
 *		 NULL on memory error
 */
static struct pbuf *prvLowLevelInput( const unsigned char * const pucInputData, unsigned short usDataLength )
{
	struct pbuf *p = NULL, *q;

	if( usDataLength > 0U )
	{
		#if ETH_PAD_SIZE
			len += ETH_PAD_SIZE; /* allow room for Ethernet padding */
		#endif

		/* We allocate a pbuf chain of pbufs from the pool. */
		p = pbuf_alloc( PBUF_RAW, usDataLength, PBUF_POOL );

		if( p != NULL )
		{
			#if ETH_PAD_SIZE
				pbuf_header( p, -ETH_PAD_SIZE ); /* drop the padding word */
			#endif

			/* We iterate over the pbuf chain until we have read the entire
			* packet into the pbuf. */
			usDataLength = 0U;
			for( q = p; q != NULL; q = q->next )
			{
				/* Read enough bytes to fill this pbuf in the chain. The
				* available data in the pbuf is given by the q->len
				* variable.
				* This does not necessarily have to be a memcpy, you can also preallocate
				* pbufs for a DMA-enabled MAC and after receiving truncate it to the
				* actually received size. In this case, ensure the usTotalLength member of the
				* pbuf is the sum of the chained pbuf len members.
				*/
				memcpy( q->payload, &( pucInputData[ usDataLength ] ), q->len );
				usDataLength += q->len;
			}

			#if ETH_PAD_SIZE
				pbuf_header( p, ETH_PAD_SIZE ); /* reclaim the padding word */
			#endif

			LINK_STATS_INC(link.recv);
		}
	}

	return p;
}

static void ftMac100_rx_pointer_incr( unsigned int *pointer )
{
	*pointer += 1;

	if( *pointer == RX_QUEUE_ENTRIES )
		*pointer = 0;
}

/*-----------------------------------------------------------*/
static void ftMac100_rx_task ( void *arg )
{
	struct xFtmac100If *macIf = arg;
	struct eth_hdr *pxHeader;
	struct pbuf *p;
	struct ftmac100_rxdes *rxdes;
	struct netif *pxNetIf = macIf->netIf;
	int error = 0;

	do
	{
		sys_sem_wait( &macIf->rx_sem );

check_next:
		rxdes = &gRxDes[macIf->rx_pointer];

		if( rxdes->rxdes0 & FTMAC100_RXDES0_RXDMA_OWN )
		{
			continue;
		}

		if( !(rxdes->rxdes0 & FTMAC100_RXDES0_FRS) )
		{
			error = 1;
			LWIP_DEBUGF( NETIF_DEBUG, ( "ftmac100 rx desc not first segment\n" ) );
		}

		if( rxdes->rxdes0 & (FTMAC100_RXDES0_FTL | FTMAC100_RXDES0_RUNT |
				     FTMAC100_RXDES0_RX_ODD_NB) )
		{
			error = 1;
			LWIP_DEBUGF( NETIF_DEBUG, ( "ftmac100: rx length error\n" ) );
#if LINK_STATS
			LINK_STATS_INC( link.lenerr );
#endif
		}

		if( rxdes->rxdes0 & FTMAC100_RXDES0_CRC_ERR  )
		{
			error = 1;
			LWIP_DEBUGF( NETIF_DEBUG, ( "ftmac100: rx checksum error\n" ) );
#if LINK_STATS
			LINK_STATS_INC( link.lenerr );
#endif
		}

		if ( error )
		{
#if LINK_STATS
			LINK_STATS_INC( link.drop );
#endif
			rxdes->rxdes0 = FTMAC100_RXDES0_RXDMA_OWN;
			ftMac100_rx_pointer_incr( &macIf->rx_pointer );
			goto check_next;
		}

		/*
		 * It is impossible to get multi-segment packets
		 * because we always provide big enough receive buffers.
		 */
		if( !(rxdes->rxdes0 & FTMAC100_RXDES0_LRS) )
			LWIP_DEBUGF( NETIF_DEBUG, ( "ftmac100 rx multi-segment packets\n" ) );

		/* move received packet into a new pbuf */
		p = prvLowLevelInput( (const unsigned char * const) rxdes->rxdes2,
				      (unsigned short)(rxdes->rxdes0 & FTMAC100_RXDES0_RFL) );

		/* no packet could be read, silently ignore this */
		if( p != NULL )
		{
#if LINK_STATS
			LINK_STATS_INC( link.recv );
#endif
			/* points to packet payload, which starts with an Ethernet header */
			pxHeader = p->payload;

			switch( htons( pxHeader->type ) )
			{
			/* IP or ARP packet? */
			case ETHTYPE_IP:
			case ETHTYPE_ARP:
				/* full packet send to tcpip_thread to process */
				if( pxNetIf->input( p, pxNetIf ) != ERR_OK )
				{
					LWIP_DEBUGF(NETIF_DEBUG, ( "ethernetif_input: IP input error\n" ) );
					pbuf_free(p);
					p = NULL;
				}
				break;

			default:
				pbuf_free( p );
				p = NULL;
				break;
			}
		}
		else
		{
#if LINK_STATS
			LINK_STATS_INC( link.memerr );
#endif
		}

		/* Done. Give desc back to hw and increment index */
		rxdes->rxdes0 = FTMAC100_RXDES0_RXDMA_OWN;
		ftMac100_rx_pointer_incr( &macIf->rx_pointer );

		goto check_next;
	}
	while (1);
}
/*-----------------------------------------------------------*/

void ftMac100_InterruptHandler( void )
{
	extern portBASE_TYPE xInsideISR;
	int i, status, phycr;

	/* Ensure the pbuf handling functions don't attempt to use critical
	sections. */
	xInsideISR++;

	status = FTMAC100_OFFSET_ISR;

	if( status & (FTMAC100_INT_RPKT_FINISH | FTMAC100_INT_NORXBUF) )
	{
		sys_sem_signal( &pgMac100If->rx_sem );
	}

	if( status & (FTMAC100_INT_NORXBUF | FTMAC100_INT_RPKT_LOST |
		      FTMAC100_INT_AHB_ERR | FTMAC100_INT_PHYSTS_CHG) )
	{
		LWIP_DEBUGF( NETIF_DEBUG, ( "[ISR] = 0x%x: %s%s%s%s\n", status,
				    status & FTMAC100_INT_NORXBUF ? "NORXBUF " : "",
				    status & FTMAC100_INT_RPKT_LOST ? "RPKT_LOST " : "",
				    status & FTMAC100_INT_AHB_ERR ? "AHB_ERR " : "",
				    status & FTMAC100_INT_PHYSTS_CHG ? "PHYSTS_CHG" : "") );

		if( status & FTMAC100_INT_NORXBUF )
		{
			/* RX buffer unavailable */
#if LINK_STATS
			LINK_STATS_INC( link.memerr );
#endif
		}

		if( status & FTMAC100_INT_RPKT_LOST )
		{
			/* received packet lost due to RX FIFO full */
#if LINK_STATS
			LINK_STATS_INC( link.memerr );
#endif
		}

		if( status & FTMAC100_INT_PHYSTS_CHG )
		{
			/* PHY link status change */
			phycr = FTMAC100_PHYCR_PHYAD(0) |
				FTMAC100_PHYCR_REGAD(1) |
				FTMAC100_PHYCR_MIIRD;

			FTMAC100_OFFSET_PHYCR = phycr;

			for (i = 0; i < 10; i++) {
				phycr = FTMAC100_OFFSET_PHYCR;

				if( (phycr & FTMAC100_PHYCR_MIIRD) == 0 )
					break;
			}

			if( phycr & 0x4 )
				netif_set_link_up( pgMac100If->netIf );
			else
				netif_set_link_down( pgMac100If->netIf );
		}
	}

	xInsideISR--;
}

static void vFtMac100_ISR_Wrapper( void )
{
	/* Save the context of the interrupted task. */
	/* Done at boot.s*/

	/* Call the handler.  This must be a separate function from the wrapper
	to ensure the correct stack frame is set up. */
	__asm volatile ("bl ftMac100_InterruptHandler");

	/* Restore the context of whichever task is going to run next. */
	portRESTORE_CONTEXT();
}

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function ftmac100_Init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to pxNetIf_add().
 *
 * @param pxNetIf the lwip network interface structure for this etherpxNetIf
 * @return ERR_OK if the loopif is initialized
 *		 ERR_MEM if private data couldn't be allocated
 *		 any other err_t on error
 */
err_t ftMac100_init( struct netif *pxNetIf )
{
	err_t xReturn = ERR_OK;

	LWIP_ASSERT( "pxNetIf != NULL", ( pxNetIf != NULL ) );

	pgMac100If = mem_malloc( sizeof( struct xFtmac100If ) );
	if( pgMac100If == NULL )
	{
		LWIP_DEBUGF(NETIF_DEBUG, ( "ftmac100_init: out of memory\n" ) );
		xReturn = ERR_MEM;
	}
	else
	{
		#if LWIP_NETIF_HOSTNAME
		{
			/* Initialize interface hostname */
			pxNetIf->hostname = "lwip";
		}
		#endif /* LWIP_NETIF_HOSTNAME */

		pgMac100If->netIf = pxNetIf;

		pxNetIf->state = pgMac100If;
		pxNetIf->name[ 0 ] = IFNAME0;
		pxNetIf->name[ 1 ] = IFNAME1;

		/* set MAC hardware address length */
		pxNetIf->hwaddr_len = ETHARP_HWADDR_LEN;

		/* set MAC hardware address */
		pxNetIf->hwaddr[ 0 ] = pgMac100If->hwaddr[ 0 ] = 0xc3;
		pxNetIf->hwaddr[ 1 ] = pgMac100If->hwaddr[ 1 ] = 0x10;
		pxNetIf->hwaddr[ 2 ] = pgMac100If->hwaddr[ 2 ] = 0xda;
		pxNetIf->hwaddr[ 3 ] = pgMac100If->hwaddr[ 3 ] = 0xce;
		pxNetIf->hwaddr[ 4 ] = pgMac100If->hwaddr[ 4 ] = 0x3f;
		pxNetIf->hwaddr[ 5 ] = pgMac100If->hwaddr[ 5 ] = 0x69;

		/* device capabilities */
		pxNetIf->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

		/* maximum transfer unit */
		pxNetIf->mtu = netifMAX_MTU;

		/* We directly use etharp_output() here to save a function call.
		* You can instead declare your own function an call etharp_output()
		* from it if you have to do some checks before sending (e.g. if link
		* is available...) */
		pxNetIf->output = etharp_output;
		pxNetIf->linkoutput = prvLowLevelOutput;

		ftMac100_alloc_buffers( );

		pgMac100If->rx_pointer = 0;
		pgMac100If->tx_clean_pointer = 0;
		pgMac100If->tx_pointer = 0;
		pgMac100If->tx_pending = 0;

		/* initialize the hardware */
		prvLowLevelInit( pgMac100If );

		/* Hardware ready now, create task for receive packets */
		if( ( xReturn = sys_sem_new( &pgMac100If->tx_sem, 1 ) ) != ERR_OK )
		{
			return xReturn;
		}
		else if( ( xReturn = sys_sem_new( &pgMac100If->rx_sem, 0 ) ) != ERR_OK )
		{
			return xReturn;
		}
		else if( sys_thread_new( "mac100rx", ftMac100_rx_task, pgMac100If, FTMAC100_RX_STACK_SIZE,
					 FTMAC100_RX_TASK_PRIORITY ) == NULL )
		{
			xReturn = ERR_MEM;
		}
	}

	return xReturn;
}

