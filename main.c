#include <inc/hw_ints.h>
#include <stdint.h>
#include "common.h"
#include "enc28j60.h"
#include "spi.h"
#include <driverlib/systick.h>
#include <driverlib/interrupt.h>

#include <lwip/init.h>
#include <lwip/netif.h>
#include <lwip/dhcp.h>
#include <lwip/tcp_impl.h>
#include <netif/etharp.h>
#include "lwipopts.h"

#include "http.h"


static volatile unsigned long g_ulFlags;

volatile unsigned long g_ulTickCounter = 0;

#define FLAG_SYSTICK		0
#define FLAG_RXPKT		1
#define FLAG_TXPKT		2
#define FLAG_RXPKTPEND		3
#define FLAG_ENC_INT		4

#define TICK_MS			250
#define SYSTICKHZ		(1000/TICK_MS)



const uint8_t mac_addr[] = { 0x00, 0xC0, 0x033, 0x50, 0x48, 0x12 };

struct netif netif_glob;
ip_addr_t ipaddr_glob, netmask_glob, gw_glob;

unsigned long last_arp_time, last_tcp_time, last_dhcp_coarse_time, last_dhcp_fine_time;

static void spi_init(void);
static void cpu_init(void) ;
static void uart_init(void);

void enc_task();
void systick_int_enable();
void enc28j60_comm_init(void) ;

void lwip_task(void);
void init_ip(void);


int main(void) {
  cpu_init();
  uart_init();
  spi_init();
  enc28j60_comm_init();

  printf("Welcome\n");

  enc_init(mac_addr);

  systick_int_enable();

  lwip_init();
  init_ip();

  httpd_init();

  last_arp_time = last_tcp_time = 0;

  while(1) {
    MAP_SysCtlSleep();

    lwip_task();

    enc_task();

  }

  return 0;
}

void lwip_task(void)
{
	if(HWREGBITW(&g_ulFlags, FLAG_SYSTICK) == 1) {
	      HWREGBITW(&g_ulFlags, FLAG_SYSTICK) = 0;

	      if( (g_ulTickCounter - last_arp_time) * TICK_MS >= ARP_TMR_INTERVAL) {
	    	  etharp_tmr();
	    	  last_arp_time = g_ulTickCounter;
	      }

	      if( (g_ulTickCounter - last_tcp_time) * TICK_MS >= TCP_TMR_INTERVAL) {
	    	  tcp_tmr();
	    	  last_tcp_time = g_ulTickCounter;
	      }

	      if( (g_ulTickCounter - last_dhcp_coarse_time) * TICK_MS >= DHCP_COARSE_TIMER_MSECS) {
	    	  dhcp_coarse_tmr();
	    	  last_dhcp_coarse_time = g_ulTickCounter;
	      }

	      if( (g_ulTickCounter - last_dhcp_fine_time) * TICK_MS >= DHCP_FINE_TIMER_MSECS) {
	    	  dhcp_fine_tmr();
	    	  last_dhcp_fine_time = g_ulTickCounter;
	      }
	    }
}

void init_ip()
{
#if LWIP_DHCP == 0
  IP4_ADDR(&gw_glob, 10,0,0,1);
  IP4_ADDR(&ipaddr_glob, 10,0,0,100);
  IP4_ADDR(&netmask_glob, 255, 255, 255, 0)
#else
  IP4_ADDR(&gw_glob, 0,0,0,0);
  IP4_ADDR(&ipaddr_glob, 0,0,0,0);
  IP4_ADDR(&netmask_glob, 0, 0, 0, 0);
#endif

  netif_add(&netif_glob, &ipaddr_glob, &netmask_glob, &gw_glob, NULL, enc28j60_init, ethernet_input);
  netif_set_default(&netif_glob);

#if LWIP_DHCP == 1
  dhcp_start(&netif_glob);
  printf("DHCP Started\n");
#else
  netif_set_up(&netif);
  printf("Static IP Address Assigned\n");
#endif


}

void enc_task()
{
    if( HWREGBITW(&g_ulFlags, FLAG_ENC_INT) == 1 ) {
      HWREGBITW(&g_ulFlags, FLAG_ENC_INT) = 0;
      enc_action(&netif_glob);
    }
}

void systick_int_enable()
{
	//
	// Configure SysTick for a periodic interrupt.
	//
	MAP_SysTickPeriodSet(MAP_SysCtlClockGet() / SYSTICKHZ);
	MAP_SysTickEnable();
	MAP_SysTickIntEnable();

	//MAP_IntEnable(INT_GPIOA);
	MAP_IntEnable(INT_GPIOE);
	MAP_IntMasterEnable();

	MAP_SysCtlPeripheralClockGating(false);

	MAP_GPIOIntTypeSet(GPIO_PORTE_BASE, ENC_INT, GPIO_FALLING_EDGE);
	MAP_GPIOPinIntClear(GPIO_PORTE_BASE, ENC_INT);
	MAP_GPIOPinIntEnable(GPIO_PORTE_BASE, ENC_INT);
	printf("systick and interrupt enabled\n");
}

uint8_t spi_send(uint8_t c) {
  unsigned long val;
  MAP_SSIDataPut(SSI2_BASE, c);
  MAP_SSIDataGet(SSI2_BASE, &val);
  return (uint8_t)val;
}

void
SysTickIntHandler(void)
{
    //
    // Increment the system tick count.
    //
    g_ulTickCounter++;

    //
    // Indicate that a SysTick interrupt has occurred.
    //
    HWREGBITW(&g_ulFlags, FLAG_SYSTICK) = 1;
    //g_ulFlags |= FLAG_SYSTICK;
}

void GPIOPortEIntHandler(void) {
  uint8_t p = MAP_GPIOPinIntStatus(GPIO_PORTE_BASE, true) & 0xFF;

  MAP_GPIOPinIntClear(GPIO_PORTE_BASE, p);

  HWREGBITW(&g_ulFlags, FLAG_ENC_INT) = 1;
}

void enc28j60_comm_init(void) {
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, ENC_CS);
//  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, ENC_CS | ENC_RESET | SRAM_CS);
  MAP_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, ENC_INT);

//  MAP_GPIOPinWrite(GPIO_PORTA_BASE, ENC_RESET, 0);
  MAP_GPIOPinWrite(ENC_CS_PORT, ENC_CS, ENC_CS);
}

static void spi_init(void) {
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

  // Configure SSI1 for SPI RAM usage
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
  MAP_GPIOPinConfigure(GPIO_PB4_SSI2CLK);
  MAP_GPIOPinConfigure(GPIO_PB6_SSI2RX);
  MAP_GPIOPinConfigure(GPIO_PB7_SSI2TX);
  MAP_GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7);
  MAP_SSIConfigSetExpClk(SSI2_BASE, MAP_SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
			 SSI_MODE_MASTER, 1000000, 8);
  MAP_SSIEnable(SSI2_BASE);

  unsigned long b;
  while(MAP_SSIDataGetNonBlocking(SSI2_BASE, &b)) {}
}

static void cpu_init(void) {
  // A safety loop in order to interrupt the MCU before setting the clock (wrongly)
  int i;
  for(i=0; i<1000000; i++);

  // Setup for 16MHZ external crystal, use 200MHz PLL and divide by 4 = 50MHz
  MAP_SysCtlClockSet(SYSCTL_SYSDIV_16 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
      SYSCTL_XTAL_16MHZ);
}

static void uart_init(void) {
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

  // Configure PD0 and PD1 for UART
  MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
  MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
  MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  /*UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
                      UART_CONFIG_WLEN_8| UART_CONFIG_STOP_ONE| UART_CONFIG_PAR_NONE);*/
  UARTStdioInitExpClk(0, 115200);
}


#if 0
void
dhcpc_configured(const struct dhcpc_state *s)
{
    uip_sethostaddr(&s->ipaddr);
    uip_setnetmask(&s->netmask);
    uip_setdraddr(&s->default_router);
    printf("IP: %d.%d.%d.%d\n", s->ipaddr[0] & 0xff, s->ipaddr[0] >> 8,
	   s->ipaddr[1] & 0xff, s->ipaddr[1] >> 8);
}
#endif

#if 0
clock_time_t
clock_time(void)
{
    return((clock_time_t)g_ulTickCounter);
}
#endif

