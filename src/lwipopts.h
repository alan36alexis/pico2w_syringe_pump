#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

#define NO_SYS                      0
#define LWIP_SOCKET                 0
#define LWIP_COMPAT_SOCKETS         0
#define LWIP_POSIX_SOCKETS_IO_NAMES 0
#define LWIP_IGMP                   1
#define LWIP_MDNS_RESPONDER         1
#define LWIP_NUM_NETIF_CLIENT_DATA  (LWIP_MDNS_RESPONDER)
#define LWIP_HAVE_LOOPIF            1
#define LWIP_NETIF_EXT_STATUS_CALLBACK 1

#define LWIP_TCP_KEEPALIVE          1
#define LWIP_TCP_TIMEWAIT_DELAY     3000

#define MEM_LIBC_MALLOC             0
#define MEM_ALIGNMENT               4
#define MEM_SIZE                    4000
#define MEMP_NUM_TCP_SEG            32
#define MEMP_NUM_ARP_QUEUE          10
#define MEMP_NUM_SYS_TIMEOUT        15
#define PBUF_POOL_SIZE              24
#define LWIP_ARP                    1
#define LWIP_ETHERNET               1
#define LWIP_ICMP                   1
#define LWIP_RAW                    1
#define TCP_WND                     (8 * TCP_MSS)
#define TCP_MSS                     1460
#define TCP_SND_BUF                 (8 * TCP_MSS)
#define TCP_SND_QUEUELEN            ((4 * (TCP_SND_BUF) + (TCP_MSS - 1)) / (TCP_MSS))
#define LWIP_NETIF_TX_SINGLE_PBUF   1
#define LWIP_DHCP                   1
#define LWIP_IPV4                   1

#define LWIP_MQTT                   1

// OS specific settings
#define TCPIP_THREAD_PRIO           3
#define TCPIP_MBOX_SIZE             16
#define DEFAULT_RAW_RECVMBOX_SIZE   16
#define DEFAULT_UDP_RECVMBOX_SIZE   16
#define DEFAULT_TCP_RECVMBOX_SIZE   16
#define DEFAULT_ACCEPTMBOX_SIZE     16
#define TCPIP_THREAD_STACKSIZE      2048
#define DEFAULT_THREAD_STACKSIZE    2048

#endif /* _LWIPOPTS_H */
