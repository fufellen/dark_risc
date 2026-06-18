#ifndef LWIPDEMO_LWIPOPTS_H
#define LWIPDEMO_LWIPOPTS_H

#define NO_SYS                          1
#define SYS_LIGHTWEIGHT_PROT            0
#define LWIP_TIMERS                     1

#define LWIP_SOCKET                     0
#define LWIP_NETCONN                    0
#define LWIP_NETIF_API                  0

#define LWIP_IPV4                       1
#define LWIP_IPV6                       0
#define LWIP_ARP                        1
#define LWIP_ETHERNET                   1
#define LWIP_UDP                        1
#define LWIP_TCP                        1
#define LWIP_RAW                        0
#define LWIP_ICMP                       0
#define LWIP_DNS                        0
#define LWIP_DHCP                       0
#define LWIP_AUTOIP                     0
#define LWIP_IGMP                       0
#define LWIP_SNMP                       0
#define IP_SOF_BROADCAST                1
#define IP_SOF_BROADCAST_RECV           1

#define LWIP_SINGLE_NETIF               1
#define LWIP_NETIF_HOSTNAME             0
#define LWIP_NETIF_STATUS_CALLBACK      0
#define LWIP_NETIF_LINK_CALLBACK        0
#define LWIP_NETIF_EXT_STATUS_CALLBACK  0
#define LWIP_NETIF_HWADDRHINT           0
#define LWIP_NETIF_TX_SINGLE_PBUF       0

#define MEM_ALIGNMENT                   4
#define MEM_SIZE                        8192
#define MEMP_MEM_MALLOC                 0
#define MEM_LIBC_MALLOC                 0
#define MEM_USE_POOLS                   0

#define MEMP_NUM_PBUF                   8
#define MEMP_NUM_UDP_PCB                3
#define MEMP_NUM_TCP_PCB                3
#define MEMP_NUM_TCP_PCB_LISTEN         2
#define MEMP_NUM_TCP_SEG                8
#define MEMP_NUM_SYS_TIMEOUT            8
#define MEMP_NUM_NETBUF                 0
#define MEMP_NUM_NETCONN                0
#define MEMP_NUM_TCPIP_MSG_API          0
#define MEMP_NUM_TCPIP_MSG_INPKT        0
#define PBUF_POOL_SIZE                  8
#define PBUF_POOL_BUFSIZE               256

#define TCP_MSS                         256
#define TCP_WND                         (4 * TCP_MSS)
#define TCP_SND_BUF                     (4 * TCP_MSS)
#define TCP_SND_QUEUELEN                8
#define TCP_LISTEN_BACKLOG              0
#define TCP_QUEUE_OOSEQ                 0

#define ARP_TABLE_SIZE                  4
#define ETHARP_SUPPORT_STATIC_ENTRIES   1
#define ETHARP_QUEUEING                 0
#define ETH_PAD_SIZE                    0

#define IP_REASSEMBLY                   0
#define IP_FRAG                         0
#define LWIP_RANDOMIZE_INITIAL_LOCAL_PORTS 0

#define CHECKSUM_CHECK_IP               0
#define CHECKSUM_CHECK_UDP              0
#define CHECKSUM_CHECK_TCP              0
#define CHECKSUM_CHECK_ICMP             0
#define CHECKSUM_GEN_IP                 1
#define CHECKSUM_GEN_UDP                1
#define CHECKSUM_GEN_TCP                1

#define LWIP_STATS                      0
#define LWIP_DEBUG                      0

#endif
