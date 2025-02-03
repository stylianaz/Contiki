/*Set up the MAC to be used*/
#undef NETSTACK_CONF_MAC
//#define NETSTACK_CONF_MAC nullmac_driver
#define NETSTACK_CONF_MAC csma_driver


/*Set up the RDC to be used*/
//#undef NETSTACK_CONF_RDC
//#define NETSTACK_CONF_RDC nullrdc_driver
