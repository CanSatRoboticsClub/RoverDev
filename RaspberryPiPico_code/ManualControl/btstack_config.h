/* btstack_config.h – Complete config for Pico W BLE GATT Server */

#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

// Core BLE functionality
#define ENABLE_BLE
#define ENABLE_LE_PERIPHERAL

// Required for GATT server functionality
#define ENABLE_ATT_SERVER
#define ENABLE_GATT_SERVER
#define ENABLE_L2CAP_LE_CREDIT_BASED_FLOW_CONTROL_MODE

// ATT Database configuration
#define MAX_ATT_DB_SIZE 1024
#define MAX_NR_ATT_SERVER_CLIENTS 1

// Device Database configuration
#define NVM_NUM_DEVICE_DB_ENTRIES 1

// CYW43 HCI Transport configuration
#define HCI_OUTGOING_PRE_BUFFER_SIZE 4
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT 4

// Buffer sizes
#define HCI_ACL_PAYLOAD_SIZE (255+4)
#define MAX_NR_HCI_CONNECTIONS 2
#define MAX_NR_L2CAP_CHANNELS 3
#define MAX_NR_LE_DEVICE_DB_ENTRIES 1

// ATT Server configuration
#define ATT_SERVER_MAX_PREPARE_WRITE_REQUESTS 4

// Logging
#define ENABLE_LOG_INFO 
#define ENABLE_LOG_ERROR
#define ENABLE_PRINTF_HEXDUMP

// Required for embedded systems
#define HAVE_EMBEDDED_TIME_MS

// Enable GATT compiler support (if using .gatt files)
#define ENABLE_ATT_DELAYED_RESPONSE

#endif /* __BTSTACK_CONFIG */