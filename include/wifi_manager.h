
#ifndef WIFI_MANAGER_H_INCLUDED
#define WIFI_MANAGER_H_INCLUDED


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Defines the maximum size of a SSID name. 32 is IEEE standard.
 * @warning limit is also hard coded in wifi_config_t. Never extend this value.
 */
#define MAX_SSID_SIZE		32

/**
 * @brief Defines the maximum size of a WPA2 passkey. 64 is IEEE standard.
 * @warning limit is also hard coded in wifi_config_t. Never extend this value.
 */
#define MAX_PASSWORD_SIZE	64


/**
 * @brief Defines the maximum number of access points that can be scanned.
 *
 * To save memory and avoid nasty out of memory errors,
 * we can limit the number of APs detected in a wifi scan.
 */
#define MAX_AP_NUM 			15


/** @brief Defines the auth mode as an access point
 *  Value must be of type wifi_auth_mode_t
 *  @see esp_wifi_types.h
 *  @warning if set to WIFI_AUTH_OPEN, passowrd me be empty. See DEFAULT_AP_PASSWORD.
 */
#define AP_AUTHMODE 		WIFI_AUTH_WPA2_PSK

/** @brief Defines visibility of the access point. 0: visible AP. 1: hidden */
#define DEFAULT_AP_SSID_HIDDEN 		0

/** @brief Defines access point's name. Default value: esp32. Run 'make menuconfig' to setup your own value or replace here by a string */
#define DEFAULT_AP_SSID 			CONFIG_DEFAULT_AP_SSID

/** @brief Defines access point's password.
 *	@warning In the case of an open access point, the password must be a null string "" or "\0" if you want to be verbose but waste one byte.
 *	In addition, the AP_AUTHMODE must be WIFI_AUTH_OPEN
 */
#define DEFAULT_AP_PASSWORD 		CONFIG_DEFAULT_AP_PASSWORD


/** @brief Defines access point's bandwidth.
 *  Value: WIFI_BW_HT20 for 20 MHz  or  WIFI_BW_HT40 for 40 MHz
 *  20 MHz minimize channel interference but is not suitable for
 *  applications with high data speeds
 */
#define DEFAULT_AP_BANDWIDTH 			WIFI_BW_HT20

/** @brief Defines access point's channel.
 *  Channel selection is only effective when not connected to another AP.
 *  Good practice for minimal channel interference to use
 *  For 20 MHz: 1, 6 or 11 in USA and 1, 5, 9 or 13 in most parts of the world
 *  For 40 MHz: 3 in USA and 3 or 11 in most parts of the world
 */
#define DEFAULT_AP_CHANNEL 			CONFIG_DEFAULT_AP_CHANNEL



/** @brief Defines the access point's default IP address. Default: "192.168.1.1" */
#define DEFAULT_AP_IP				CONFIG_DEFAULT_AP_IP
/* Halley's comment */
/** @brief Defines the access point's gateway. This should be the same as your IP. Default: "192.168.1.1" */
#define DEFAULT_AP_GATEWAY			CONFIG_DEFAULT_AP_GATEWAY

/** @brief Defines the access point's netmask. Default: "255.255.255.0" */
#define DEFAULT_AP_NETMASK			CONFIG_DEFAULT_AP_NETMASK

/** @brief Defines access point's maximum number of clients. Default: 4 */
#define DEFAULT_AP_MAX_CONNECTIONS 	CONFIG_DEFAULT_AP_MAX_CONNECTIONS

/** @brief Defines access point's beacon interval. 100ms is the recommended default. */
#define DEFAULT_AP_BEACON_INTERVAL 	CONFIG_DEFAULT_AP_BEACON_INTERVAL

/** @brief Defines if esp32 shall run both AP + STA when connected to another AP.
 *  Value: 0 will have the own AP always on (APSTA mode)
 *  Value: 1 will turn off own AP when connected to another AP (STA only mode when connected)
 *  Turning off own AP when connected to another AP minimize channel interference and increase throughput
 */
#define DEFAULT_STA_ONLY 			1

/** @brief Defines if wifi power save shall be enabled.
 *  Value: WIFI_PS_NONE for full power (wifi modem always on)
 *  Value: WIFI_PS_MODEM for power save (wifi modem sleep periodically)
 *  Note: Power save is only effective when in STA only mode
 */
#define DEFAULT_STA_POWER_SAVE 			WIFI_PS_NONE

/**
 * @brief Defines the maximum length in bytes of a JSON representation of an access point.
 *
 *  maximum ap string length with full 32 char ssid: 75 + \\n + \0 = 77\n
 *  example: {"ssid":"abcdefghijklmnopqrstuvwxyz012345","chan":12,"rssi":-100,"auth":4},\n
 *  BUT: we need to escape JSON. Imagine a ssid full of \" ? so it's 32 more bytes hence 77 + 32 = 99.\n
 *  this is an edge case but I don't think we should crash in a catastrophic manner just because
 *  someone decided to have a funny wifi name.
 */
#define JSON_ONE_APP_SIZE 99

/**
 * @brief Defines the maximum length in bytes of a JSON representation of the IP information
 * assuming all ips are 4*3 digits, and all characters in the ssid require to be escaped.
 * example: {"ssid":"abcdefghijklmnopqrstuvwxyz012345","ip":"192.168.1.119","netmask":"255.255.255.0","gw":"192.168.1.1","urc":0}
 */
#define JSON_IP_INFO_SIZE 770

#define JSON_RESPONSE_INFO_SIZE 7

/*
typedef enum update_reason_code_t {
	UPDATE_CONNECTION_OK = 0,
	UPDATE_FAILED_ATTEMPT = 1,
	UPDATE_USER_DISCONNECT = 2,
	UPDATE_LOST_CONNECTION = 3
}update_reason_code_t;
*/
/**
 * The actual WiFi settings in use
 */
/*
struct wifi_settings_t{
	uint8_t ap_ssid[MAX_SSID_SIZE];
	uint8_t ap_pwd[MAX_PASSWORD_SIZE];
	uint8_t ap_channel;
	uint8_t ap_ssid_hidden;
	wifi_bandwidth_t ap_bandwidth;
	bool sta_only;
	wifi_ps_type_t sta_power_save;
	bool sta_static_ip;
	tcpip_adapter_ip_info_t sta_static_ip_config;
};
extern struct wifi_settings_t wifi_settings;
*/
char* wifi_manager_get_ip_info_json();





/**
 * @brief Generates the connection status json: ssid and IP addresses.
 * @note This is not thread-safe and should be called only if wifi_manager_lock_json_buffer call is successful.
 */
void wifi_manager_generate_ip_info_json(int );
/**
 * @brief Clears the connection status json.
 * @note This is not thread-safe and should be called only if wifi_manager_lock_json_buffer call is successful.
 */
void wifi_manager_clear_ip_info_json();

#ifdef __cplusplus
}
#endif

#endif /* WIFI_MANAGER_H_INCLUDED */
