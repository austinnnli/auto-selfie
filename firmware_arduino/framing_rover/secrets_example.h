/* secrets_example.h - copy to secrets.h and fill in.
 *
 *     cp secrets_example.h secrets.h
 *
 * secrets.h is gitignored.  This file is not, so do not put real
 * credentials here.
 *
 * The rover joins this network as an ordinary station and gets a DHCP
 * address, which it prints on the serial monitor.  The laptop must be on the
 * same network.
 */

#ifndef ROVER_SECRETS_H
#define ROVER_SECRETS_H

#define ROVER_WIFI_SSID "your-network"
#define ROVER_WIFI_PASS "your-password"

#endif /* ROVER_SECRETS_H */
