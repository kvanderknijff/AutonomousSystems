import network
import time

wlan = network.WLAN(network.STA_IF)

print("Activating WiFi...") 
wlan.active(True) 

time.sleep(1)

print("Active:", wlan.active())


mac = wlan.config("mac")

mac_str = ':'.join('{:02X}'.format(b) for b in mac)

print("MAC:", mac_str)