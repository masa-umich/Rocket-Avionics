# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "scapy",
#     "click",
# ]
# ///
from scapy.all import IP, UDP, send
from scapy.layers.ntp import NTPHeader
import time
from scapy.interfaces import get_working_ifaces, show_interfaces
import click
import re
from typing import override
import ipaddress

class NetworkAddress(click.ParamType):
    name: str = "address/range"
    _pattern: re.Pattern[str] = re.compile(r"^(?:\d{1,3}\.){3}\d{1,3}/\d{1,2}$")

    @override
    def convert(self, value: str, param: click.Parameter | None, ctx: click.Context | None) -> ipaddress.IPv4Network:
        if not self._pattern.match(value):
            self.fail("Address must be of the form [ip-address]/[range]", param, ctx)

        try:
            ip_network = ipaddress.IPv4Network(value)
        except (ipaddress.AddressValueError, ValueError):
            self.fail("Address must be a valid network address", param, ctx)
        
        return ip_network

@click.command(context_settings={"help_option_names": ["--help", "-h"]})
@click.option("--network", type=NetworkAddress(), default="141.212.192.0/24")
@click.option("--interval", default=2, type=click.FLOAT)
@click.option("--use-iface", is_flag=True)
@click.option("--all-iface", is_flag=True)
@click.option("--count", default=0, type=click.INT)
def main(network: ipaddress.IPv4Network, interval: float, use_iface: bool, all_iface: bool, count: int):
    if use_iface and all_iface:
        raise Exception("--use-iface and --all-iface cannot be used at the same time")
    
    if use_iface:
        show_interfaces()
        interface = input("Enter interface name: ")
        
    if use_iface:
        broadcast_strs = [f"{network.broadcast_address}%{interface}"]
    elif all_iface:
        broadcast_strs = []
        for iface in get_working_ifaces():
            if iface.ip is not None and iface.ip.strip() != "":
                broadcast_strs.append(f"{network.broadcast_address}%{iface.name}")
    else:
        broadcast_strs = [str(network.broadcast_address)]
    
    i = 0
    while count == 0 or i < count:
        for b in broadcast_strs:
            ntp_packet = NTPHeader(mode = 5)
            send(IP(dst=b)/UDP(dport=123)/ntp_packet)

        time.sleep(interval)
        i += 1

if __name__ == "__main__":
    main()