//*********************************************************************************/
// NAME              REV  DATE       REMARKS			@
// Goran Scuric      1.0  01012018  Initial design     goran@usa.net
//================================================================================
/* Various net utilities */

package common

import (
	"fmt"
	"net"
	"strconv"
	"strings"
)

func MacAddressToByte(macAddress string) []byte {
	var newMac = strings.ReplaceAll(macAddress, ":", "")
	var byteMac = []byte(newMac)
	//var mac     [6] byte
	// we now have 12 chars to convert to 12 hex
	// convert hex string to byte values
	//for c := range byteMac {
	//	mac[0] =
	//}
	return byteMac
}

//================================================
// GetLocalIp() - return non-loopback IP address and MAC
//================================================
func GetLocalIp() (string, string) {
	var HardwareName string
	netInterfaceAddresses, err := net.InterfaceAddrs()
	for _, ip1 := range netInterfaceAddresses {
		fmt.Println("IP=", ip1)
	}
	if err == nil { // no error
		for _, netInterfaceAddress := range netInterfaceAddresses {
			networkIp, ok := netInterfaceAddress.(*net.IPNet)
			//fmt.Println("GetIP: ", networkIp.IP.String)
			if strings.Contains(networkIp.IP.String(), "169.") == false {

				if ok && !networkIp.IP.IsLoopback() && networkIp.IP.To4() != nil {
					ip := networkIp.IP.String()
					interfaces, _ := net.Interfaces()
					for _, interf := range interfaces {
						if addrs, err := interf.Addrs(); err == nil {
							for index, addr := range addrs {
								fmt.Println("[", index, "]", interf.Name, ">", addr, " IP=", ip)
								// only interested in the name with current IP address
								if strings.Contains(addr.String(), ip) {
									// fmt.Println("Use name : ", interf.Name)
									HardwareName = interf.Name
								}
							}
						}
					}
					netInterface, err := net.InterfaceByName(HardwareName)
					if err != nil {
						fmt.Println(err)
					}
					// name := netInterface.Name
					macAddress := netInterface.HardwareAddr

					fmt.Println("NetUtil: ", "Host IP: "+ip+" HwName="+HardwareName+" MAC="+macAddress.String())
					hwAddr, err := net.ParseMAC(macAddress.String())
					fmt.Println("IP="+ip, " MAC="+hwAddr.String())
					return ip, hwAddr.String()
				}
			}
		}
	}

	return "", ""
}

//====================================================================================
//
//====================================================================================
func GetMastersIP(master string) string {
	a := []string{""}
	for i := 1; i < 10; i++ {
		currIp := "172.18.0." + strconv.Itoa(i)
		// fmt.Println("i=", i, "  LOOKUP ", currIp)

		a, _ = net.LookupAddr(currIp) //names []string, err error)
		if a != nil && len(a) > 0 {
			// fmt.Println("a[0]=", a[0])
			if strings.Contains(a[0], master) {
				// FOUND
				return currIp
				break
			}
		}
	}
	return ""
}

//====================================================================================
//
//====================================================================================
func myfindUdpAddress(service string) {
	udpAddr, _ := net.ResolveUDPAddr("udp", service)

	fmt.Println("AeroMesh found at ", udpAddr)
}
