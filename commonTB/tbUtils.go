//*********************************************************************************/
// NAME              REV  DATE       REMARKS			@
// Goran Scuric      1.0  10222019  Initial design
//================================================================================
package common

import (
	"encoding/binary"
	"fmt"
	"math"
	"net"
)

func isUnicast(mac []byte) bool {
	for i := 0; i < 6; i++ {
		if mac[i] != 0xff {
			return true
		}
	}
	return false
}
func isMyMac(pktMac, myMac []byte) bool {
	for i := 0; i < 6; i++ {
		if pktMac[i] != myMac[i] {
			return false
		}
	}
	return true
}
func setBroadcastMac(mac []byte) {
	for i := 0; i < 6; i++ {
		mac[i] = 0xff
	}
}

func IP2int(ip net.IP) uint32 {
	if len(ip) == 16 {
		return binary.BigEndian.Uint32(ip[12:16])
	}
	return binary.BigEndian.Uint32(ip)
}

func Int2IP(nn uint32) net.IP {
	ip := make(net.IP, 4)
	binary.BigEndian.PutUint32(ip, nn)
	return ip
}

func GetDistanceFromCoordinates(x1, y1, z1, x2, y2, z2 float64) float64 {
	// distance between point above the city and the satellite, convert to km
	distanceSquare := ((x2-x1)/1000)*((x2-x1)/1000) +
		((y2-y1)/1000)*((y2-y1)/1000) +
		((z2-z1)/1000)*((z2-z1)/1000)
	distance := math.Sqrt(distanceSquare)
	fmt.Println("Distance = ", distance)
	return distance
}

func GetPositionFromCoordinates(x, y, z, radius float64) (float64, float64) {
	latitude := math.Asin(z/radius) * 180.0 / 3.14159
	longitude := math.Atan(y/x) * 180.0 / 3.14159
	return longitude, latitude
}

func GetCoordinatesFromPosition(longitude, latitude, radius float64) (float64, float64, float64) {
	//double NY_lat  = 40.730610;  // 40 deg 43 min 50,1960 sec N
	//double NY_lon  = -73.935242; // 73 deg 56 min 6.8712 sec W
	//double LON_lat = 51.509865; // 51 deg 30 min 35.5140 sec N
	//double LON_lon = -0.118092; // 0  deg 7  min 5.1312 sec N
	x := radius * math.Cos(latitude*3.14159/180) * math.Cos(longitude*3.14159/180)
	y := radius * math.Cos(latitude*3.14159/180) * math.Sin(longitude*3.14159/180)
	z := radius * math.Sin(latitude*3.14159/180)
	return x, y, z
}

func GetDistanceFromPosition(longitude1, latitude1, radius1, longitude2, latitude2, radius2 float64) float64 {
	x1, y1, z1 := GetCoordinatesFromPosition(longitude1, latitude1, radius1)
	x2, y2, z2 := GetCoordinatesFromPosition(longitude2, latitude2, radius2)
	distance := GetDistanceFromCoordinates(x1, y1, z1, x2, y2, z2)
	return distance
}

//===========================================================================================
//
//===========================================================================================
func ConvertXYZtoLatLong(x, y, z float64, heightFromEarthCenter float64) (float64, float64) {
	// calculate back the lat and long
	// latitude  = asin (z / R) * 180 /3.14159
	// longitude = atan2(y / x) * 180 /3.14159
	// all values in either just kilo meters, OR just meters, no mixing
	latitude := math.Asin(z/heightFromEarthCenter) * 180.0 / 3.14159
	longitude := math.Atan(y/x) * 180.0 / 3.14159
	return latitude, longitude
}

//===========================================================================================
//
//===========================================================================================
func ConvertLatLongToXYZ(latitude, longitude float64, heightFromEarthCenter float64) (float64, float64, float64) {
	// all values in kilo meters, earth Radius = 6371 km
	x := heightFromEarthCenter * math.Cos((latitude*3.1415926535)/180.0) * math.Cos((longitude*3.1415926535)/180.0)
	y := heightFromEarthCenter * math.Cos((latitude*3.1415926535)/180.0) * math.Sin((longitude*3.1415926535)/180.0)
	z := heightFromEarthCenter * math.Sin((latitude*3.1415926535)/180.0)
	return x, y, z
}

//===========================================================================================
//
//===========================================================================================
func DistanceTwoPoints(x1, y1, z1, x2, y2, z2 float64) float64 {
	distanceFloat64 := math.Sqrt(
		math.Pow(x1-x2, 2) +
			math.Pow(y1-y2, 2) +
			math.Pow(z1-z2, 2))
	return distanceFloat64
}

//===========================================================================================
//
//===========================================================================================
func DistanceToEarthCenter(x, y, z float64) float64 {
	// Note Drone.GroundX, Y and Z are from GROUNDINFO msg previously
	// received from the Ground Station
	// This returns distance from SAT to a point on earth surface
	distanceFloat64 := math.Sqrt(
		math.Pow(x, 2) +
			math.Pow(y, 2) +
			math.Pow(z, 2))
	return distanceFloat64
}

//============================================================================
// Locate specific module in the slice of all known (containing rows for
// all known masters including ourselves and the office master). The slice is
// populated from the KeepAlive msg received from officeMaster
// Return nil if row not found
//============================================================================
func LocateDroneRecord(slice []TerminalInfo, DroneName string) (*TerminalInfo, int) {
	for index := range slice {
		if slice[index].TerminalName == DroneName {
			return &slice[index], index
		}
	}
	// requested module not found in the slice of known modules
	return nil, -1
}

//====================================================================================
//
//====================================================================================
func LocateGroundInfo(drone M3Info, myName string, sliceOfDrones []TerminalInfo) (*net.UDPAddr,
	NameId, bool) {
	// NOTE that this will fail if GroundUdpAddress has not been initialized due
	// to Ground not being up . Add state and try again to Resolve before
	// doing this
	var err error
	var UdpAddress = new(net.UDPAddr)
	var FullName NameId

	// fmt.Println(myName, ": Locate Office Manager")
	UdpAddress, err = net.ResolveUDPAddr("udp", drone.GroundIP)
	if err != nil {
		fmt.Println(myName, ": ERROR locating Office Manager, will retry", err)
		return nil, FullName, false
	}

	FullName = NameId{Name: drone.GroundIPandPort, Address: *UdpAddress}

	// Now that we know that GROUND is alive add entry to our slice of known
	//officeMaster := NodeInfo{NodeName: drone.GroundIPandPort, LastChangeTime: TBtimestampNano(), // time.Now().String(),
	//	MsgsSent: 0, MsgLastSentAt: time.Now(), MsgsRcvd: 0, MsgLastRcvdLocal: time.Now()}

	//sliceOfDrones = append(sliceOfDrones, officeMaster)

	theGround, _ := LocateDroneRecord(sliceOfDrones, drone.GroundIPandPort)
	if theGround != nil {
		fmt.Println(myName, ": MGR=", theGround.TerminalName, "ADDRESS:", theGround.TerminalIP,
			"Port:", theGround.TerminalPort, "MSGSRCVD:", theGround.TerminalMsgsRcvd)
	}
	return UdpAddress, FullName, true
}

//====================================================================================
//
//====================================================================================
func formatReceiver(name string, osId int, udpAddress net.UDPAddr) NameId {
	receiver := NameId{Name: name, Address: udpAddress} // TimeCreated: "",
	return receiver
}

/*
//====================================================================================
// Save the pointer to my own row for faster handling
// Send registration msg to officeMaster containing our full row from our slice of
// masters.
//====================================================================================
func SendRegisterMsg(senderFullName NameId,
	sliceOfMasters []NodeInfo, myConnection *net.UDPConn) {
	// Locate our own record in the slice of masters
	me, _ := LocateDroneRecord(sliceOfMasters, senderFullName.Name)

	if me != nil {
		officeUdpAddress, rcvrFullName, result :=
			LocateGroundInfo(senderFullName.Name, sliceOfMasters)
		if result == false {
			fmt.Println(senderFullName.Name, ": Failed to Send Register Msg")
			return
		}
		// Marshall our own row from the slice
		msgBody, _ := TBmarshal(me)
		me.MsgLastSentAt = time.Now().String() // strconv.FormatInt(TBtimestamp(),10)
		newMsg := TBregisterMsg(senderFullName, rcvrFullName, string(msgBody))
		// fmt.Println(myName, "stateConnected REGISTER with officeMaster ")
		TBsendMsgOut(newMsg, *officeUdpAddress, myConnection)
	} else {
		fmt.Println(senderFullName.Name, ": FAILED to locate mngr record in the slice")
	}
}
*/
//var p1Name = "NewYork"

/*
		var p1Lat =  40.730610
		var p1Long = -73.935242
	var p2Name "London"
	var p2Lat = 51.509865
	var p2Long = -0.118092
	var p3Name = "LosAngeles"
	var p3Lat  = 34.052235
	var p3Long = -118.243683
	var a = 100.0
	var b = 100.0
	var west = -1
	var north = 1
		deltaLat  := math.Atan(field_b/(2.0 * refElev)) * (180.0/3.14159265358979323846)
		deltaLong := math.Atan(field_a/(2.0 * refElev))* (180.0/3.14159265358979323846)
		fmt.Printf("refPoint deltaLat=%v  deltaLong=%v \n", deltaLat, deltaLong)
		fmt.Println("-------------------------------------------------------------------------------")
		if p3Lat < 0 { north = -1}
		if p3Long > 0 { west = 1}
		fmt.Println("WEST=", west, "  NORTH=", north)
		// TESTx,y,z
		fmt.Println("Point of reference = ", p3Name)
		x,y,z := ConvertLatLongToXYZ(p3Lat, p3Long, groundHeightFromEartCenter)
		lat, lon := ConvertXYZtoLatLong(x,y,z,groundHeightFromEartCenter)
		if west == -1 {lon = (180 - lon) * float64(west)}
		if north == -1 {lat = (180 - lat) * float64(north) }
		fmt.Println("Height=", groundHeightFromEartCenter," p3Lat=", p3Lat, " p3Long=", p3Long,
			" x=", x, " y=", y, " z=", z)
		fmt.Println("Reverse: Height=", groundHeightFromEartCenter, " p3Lat=", lat, " p3Long=", lon)
		d := DistanceToEarthCenter(x,y,z)
		fmt.Println("Distance to Earth d=", d)
		fmt.Println("-------------------------------------------------------------------------------")
		x1,y1,z1 := ConvertLatLongToXYZ(p3Lat, p3Long, groundHeightFromEartCenter + 1000)
		lat1, lon1 := ConvertXYZtoLatLong(x1,y1,z1,groundHeightFromEartCenter+1000)
		if west == -1 {lon1 = (180 - lon1) * float64(west)}
		if north == -1 {lat1 = (180 - lat1) * float64(north) }
		fmt.Println("Height=", groundHeightFromEartCenter + 1000," p3Lat1=", p3Lat, " p3Long1=", p3Long,
			" x=", x1, " y=", y1, " z=", z1)
		fmt.Println("Reverse: Height=", groundHeightFromEartCenter+1000, " p3Lat=", lat1, " p3Long=", lon1)
		d1 := DistanceToEarthCenter(x1,y1,z1)
		fmt.Println("Distance to Earth d1=", d1)
		fmt.Println("-------------------------------------------------------------------------------")
		dPhi := math.Asin(a/(2 * (groundHeightFromEartCenter)))
		dLambda := math.Asin(b/(2 * (groundHeightFromEartCenter)))
		dPhi1 := math.Asin(a/(2 * (groundHeightFromEartCenter + 1000)))
		dLambda1 := math.Asin(b/(2 * (groundHeightFromEartCenter + 1000)))
		fmt.Println("deltaPhi  = ", dPhi)
		fmt.Println("deltaPhi1 = ", dPhi1)
		fmt.Println("deltaLambda  = ", dLambda)
		fmt.Println("deltaLambda1 = ", dLambda1)
		fmt.Println("-------------------------------------------------------------------------------")

		x5, y5, z5 := myGeo.ToECEF(latLA, longLA, refElev) // returns meters
		fmt.Printf("LosAngeles+1000: refX=%v refY=%v refZ=%v\n", int(x5), int(y5), int(z5))

		refPointLat, refPointLong, refPointAlt := myGeo.ToLLA(x5, y5, z5)
		fmt.Printf("LosAngeles+1000: lat=%v lon=%v alt=%v \n", refPointLat, refPointLong, int(refPointAlt))
	//===============================================================================
	// calculate four corners
	// First method of locating the 4 corners did not work well
	// erros were to big, look at the second method bellow which only has about 0.1%
	// error
	//===============================================================================
	x4, y4, z4 := myGeo.ToECEF(latLA, longLA, groundHeightFromEartCenter)
	fmt.Printf("LosAngeles     : x4=%v y4=%v z4=%v\n", int(x4), int(y4), int(z4))
	lat4, lon4, alt4 := myGeo.ToLLA(x4, y4, z4)
	fmt.Printf("LosAngeles Reverse: lat4=%v lon4=%v alt4=%v \n", lat4, lon4, int(alt4))
	var latE,  latG float64
	var longF,longH float64
	if refLat > 0 {
		latE = refLat + deltaLat
		latG = refLat - deltaLat
	} else {
		latE = refLat - deltaLat
		latG = refLat + deltaLat
	}
	if refLong > 0 {
		longF = refLong + deltaLong
		longH = refLong - deltaLong
	} else {
		longF = refLong + deltaLong
		longH = refLong - deltaLong
	}
	latA  := latE
	latB  := latE
	latC  := latG
	latD  := latG
	longA := longH
	longB := longF
	longC := longF
	longD := longH
	fmt.Printf("LosAngeles+1k: lat=%v lon=%v alt=%v \n", refLat, refLong, int(refPointAlt))
	fmt.Println("-------------------------------------------------------------------------------")
	fmt.Printf("LosAngeles+AA: lat=%v lon=%v alt=%v \n", latA, longA, int(refPointAlt))
	fmt.Printf("LosAngeles+BB: lat=%v lon=%v alt=%v \n", latB, longB, int(refPointAlt))
	fmt.Printf("LosAngeles+CC: lat=%v lon=%v alt=%v \n", latC, longC, int(refPointAlt))
	fmt.Printf("LosAngeles+DD: lat=%v lon=%v alt=%v \n", latD, longD, int(refPointAlt))
	fmt.Println("-------------------------------------------------------------------------------")
	diag := math.Sqrt(math.Pow(field_a, 2) + math.Pow(field_b, 2) )
	fmt.Printf("Diagonal length = %v meters\n", int(diag))
	distanceAB, bearingAB := myGeo.To(latA, longA, latB, longB)
	x1,y1,z1 := myGeo.ToECEF(latA, longA, refElev)
	dACenter :=DistanceTwoPoints(x1,y1,z1,x5,y5,z5)
	fmt.Printf("Distance A-Center = %v meters\n", int(dACenter))
	x2,y2,z2 := myGeo.ToECEF(latB, longB, refElev)
	dBCenter :=DistanceTwoPoints(x2,y2,z2,x5,y5,z5)
	fmt.Printf("Distance B-Center = %v meters\n", int(dBCenter))
	dAB :=DistanceTwoPoints(x1,y1,z1,x2,y2,z2)
	fmt.Printf("Point A     : x1=%v y1=%v z1=%v  meters\n", int(x1), int(y1), int(z1))
	fmt.Printf("Point B     : x2=%v y2=%v z2=%v  meters\n", int(x2), int(y2), int(z2))
	fmt.Printf("DistanceAB = %v meters\n", int(dAB))
	fmt.Printf("DistanceAB = %v BearingAB = %v\n", int(distanceAB), bearingAB)

	distanceCD, bearingCD := myGeo.To(latC, longC, latD, longD)
	fmt.Printf("DistanceCD = %v BearingCD = %v\n", int(distanceCD), bearingCD)
	distanceAD, bearingAD := myGeo.To(latA, longA, latD, longD)
	fmt.Printf("DistanceAD = %v BearingAD = %v\n", int(distanceAD), bearingAD)
	distanceBC, bearingBC := myGeo.To(latB, longB, latC, longC)
	fmt.Printf("DistanceBC = %v BearingBC = %v\n", int(distanceBC), bearingBC)

*/
