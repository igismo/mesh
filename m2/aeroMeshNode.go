//*********************************************************************************/
// Copyright 2017 www.igismo.com.  All rights reserved. See open source license
// HISTORY:
// NAME              REV  DATE       REMARKS			@
// Goran Scuric      1.0  01012018  Initial design     goran@usa.net  by igismo
// Goran Scuric		 2.0  09012019  Adapted for bifrost project
// Goran Scuric      3.0  12012020  Adapted for AeroMesh Space Mesh Networking
//=================================================================================
// COMMAND LINE OPTIONS:
// FORMAT: sudo ./drone  [DroneName DroneId [myIP [myPort [groundIP [groundPort]]]]]
// use 0 anywhere for default
// EXAMPLE1: sudo ./aeroMeshNode DroneX 01 192.168.22.2 1201  192.168.1.1 1200
//=================================================================================
//================================================================================
package main

import (
	"fmt"
	"github.com/StefanSchroeder/Golang-Ellipsoid/ellipsoid"
	"github.com/spf13/viper"
	"net"
	"os"
	"os/exec"
	"reflect"
	"runtime"
	"strconv"
	"strings"
	"time"
	//common "../AeroMeshNode" // Windows Version
	//common "AeroMeshNode/common"   // Linux version
	// ALTERNATE
	//common "./common"              // Windows Versionls
	//"compress/gzip"
)

//"github.com/spf13/viper"
// c "../AeroMeshNode/config.yml"

const DEBUG_ROLE = 0
const DEBUG_DISCOVERY = 0

var Drone DroneInfo     // all info about me
var Me *NodeInfo = nil  // pointer to my own NodeInfo in NodeList[]
var Log = LogInstance{} // for log file storage

// DRONE CONNECTIVITY STATES  ... for now
const StateAlone = "ALONE"
const StateConnecting = "CONNECTING"
const StateConnected = "CONNECTED"

// Command from local terminal, or rcvd from ground controller
const LOCAL_CMD = "LOCAL"
const REMOTE_CMD = "REMOTE"
const STEPMODE_DISABLED = -1

//====================================================================================
// MAIN: Need to set GROUND address correctly later, at the moment
// Drone IP = local address, and then groundIP = drone ip
// Note that these are relevant structures:
// - structure DroneInfo - one per this drone, top level structure
// 		- structure array NodeInfo[64] - one struct for each drone in the group
// - KnownDrones []NodeInfo - array list of known/learned drones
//		- could be integrated with the one above
// - ME points to our own NodeInfo[my drone id]
//====================================================================================

//===============================================================================
//
//===============================================================================
func main() {
	myOS := runtime.GOOS
	fmt.Println("========= DRONE START on ", myOS, " at ", time.Now(), "==========")

	//loc,err := Geocode("11636,Montana Ave,Los Angeles,California,United States")

	// ReverseGeocode(lat, lng float64) (*Address, error)

	GpsUtcToPst()
	//lat,long,err7 :=GpsLocationToLatLong("11636,Montana Ave,Los Angeles,California,United States")
	//fmt.Println("Latitude:", lat, " Longitude:", long,"  err=", err7)
	//addr, err8 := GpsLatLongToLocation(40.775807,-73.97632 )
	//fmt.Println("Address=", addr, "  Err=", err8)
	// TEST 1
	// GpsDeviceThread()
	//TEST 2
	//if TestPositions() { return }

	// Check my win10 workstation IP ... just for fun
	ad, _ := net.LookupIP("AFA381.aero.org")
	fmt.Println("===== AF381 IP=", ad)
	//===============================================================================
	// UPDATE RELEVANT VARIABLES and structures
	//===============================================================================
	InitDroneConfiguration()
	// First try to read config file
	InitFromConfigFile()
	// Then overwrite if any command arguments given
	InitFromCommandLine()

	// TODO: set drone arrays, or in the future the list of drones
	for i := 0; i < MAX_NODES; i++ {
		nodex := &Drone.NodeList[i]
		nodex.NodeActive = false
		// TODO maybe not needed ? used for drawing edges
		BMclear(&nodex.NewConnectivity)
		BMclear(&nodex.PreviousConnectivity)
		var bit uint64 = 0x1
		for j := 0; j < 64; j++ {
			// TODO .. from prev coded, should be just = 0xffffffffffffffff
			nodex.BaseStationList.M_BitMask[j] = bit
			nodex.NewConnectivity.M_BitMask[j] = bit
			nodex.PreviousConnectivity.M_BitMask[j] = bit
			nodex.SubscriberList.M_BitMask[j] = bit
			bit <<= 1
		}
	}

	SetFinalDroneInfo()
	// Create LOG file
	CreateLog(&Log, Drone.DroneName, Drone.DroneLogPath)
	Log.Warning(&Log, "Warning test:this will be printed anyway")

	InitDroneConnectivity()

	// Make this work one of these days ...
	var err error
	checkErrorNode(err)

	ControlPlaneInit()

	// TODO: Template for future, for going from fixed 64 drones to any number
	// Add my self to known drone array
	//myEntry := NodeInfo{ NodeName: Drone.DroneFullName.Name, TimeCreated: Drone.DroneCreationTime,
	//							MsgsSent: 0, MsgLastSentAt: "0", MsgsRcvd: 0, MsgLastRcvdAt: "0"}
	//Drone.KnownDrones = append(Drone.KnownDrones, myEntry)

	// START SEND AND RECEIVE THREADS:
	err2 := ControlPlaneRecvThread()
	if err2 != nil {
		fmt.Println(Drone.DroneName, "INIT: Error creating Broadcast/Unicast RX thread")
		panic(err2)
	}

	err3 := GpsDeviceStartThread()
	if err3 != nil {
		fmt.Println(Drone.DroneName, "INIT: Error creating GPS device thread")
		panic(err3)
	}
	// TODO: Make this work later
	if Drone.GroundIsKnown == true {
		fmt.Println(Drone.DroneName, ":", Drone.DroneFullName, "INIT: GROUND LOCATED")
		changeState(StateConnected)
		Drone.DroneKeepAliveRcvdTime = time.Now()
	}

	//================================================================================
	// START TIMER : Call periodicFunc on every timerTick
	//================================================================================
	tick := 300 * time.Millisecond
	fmt.Println(Drone.DroneName, "MAIN: Starting a new Timer Ticker at ", tick, " msec")
	ticker := time.NewTicker(tick)
	go func() {
		for t := range ticker.C {
			//Call the periodic function here.
			periodicFunc(t)
		}
	}()

	//================================================================================
	// START CONSOLE:
	//================================================================================
	StartConsole(ConsoleInput)

	//================================================================================
	// RECEIVE AND PROCESS MESSAGES: Control Plane msgs, and Commands from console
	// Note that this software is implemented as FSM with run to completion
	//================================================================================
	for {
		select {
		case UnicastMsg := <-Drone.DroneUnicastRcvCtrlChannel:
			fmt.Println(Drone.DroneName, "MAIN: Unicast MSG in state", Drone.DroneState, "MSG=", string(UnicastMsg))
			// these include text messages from the ground/controller
			ControlPlaneMessages(UnicastMsg)
		case BroadcastMsg := <-Drone.DroneBroadcastRcvCtrlChannel:
			//fmt.Println(Drone.DroneName, "MAIN: Broadcast MSG in state", Drone.DroneState, "MSG=",string(BroadcastMsg))
			// these include text messages from the ground/controller
			ControlPlaneMessages(BroadcastMsg)
		case MulticastMsg := <-Drone.DroneMulticastRcvCtrlChannel:
			//fmt.Println(Drone.DroneName, "MAIN: Multicast MSG in state", Drone.DroneState, "MSG=",string(MulticastMsg))
			// these include text messages from the ground/controller
			ControlPlaneMessages(MulticastMsg)
		case CmdText, ok := <-ConsoleInput: // These are messsages from local drone console
			if !ok {
				fmt.Println("ERROR Reading input from stdin:", CmdText)
				break
			} else {
				fmt.Println("Read input from stdin:", CmdText)
				LocalCommandMessages(CmdText)
				// SendTextMsg(stdin)
				// fmt.Println("Console input sent to ground");
			}
		case GpsInfoMsg := <-Drone.GpsDeviceChannel:
			fmt.Println(GpsInfoMsg)
			//default:
			//	fmt.Println("done and exit select")
		} // EndOfSelect
	} // EndOfFOR
	ControlPlaneCloseConnections()
	// os.Exit(0)
}

//====================================================================================
// periodicFunc(tick) - executed every tick - originaly every 3 sec
// TODO add command to change time periods
//====================================================================================
func round(val float64) int {
	if val < 0 {
		return int(val - 0.5)
	}
	return int(val + 0.5)
}
func periodicFunc(tick time.Time) {
	// TODO - figure out reasonable timer period to process these two
	// Note that we may have not receive messages from some drones for some time
	// So maybe wait until the end of processing period and figure out who we
	// received discovery msgs from and based on that figure out the connectivity

	// For testing: step mode will stop the processing and/or sending any messages
	// Note that step command may define # of runs to be processed as opposed immediate stop
	// We will still process any subsequent commands from the ground/control
	if Drone.StepMode == 0 { // we are paused now untill next step command
		//fmt.Println("STEP MODE = 0, REMAIN FROZEN, SKIP TIMER TICK")
		return
	} else if Drone.StepMode > 0 { // we still have few steps to do
		Drone.StepMode--
	}
	// fmt.Println("TICK: UPDATE CONNECTIVITY ------------------------")
	UpdateConnectivity()
	Me.NodeDistanceToGround = DistanceToGround(Me) // update the distance to ground
	DroneColor(Me.NodeRole, Me.NodeIamAlone)
	//BMprintIDs(&Me.SubscriberList,  "Tick-OUT: SUBSCRIBERS:")
	//BMprintIDs(&Me.BaseStationList, "Tick-OUT: BASESTATION:")
	currTimeMilli := TBtimestampMilli()
	tt := currTimeMilli - Drone.LastDiscoverySent

	if tt > Drone.DiscoveryInterval { // in nano sec
		sendBroadcastDiscoveryPacket()
		Drone.LastDiscoverySent = currTimeMilli
	}

	if Drone.DroneActivityTimer > 0 {
		Drone.DroneActivityTimer--
		if Drone.DroneActivityTimer == 0 {
			if isGroundKnown() == true {
				//fmt.Println(Drone.DroneName, "CONNECTED TO MASTER/GROUND")
				changeState(StateConnected)
				Drone.DroneKeepAliveRcvdTime = time.Now()
			} else {
				//fmt.Println(Drone.DroneName, "NO CONNECTION TO MASTER/GROUND")
				Drone.DroneActivityTimer = Drone.DroneConnTimer // 3*5=15 sec, check periodic timer above
			}
		}
	} else {
		currTime := time.Now()
		elapsedTime := currTime.Sub(Drone.DroneKeepAliveRcvdTime)
		//fmt.Println("Elapsed time=", elapsedTime)
		if elapsedTime > (time.Second * 3) {
			if isGroundKnown() == true {
				changeState(StateConnected)
				Drone.DroneKeepAliveRcvdTime = time.Now()
			} else {
				changeState(StateConnecting)
				Drone.DroneActivityTimer = Drone.DroneConnTimer //int(round(time.Second.Seconds))
				// 3*5=15 sec, check periodic timer above
			}
		}
	}
}

//====================================================================================
//
//====================================================================================
func getSize(v interface{}) int {
	size := int(reflect.TypeOf(v).Size())
	switch reflect.TypeOf(v).Kind() {
	case reflect.Slice:
		s := reflect.ValueOf(v)
		for i := 0; i < s.Len(); i++ {
			size += getSize(s.Index(i).Interface())
		}
	case reflect.Map:
		s := reflect.ValueOf(v)
		keys := s.MapKeys()
		size += int(float64(len(keys)) * 10.79) // approximation from https://golang.org/src/runtime/hashmap.go
		for i := range keys {
			size += getSize(keys[i].Interface()) + getSize(s.MapIndex(keys[i]).Interface())
		}
	case reflect.String:
		size += reflect.ValueOf(v).Len()
	case reflect.Struct:
		s := reflect.ValueOf(v)
		for i := 0; i < s.NumField(); i++ {
			if s.Field(i).CanInterface() {
				size += getSize(s.Field(i).Interface())
			}
		}
	}
	return size
}

//====================================================================================
// ControlPlaneMessages() - handle Control Plane messages
//====================================================================================
func ControlPlaneMessages(message []byte) {
	msg := new(Msg)
	err1 := TBunmarshal(message, &msg)
	if err1 != nil {
		println("Error unmarshalling message: ", msg.MsgHeader.MsgCode)
		return
	}
	msgHeader := &msg.MsgHeader
	sender := msgHeader.SrcId
	//msglen := len(message)
	fmt.Println("ControlPlaneMessages: Msg=", msg) //.MsgCode, " msglen=", msglen)

	// Was this msg originated by us ?
	if strings.Contains(msgHeader.SrcIP, Drone.DroneIpAddress) && sender == Me.NodeId {
		// println("My own message: MsgCode=", msgHeader.MsgCode, " Me.NodeId=", Me.NodeId)
		return
	}
	//============================================================================
	// Is the other side within the RF range ?
	// we need to do this for ethernet connectivity as we receive everything
	//============================================================================
	// First check that the senders id is in valid range
	if sender == Me.NodeId || sender < 1 || sender > MAX_NODES {
		println("Sender id WRONG: ", sender, " MsgCode=", msgHeader.MsgCode)
		return
	}
	node := &Drone.NodeList[sender-1]
	// are we in range ?
	if IsNodeInRange(Me, node, msgHeader.LocationX, msgHeader.LocationY, msgHeader.LocationZ) == false &&
		node.NodeId > 0 && node.NodeId <= MAX_NODES {
		fmt.Println("NODE", msgHeader.SrcId, " NOT IN RANGE, DELETE MESSAGE X=",
			msgHeader.LocationX, "  Y=", msgHeader.LocationY)
		// Forget everything about this node
		BMremoveID(&Me.BaseStationList, node.NodeId-1)
		BMremoveID(&Me.SubscriberList, node.NodeId-1)
		node.NodeActive = false
		return
	}
	fmt.Println("CHECK MESSAGE CODE ", msgHeader.MsgCode)
	switch msgHeader.MsgCode {
	case MSG_TYPE_DISCOVERY: // from another drone
		if Drone.StepMode == 0 { // we are paused now untill next step command
			return
		} // dont process DISCOVERY messages
		var discoveryMsg = new(MsgCodeDiscovery)
		err := TBunmarshal(message, discoveryMsg) //message
		if err != nil {
			println("ControlPlaneMessages: ERR=", err)
			return
		}
		ControlPlaneProcessDiscoveryMessage(msgHeader, &discoveryMsg.MsgDiscovery)
		break
	case MSG_TYPE_GROUND_INFO: // info from ground
		// TODO: will require some rethinking how to handle
		// TODO: may need to rebroadcast for nodes that aare out of range
		// Note that in order to cure the situation where a node might have been out of reach
		// at the time the STEP message was sent, GROUND will insert the latest value for
		//the StepMode in all GROUNDINFO messages .... but we need to process those ...
		if sender == GROUND_STATION_ID {
			Drone.StepMode = msgHeader.StepMode
			//fmt.Println("STEP MODE = ", Drone.StepMode, "CHANGED TO VALUE FROM GROUNDINFO") ///
		}
		handleGroundInfoMsg(msgHeader, message)
		break
	case MSG_TYPE_DRONE_MOVE: // command from ground
		break
	case MSG_TYPE_STATUS_REQ: // command from ground
		handleGroundStatusRequest(msgHeader, message)
		break
	case MSG_TYPE_STEP: // from ground
		handleGroundStepMsg(msgHeader, message)
		break
	case "UPDATE":
		break
	default:
		fmt.Println("ControlPlaneMessages:  UNKNOWN Message")
		break
	}
}

//====================================================================================
// ControlPlaneMessage STEP
//====================================================================================
func handleGroundStepMsg(msgHeader *MessageHeader, message []byte) {
	/*
		var stepMsg = new(MsgCodeStep)
		err := TBunmarshal(message, stepMsg) //message
		if err != nil {
			println("ControlPlaneMessages STEP: ERR=", err)
			return
		}
		println("STEP MESSAGE step=", stepMsg.MsgStep.Steps)
		Drone.StepMode = stepMsg.MsgStep.Steps
	*/

	Drone.StepMode = msgHeader.StepMode
}

//====================================================================================
// ControlPlaneMessage STATUS REQ
//====================================================================================
func handleGroundStatusRequest(msgHeader *MessageHeader, message []byte) {
	//fmt.Println("...... STATUS REQUEST MESSAGE ................")
	fmt.Println("...... STATUS REQUEST: srcIP=", msgHeader.SrcIP, " SrcMAC=", msgHeader.SrcMAC,
		" DstID=", msgHeader.DstId, " SrcID=", msgHeader.SrcId)

	// REPLY
	sendUnicastStatusReplyPacket(msgHeader)
}

//====================================================================================
// ControlPlaneMessage   GROUNDINFO
//====================================================================================
func handleGroundInfoMsg(msgHeader *MessageHeader, message []byte) {
	var err error
	// TODO  ... add to msg the playing field size .... hmmm ?? relation to random etc
	Drone.GroundFullName.Name = msgHeader.SrcName //.DstName
	Drone.GroundIP = msgHeader.SrcIP
	Drone.GroundIPandPort = string(msgHeader.SrcIP) + ":" + strconv.Itoa(msgHeader.SrcPort)
	Drone.GroundUdpPort = msgHeader.SrcPort
	Drone.GroundIsKnown = true //msg.GroundUp
	Drone.GroundLatitude = msgHeader.Latitude
	Drone.GroundLongitude = msgHeader.Longitude
	// latitude,longitude := ConvertXYZtoLatLong(Me.MyX, Me.MyY, Me.MyZ, GpsGroundHeightFromEartCenter)
	// x,y,z := ConvertLatLongToXYZ(Me.NodeLatitude, Me.NodeLongitude, Me.NodeDistanceToGround)
	x, y, z := ConvertLatLongToXYZ(msgHeader.Latitude, msgHeader.Longitude, Drone.GpsGroundHeightFromEartCenter)
	// TODO later use x,y,z instead of LocationX,Y,Z
	Drone.GroundX = msgHeader.LocationX // TODO
	Drone.GroundY = msgHeader.LocationY
	Drone.GroundZ = msgHeader.LocationZ
	Drone.GroundRadioRange = float64(msgHeader.RadioRange)

	fmt.Println("DISTANCE from ME to Ground =", DistanceToGround(Me))
	fmt.Println("Ground position from msg GroundX=", Drone.GroundX, " Y=", Drone.GroundY,
		" Z=", Drone.GroundZ, " Calc From msg Lat/Long X=", x, " Y=", y, " Z=", z)
	fmt.Println("DISCOVERY from msg: Latitude=", msgHeader.Latitude, " Longitude=", msgHeader.Longitude)
	latitude, longitude := ConvertXYZtoLatLong(Drone.GroundX, Drone.GroundY, Drone.GroundZ, Drone.GpsGroundHeightFromEartCenter)
	fmt.Println("DISCOVERY conv from msg XYZ coord: Latitude=", latitude, " Longitude=", longitude)

	// TESTx,y,z
	lat, lon := ConvertXYZtoLatLong(20, 100, 0, Drone.GpsGroundHeightFromEartCenter)
	x, y, z = ConvertLatLongToXYZ(lat, lon, Drone.GpsGroundHeightFromEartCenter)
	fmt.Println("TEST: lat=", lat, " long=", lon, " x=", x, " y=", y, " z=", z)
	lat, lon = ConvertXYZtoLatLong(x, y, z, Drone.GpsGroundHeightFromEartCenter)
	fmt.Println("TEST: Reverse: lat=", lat, " long=", lon, " x=", x, " y=", y, " z=", z)

	// Drone.GroundUdpAddrSTR.IP = msg.TxIP;
	//fmt.Println(TERMCOLOR, "... GROUNDINFO: Name=", Drone.GroundFullName.Name,
	//	"  IP:Port=", Drone.GroundIPandPort, " StepMode=", msgHeader.StepMode)
	// fmt.Println("Port=", msg.TxPort, " txIP=", msg.TxIP, " " +
	//	"groundIP=",Drone.GroundIP)

	Drone.GroundUdpAddrSTR, err = net.ResolveUDPAddr("udp", Drone.GroundIPandPort)
	//Drone.GroundUdpAddrSTR.Port = msg.TxPort
	//Drone.GroundUdpAddrSTR.IP   = net.IP(msg.TxIP)
	//Drone.GroundUdpAddrSTR.IP 			 = net.IP((Drone.GroundIP))
	//Drone.GroundUdpAddrSTR.Port, _ 	 = strconv.Atoi(Drone.GroundUdpPort)
	//Drone.GroundUdpAddrSTR.IP	 = Drone.GroundIP // net.IP(Drone.GroundIP)

	//fmt.Println("1 Drone.GroundUdpAddrSTR=", Drone.GroundUdpAddrSTR)
	Drone.GroundUdpAddrSTR = &net.UDPAddr{IP: net.ParseIP(msgHeader.DstIP), Port: msgHeader.DstPort}
	Drone.GroundFullName = NameId{Name: msgHeader.DstName, Address: *Drone.GroundUdpAddrSTR}
	//fmt.Println("2 Drone.GroundUdpAddrSTR=", Drone.GroundUdpAddrSTR)

	if err != nil {
		fmt.Println("ERROR in net.ResolveUDPAddr = ", err)
		fmt.Println("ERROR locating master, will retry")
		return
	} else {
		// fmt.Println("GROUND INFO: Name=", Drone.GroundFullName.Name, "  IP:Port=", Drone.GroundIPandPort)
	}
}
func xParseIP(s string) string {
	ip, _, err := net.SplitHostPort(s)
	if err == nil {
		return ip //, nil
	}

	ip2 := net.ParseIP(s)
	if ip2 == nil {
		return "" //, errors.New("invalid IP")
	}

	return ip2.String() //, nil
}

//====================================================================================
// Handle DISCOVERY messages in all states
//====================================================================================
func ControlPlaneProcessDiscoveryMessage(msgHeader *MessageHeader,
	discoveryMsg *DiscoveryMsgBody) {
	//fmt.Println("Discovery MSG in state ", Drone.DroneState)
	switch Drone.DroneState {
	case StateAlone:
		stateConnectedDiscoveryMessage(msgHeader, discoveryMsg)
		break
	case StateConnecting:
		stateConnectedDiscoveryMessage(msgHeader, discoveryMsg)
		break
	case StateConnected:
		stateConnectedDiscoveryMessage(msgHeader, discoveryMsg)
		break
	default:
	}
}

//==========================================================================
//
//===========================================================================
func stateConnectedDiscoveryMessage(msgHeader *MessageHeader, discoveryMsg *DiscoveryMsgBody) {
	sender := msgHeader.SrcId
	if sender < 1 || sender > MAX_NODES || sender == Me.NodeId {
		// fmt.Println("DISCARD MSG: invalid senderId=", sender)
		return
	}
	node := &Drone.NodeList[sender-1]
	if DEBUG_DISCOVERY != 0 {
		fmt.Println("DISCOVERY ", sender, " DistToGround=", DistanceToGround(node), " ROLE=", NodeRole(msgHeader.SrcRole),
			" srcIP=", msgHeader.SrcIP, ":", msgHeader.SrcPort,
			" SrcName=", msgHeader.SrcName, " src MAC=", msgHeader.SrcMAC, " DstName=", msgHeader.DstName)
	}
	// update info for the sending drone
	node.NodeName = msgHeader.SrcName
	node.NodeId = msgHeader.SrcId
	node.NodeIP = msgHeader.SrcIP
	node.NodeMac = msgHeader.SrcMAC
	node.NodePort = msgHeader.SrcPort
	node.NodeMsgSeq = msgHeader.SrcSeq
	node.NodeLongitude = msgHeader.Longitude
	node.NodeLatitude = msgHeader.Latitude
	node.NodeRadioRange = msgHeader.RadioRange
	node.MyX = float64(msgHeader.LocationX) // TODO
	node.MyY = float64(msgHeader.LocationY)
	node.MyZ = float64(msgHeader.LocationZ)

	node.NodeTimeCreated = discoveryMsg.TimeCreated // incarnation #
	node.NodeLastChangeTime = discoveryMsg.LastChangeTime
	node.NodeActive = discoveryMsg.NodeActive
	node.NodeMsgsSent = discoveryMsg.MsgsSent
	node.NodeMsgsRcvd = discoveryMsg.MsgsRcvd
	node.NodeMsgLastSentAt = discoveryMsg.MsgLastSentAt
	node.NodeMsgLastRcvdRemote = discoveryMsg.MsgLastRcvdAt
	g, _ := strconv.ParseUint(discoveryMsg.Gateways, 10, 64)
	node.GatewayList.M_Mask = g // discoveryMsg.Gateways
	s, _ := strconv.ParseUint(discoveryMsg.Subscribers, 10, 64)
	node.SubscriberList.M_Mask = s // discoveryMsg.Subscribers
	b, _ := strconv.ParseUint(discoveryMsg.BaseStations, 10, 64)
	node.BaseStationList.M_Mask = b // discoveryMsg.BaseStations

	node.Velocity = discoveryMsg.Velocity
	node.VelocityX = float64(discoveryMsg.VelocityX)
	node.VelocityY = float64(discoveryMsg.VelocityY)
	node.VelocityZ = float64(discoveryMsg.VelocityZ)
	node.Xdirection = 1
	node.Ydirection = 1
	node.Zdirection = 1
	if node.VelocityX < 0 {
		node.Xdirection = -1
	}
	if node.VelocityY < 0 {
		node.Ydirection = -1
	}
	if node.VelocityZ < 0 {
		node.Zdirection = -1
	}

	node.NodeMsgLastRcvdLocal = time.Now() // TBtimestampNano() // time.Now()
	Drone.LastFrameRecvd[sender-1] = time.Now().String()
	// TODO:
	// node.ResidualHop
	//SubscriberList  BitMask
	//BaseStationList BitMask
	//senderNode = &Drone.NodeList[nodeId]
	//senderNode.NodeMac = msgHeader.SrcMAC // eth->getSrcMac(desc);
	//senderNode.NodeIP = msgHeader.SrcIP
	// Update sender type to whatever he is telling us
	//senderNode.NodeRole = int(msgHeader.SrcRole)
	// For the Sender replace subsriber and BaseStations lists with new ones:
	//BMset(&node.SubscriberList, node.Subscribers)
	//BMset(&node.BaseStationList, node.BaseStations)
	// Remember all node we learned about:
	// TODO Hmm ... should we forget those deleted by sender TODO

	/* // GORAN Mar05 comment out
	if node.NodeRole == BASE_STATION {
		//fmt.Println("Updated BaseStationList=", Me.BaseStationList.M_Mask)
		BMaddID(&Me.BaseStationList, int(node.NodeId - 1)) //nodeId)
		//fmt.Println("Updated BaseStationList=", Me.BaseStationList.M_Mask)
	} else {
		//fmt.Println("Updated SubscriberList=", Me.SubscriberList.M_Mask)
		BMaddID(&Me.SubscriberList, int(node.NodeId -1)) //.addID(nodeId);
		//fmt.Println("Updated SubscriberList=", Me.SubscriberList.M_Mask)
	}
	*/
	node.NodeActive = true

	if Drone.GroundIsKnown {
		node.NodeDistanceToGround = DistanceToGround(node)
		Me.NodeDistanceToGround = DistanceToGround(Me)
		// did this DISCOVERY reach the ground as well

		if node.NodeDistanceToGround > Drone.GroundRadioRange {
			fmt.Println("=== NEED TO FORWARD, OTHER NODE ", int(node.NodeDistanceToGround),
				" AWAY FROM GROUND, Ground at ", Drone.GroundRadioRange)
			theNode, distance := FindShortestConnectionToGround()
			fmt.Println("====== Me=", Me.NodeId, " MY DISTANCE=", Me.NodeDistanceToGround,
				" HIS DISTANCE=", node.NodeDistanceToGround, " SHORTEST=", distance,
				" Forwarder=", theNode.NodeId)
			if Me == theNode && Me.NodeDistanceToGround <= Drone.GroundRadioRange {
				fmt.Println("====== I AM THE FORWARDER ===========================")
				// forward the DISCOVERY as UNICAST to GROUND
				forwardUnicastDiscoveryPacket(msgHeader, discoveryMsg, int(distance))
			}
		} else {
			//fmt.Println("==NODE ", node.NodeId, " CAN REACH GROUND AT",node.DistanceToGround ,
			//	" Ground at ", Drone.GroundRadioRange)
		}
	}
}

func FindShortestConnectionToGround() (*NodeInfo, float64) {
	var theNode *NodeInfo = nil
	var distance float64 = 1000000000
	for j := 0; j < 64; j++ {
		if Drone.NodeList[j].NodeActive == true {
			if distance > Drone.NodeList[j].NodeDistanceToGround {
				distance = Drone.NodeList[j].NodeDistanceToGround
				theNode = &Drone.NodeList[j]
			}
		}
	}
	return theNode, distance
}

//====================================================================================
// Handle messages received in the CONNECTED state
//====================================================================================
func RemoteCommandMessages(msg *Msg) {
	var cmds []LinuxCommand
	// _ = TBunmarshal(msg.MsgBody, &cmds)

	for cmdIndex := range cmds {
		var cmd LinuxCommand
		cmd = cmds[cmdIndex]
		err := RunLinuxCommand(REMOTE_CMD, cmd.Cmd, cmd.Par1, cmd.Par2, cmd.Par3, cmd.Par4, cmd.Par5, cmd.Par6)
		if err != nil {
			fmt.Printf("%T\n", err)
		}
	}
}

//=======================================================================
//
//=======================================================================
func LocalCommandMessages(cmdText string) {
	//var cmd []string
	//cmd = strings.Split(cmdText, " ")

	switch cmdText {
	case "enable":
		Me.NodeActive = true
	case "disable":
		Me.NodeActive = false
	default:
	}
	fmt.Println("RCVD CONSOLE INPUT =", cmdText, " DRONE ACTIVE=", Me.NodeActive)
	// TODO figure out the bellow line
	//err := RunLinuxCommand(LOCAL_CMD, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6])
	//if err != nil {fmt.Printf("%T\n", err)}

}

//=======================================================================
//
//=======================================================================
func RunLinuxCommand(origin, Cmd, Par1, Par2, Par3, Par4, Par5, Par6 string) error {
	//fmt.Println("RCVD CMD ", cmdIndex, " =",cmd)
	// fmt.Println("cmd=", cmd.Cmd, " ", cmd.Par1, " ", cmd.Par2, " ", cmd.Par3, " ", cmd.Par4, " ", cmd.Par5, " ", cmd.Par6)
	//cmd.Output() → run it, wait, get output
	//cmd.Run() → run it, wait for it to finish.
	//cmd.Start() → run it, don't wait. err = cmd.Wait() to get result.
	var thisCmd = exec.Command(Cmd, Par1, Par2, Par3, Par4, Par5, Par6)
	output, err := thisCmd.Output()
	//if err != nil && err.Error() != "exit status 1" {
	//	fmt.Println("CMDx=", cmd.Cmd, " ", cmd.Par1, " ", cmd.Par2, " ", cmd.Par3, " ", cmd.Par4,
	//		" ", cmd.Par5, " ", cmd.Par6, " :  cmd.Run() failed with ", err)
	//} else {
	//if err != nil && err.Error() != "exit status 1" {
	//	//panic(err)
	//	//fmt.Printf("ERROR=", err, "\n")
	//	fmt.Printf("%T\n", err)
	//} else {
	//	fmt.Printf("CMD OUTPUT=",string(output))
	//	// SEND REEPLY, OR MAYBE COMBINED ALL FIRST
	//}
	fmt.Println(origin, " CMD= ", Cmd, " ", Par1, Par2, " ", Par3, " ", Par4,
		" ", Par5, " ", Par6, " :  RESULT:", string(output), "  ERR:", err)
	return err
}

//====================================================================================
//  Set new state
//====================================================================================
func changeState(newState string) {
	// fmt.Println(Drone.DroneName, "OldState=", Drone.DroneState, " NewState=", newState)
	Drone.DroneState = newState
}

//====================================================================================
//  Initialize my own info in the Drone structure
//====================================================================================
func SetFinalDroneInfo() {

	Me = &Drone.NodeList[Drone.DroneId-1]
	if Drone.DroneIpAddress == "" {
		Drone.DroneIpAddress, Drone.DroneMacAddress = GetLocalIp() // get IP and MAC
	}

	Me.NodeId = Drone.DroneId
	Me.NodeRole = SUBSCRIBER
	Me.NodeIamAlone = true
	Me.NodeIP = Drone.DroneIpAddress
	Me.NodePort, _ = strconv.Atoi(Drone.DroneUdpPort)
	Me.NodeMac = Drone.DroneMacAddress
	Me.NodeName = Drone.DroneName
	DroneColor(ALONE, Me.NodeIamAlone) //black

	Me.NodeRadioRange = int(Drone.DroneRadioRange) // RANGE_3D or RANGE_2D

	InitializeVelocity()
	// TODO memset(&distanceVector, 0, sizeof(distanceVector))
	Me.NodeLastChangeTime = float64(TBtimestampNano()) // time.Now().String()
	Me.NodeActive = true
	Drone.DroneState = StateAlone

	BMclear(&Me.BaseStationList)
	BMclear(&Me.SubscriberList)
	BMclear(&Me.GatewayList)

	var bit uint64 = 0x1
	for j := 0; j < 64; j++ {
		// TODO .. from prev coded, should be just = 0xffffffffffffffff
		Me.GatewayList.M_BitMask[j] = bit
		bit <<= 1
	}
	for droneNumber := 0; droneNumber < MAX_NODES; droneNumber++ {
		Drone.NodeList[droneNumber].NodeMsgLastRcvdLocal = time.Now() // TBtimestampNano() //time.Now()
	}
	Me.NodeLastMove = TBtimestampNano() // nano seconds
	Drone.StepMode = STEPMODE_DISABLED
	Drone.DiscoveryInterval = 1000 // millisec
	Drone.LastDiscoverySent = TBtimestampMilli()
}

//====================================================================================
// InitFromCommandLine()  READ ARGUMENTS IF ANY
//====================================================================================
func InitFromCommandLine() {
	// argsWithProg := os.Args; argsWithoutProg := os.Args[1:]
	// Second: check for the command line parametersz, they overwrite the config file
	for index := range os.Args {
		arg := os.Args[index]
		fmt.Println("Arg", index, "=", arg)
	}
	argNum := len(os.Args) // Number of arguments supplied, including the command
	fmt.Println("Number of Arguments = ", argNum)
	if argNum > 2 && os.Args[1] != "" && os.Args[1] != "0" {
		fmt.Println("Drone NAME = ", os.Args[1])
		Drone.DroneName = os.Args[1]
		Drone.DroneId, _ = strconv.Atoi(os.Args[2])
	}
	if argNum > 3 && os.Args[3] != "" && os.Args[3] != "0" {
		fmt.Println("Satellite IP   = ", os.Args[3])
		Drone.DroneIpAddress = os.Args[3]
		// TODO: Set eth0 IP address to Drone.DroneIpAddress !!!
		var ifCmd = exec.Command("sudo", "ifconfig", "eth0", Drone.DroneIpAddress, "up", "", "")
		output, err := ifCmd.Output()
		fmt.Println("SET MY IP=", "sudo", "ifconfig", "eth0", Drone.DroneIpAddress, "up", " -OUTPUT:", string(output), " ERR:", err)
	}
	if argNum > 4 && os.Args[4] != "" && os.Args[4] != "0" {
		Drone.DroneUdpPort = os.Args[4] // strconv.ParseInt(os.Args[3], 10, 64)
	}
	if argNum > 5 && os.Args[5] != "" && os.Args[5] != "0" && argNum > 6 && os.Args[6] != "" && os.Args[6] != "0" {
		Drone.GroundIPandPort = os.Args[5] + ":" + os.Args[6]
	}
}

//====================================================================================
// InitFromConfigFile() - Set configuration from config file
//====================================================================================
func InitFromConfigFile() {
	var fileName string
	argNum := len(os.Args) // Number of arguments supplied, including the command
	fmt.Println("Number of Arguments = ", argNum)
	if argNum > 2 && os.Args[1] != "" && os.Args[1] != "0" {
		Drone.DroneName = os.Args[1]
		Drone.DroneId, _ = strconv.Atoi(os.Args[2])
		fileName = "config" + os.Args[2] + ".yml"
		viper.SetConfigName(fileName)
	} else {
		// Set the file name of the configurations file
		fileName = "config.yml"
		viper.SetConfigName(fileName)
	}
	// Set the path to look for the configurations file
	viper.AddConfigPath(".")
	// Enable VIPER to read Environment Variables
	viper.AutomaticEnv()
	viper.SetConfigType("yml")
	// var droneStruct DroneInfo
	if err := viper.ReadInConfig(); err != nil {
		fmt.Printf("Error reading config file, %s", err)
		return
	}

	// Set undefined variables
	viper.SetDefault("DroneName", "Drone1")
	// store configuration into the drone structure
	err := viper.Unmarshal(&Drone) //&droneStruct)
	if err != nil {
		fmt.Printf("Unable to decode into struct, %v", err)
	}

	fmt.Println("Reading variables from ... config", fileName)
	fmt.Println("*************************************************************")
	fmt.Println("DroneId                   = ", Drone.DroneId)
	fmt.Println("DroneName                 = ", Drone.DroneName)
	fmt.Println("DroneIpAddress            = ", Drone.DroneIpAddress)
	fmt.Println("DroneLogPath              = ", Drone.DroneLogPath)
	fmt.Println("DroneConnTimer            = ", Drone.DroneConnTimer)
	fmt.Println("DroneUdpPort              = ", Drone.DroneUdpPort)

	fmt.Println("BroadcastRxIP             = ", Drone.BroadcastRxIP)
	fmt.Println("BroadcastRxPort           = ", Drone.BroadcastRxPort)
	Drone.BroadcastRxAddress = Drone.BroadcastRxIP + ":" + Drone.BroadcastRxPort
	fmt.Println("BroadcastRxAddress        = ", Drone.BroadcastRxAddress)

	fmt.Println("UnicastRxIP             = ", Drone.UnicastRxIP)
	fmt.Println("UnicastRxPort           = ", Drone.UnicastRxPort)
	Drone.UnicastRxAddress = Drone.UnicastRxIP + ":" + Drone.UnicastRxPort
	fmt.Println("UnicastRxAddress        = ", Drone.UnicastRxAddress)

	fmt.Println("BroadcastTxIP             = ", Drone.BroadcastTxIP)
	fmt.Println("BroadcastTxPort           = ", Drone.BroadcastTxPort)
	Drone.BroadcastTxAddress = Drone.BroadcastTxIP + ":" + Drone.BroadcastTxPort
	fmt.Println("BroadcastTxAddress        = ", Drone.BroadcastTxAddress)

	fmt.Println("GroundId                  = ", Drone.GroundIP)
	fmt.Println("GroundUdpPort             = ", Drone.GroundUdpPort)
	fmt.Println("GroundIPandPort           = ", Drone.GroundIPandPort)
	fmt.Println("MaxDatagramSize           = ", Drone.MaxDatagramSize)
	fmt.Println("DroneSpaceDimension       = ", Drone.DroneSpaceDimension)
	fmt.Println("DroneRadioRange           = ", Drone.DroneRadioRange)
	fmt.Println("DiscoveryInterval         = ", Drone.DiscoveryInterval)
	fmt.Println("DroneDiscoveryTimeout     = ", Drone.DroneDiscoveryTimeout)
	fmt.Println("Velocity                  = ", Drone.Velocity)
	fmt.Println("GPS Device                = ", Drone.GpsDevice)
	// TODO set all info from Drone.x  to Me.
}

//====================================================================================
// InitDroneFromConfig()  READ ARGUMENTS IF ANY
//====================================================================================
func InitDroneConfiguration() {

	Drone.DroneName = "Drone01" // will be overwritten by config
	Drone.DroneId = 1           // will be overwritten by config
	Drone.DroneIpAddress = ""
	Drone.DroneConnTimer = DRONE_KEEPALIVE_TIMER // 5
	Drone.DroneKeepAliveRcvdTime = time.Now()
	// Drone.KnownDrones 			= nil // Array of drones and ground stations learned
	Drone.DroneCmdChannel = nil            // so that all local threads can talk back
	Drone.DroneUnicastRcvCtrlChannel = nil // to send control msgs to Recv Thread
	Drone.DroneBroadcastRcvCtrlChannel = nil
	Drone.DroneMulticastRcvCtrlChannel = nil
	Drone.DroneConnection = nil
	Drone.StepMode = -1
	Drone.DroneActive = true
	Drone.GpsMyGeometry = ellipsoid.Ellipsoid{}
	Drone.GpsGroundHeightFromEartCenter = 6377.563 // kilo meters
	Drone.GpsOrbitAltitudeFromGround = 1000.0      // kilo meters
	Drone.GpsPlayingFieldSide = 100000.0           // meters
	Drone.GpsReferenceLatitude = 34.055912
	Drone.GpsReferenceLongitude = -118.46478
	Drone.DroneSpaceDimension = "2D"
	Drone.DroneRadioRange = RANGE_2D
	Drone.DroneDiscoveryTimeout = 1500 // milli sec
	Drone.DroneReceiveCount = 0
	Drone.DroneSendCount = 0
	Log.DebugLog = true
	Log.WarningLog = true
	Log.ErrorLog = true

	Drone.DroneUnicastRcvCtrlChannel = make(chan []byte) //
	Drone.DroneBroadcastRcvCtrlChannel = make(chan []byte)
	Drone.DroneMulticastRcvCtrlChannel = make(chan []byte)
	Drone.DroneCmdChannel = make(chan []byte) // receive command line cmnds

	Drone.BroadcastRxAddress = ":9999"
	Drone.BroadcastRxPort = "9999"
	Drone.BroadcastRxIP = ""
	Drone.BroadcastTxPort = "8888"
	Drone.BroadcastConnection = nil
	Drone.BroadcastTxStruct = new(net.UDPAddr)

	Drone.UnicastRxAddress = ":8888"
	Drone.UnicastRxPort = "8888"
	Drone.UnicastRxIP = ""
	Drone.UnicastTxPort = "8888"
	Drone.UnicastRxConnection = nil
	Drone.UnicastRxStruct = nil
	Drone.UnicastTxStruct = new(net.UDPAddr)
	Drone.Velocity = 0
	// TODO - these are randomly generated, just for history (last setup)
	Drone.VelocityScaleX = 50
	Drone.VelocityScaleY = 30
	Drone.VelocityScaleZ = 0
	Drone.GpsDevice = "NONE"
	Drone.GroundIsKnown = false
	Drone.GroundUdpPort = 0
	Drone.GroundIP = ""
	Drone.DroneUdpPort = "8888"
	Drone.GroundIPandPort = "" //Drone.GroundIP + ":" + Drone.GroundUdpPort
	Drone.DroneUdpAddrSTR = new(net.UDPAddr)
	Drone.GroundUdpAddrSTR = new(net.UDPAddr)
	Drone.DroneCreationTime = strconv.FormatInt(TBtimestampNano(), 10)
	Drone.GpsReferenceLatitude = 34.055912
	Drone.GpsReferenceLongitude = -118.46478
}

//====================================================================================
//  Initialize IP and UDP addressing
//====================================================================================
func InitDroneConnectivity() {

	//TODO rework completely
	if strings.Contains(Drone.DroneIpAddress, "172.") {
		//sa := strings.Split(Drone.DroneIpAddress, ".")
		//Drone.BroadcastIPandPort 	= sa[0]+"."+sa[1]+"."+sa[2]+".255:8888"
		//Drone.BroadcastIP 			= sa[0]+"."+sa[1]+"."+sa[2]+ ".255"
	} else { // Home router setup
		//Drone.BroadcastIPandPort 	= "10.0.1.255:8888"
		//Drone.BroadcastIP 			= "10.0.1.255"
		//Drone.DroneIpAddress 		= "10.0.1.244"
	}

	Drone.DroneIPandPort = Drone.DroneIpAddress + ":" + Drone.DroneUdpPort
	var err3 error
	Drone.DroneUdpAddrSTR, err3 = net.ResolveUDPAddr("udp", Drone.DroneIPandPort)
	if err3 != nil {
		fmt.Println("ERROR ResolveUDPAddr: ", err3)
	} else {
		Drone.DroneFullName = NameId{Name: Me.NodeName, Address: *Drone.DroneUdpAddrSTR}
	}
}

//====================================================================================
// Check if we are talking to the ground/controller station
//====================================================================================
func isGroundKnown() bool {

	return Drone.GroundIsKnown
	// NOTE that this will fail if Drone.GroundUdpAddress has not been initialized due
	// to master not being up . Add state and try again to Resolve before doing this
	//var err error
	//fmt.Println("Locate master control, not ground")
	//Drone.GroundUdpAddrSTR, err = net.ResolveUDPAddr("udp", Drone.GroundIPandPort)
	//if err != nil {
	//	fmt.Println("ERROR in net.ResolveUDPAddr = ", err)
	//	fmt.Println("ERROR locating master, will retry")
	//	return false
	//}
	// TODO check if this is needed
	//Drone.GroundFullName = tbMessages.NameId{Name:Drone.GroundIPandPort,OsId:0,TimeCreated:"0",Address:*Drone.GroundUdpAddress}
	//fmt.Println(Drone.DroneName, "INIT: masterFullName=", Drone.GroundFullName)
	//mastersEntry := tbMessages.TBmgr{Name:Drone.GroundFullName,Up:true,LastChangeTime:"0",MsgsSent:0,LastSentAt:"0",MsgsRcvd:0,LastRcvdAt:"0"}
	//Drone.KnownDrones = append(Drone.KnownDrones, mastersEntry)
	// fmt.Println("Records of Known Satellites=", Drone.KnownDrones)
	// check the master is there:
	//theGround := locateOtherDrone(Drone.KnownDrones, Drone.GroundIPandPort)
	//if theGround != nil {
	//	fmt.Println("Ground at:", theGround.NodeName, "ADDRESS:", theGround.NodeIP, "Port:", theGround.NodePort,
	//		"MSGSRCVD:", theGround.MsgsRcvd)
	//} else {
	//	fmt.Println("GROUND Station Not Detected YET at ", Drone.GroundIPandPort)
	//}

	//return true
}

//====================================================================================
//  Format and send DISCOVERY msg
//====================================================================================
func sendBroadcastDiscoveryPacket() {
	if Drone.StepMode == 0 { // we stepped few times and then paused
		return
	}
	// ... done in updateConnectivity ...MoveNode(Me)
	Me.NodeMsgLastSentAt = float64(TBtimestampNano()) // time.Now() //TBtimestampNano()
	//strconv.FormatInt(TBtimestampNano(), 10)

	dstPort, _ := strconv.Atoi(Drone.BroadcastTxPort)
	// TODO add heightFromEarthCenter as configuration ?? what if it wobbles?
	latitude, longitude := ConvertXYZtoLatLong(Me.MyX, Me.MyY, Me.MyZ,
		Drone.GpsGroundHeightFromEartCenter+Drone.GpsOrbitAltitudeFromGround)
	// timeNow := time.Now()
	msgHdr := MessageHeader{
		MsgCode:     "DISCOVERY",
		Ttl:         3,
		TimeSent:    float64(TBtimestampNano()), // timeNow, //Me.MsgLastSentAt, // time.Now().String()
		LocationX:   Me.MyX,
		LocationY:   Me.MyY,
		LocationZ:   Me.MyZ,
		Longitude:   longitude,
		Latitude:    latitude,
		RadioRange:  int(Me.NodeRadioRange),
		SrcSeq:      Me.NodeMsgSeq,
		SrcRole:     Me.NodeRole,
		SrcMAC:      Me.NodeMac,
		SrcName:     Me.NodeName,
		SrcId:       Me.NodeId, // node ids are 1 based
		SrcIP:       Me.NodeIP,
		SrcPort:     Me.NodePort,
		DstName:     "BROADCAST",
		DstId:       0,
		DstIP:       Drone.BroadcastTxIP,
		DstPort:     dstPort,
		GroundRange: int(Me.NodeDistanceToGround),
		Hash:        0,
	}
	g := "" + strconv.FormatUint(Me.GatewayList.M_Mask, 10)
	s := "" + strconv.FormatUint(Me.SubscriberList.M_Mask, 10)
	b := "" + strconv.FormatUint(Me.BaseStationList.M_Mask, 10)

	discBody := DiscoveryMsgBody{

		Gateways:     g, // Me.GatewayList.M_Mask,  // TODO might be not used ??
		Subscribers:  s, // Me.SubscriberList.M_Mask,
		BaseStations: b, // Me.BaseStationList.M_Mask,
		NodeActive:   Me.NodeActive,
		MsgsSent:     Me.NodeMsgsSent,
		MsgsRcvd:     Me.NodeMsgsRcvd,

		Velocity:  Me.Velocity,  // float64
		VelocityX: Me.VelocityX, // float64
		VelocityY: Me.VelocityY, // float64
		VelocityZ: Me.VelocityZ, // float64
		BwRequest: 0,
	}
	myMsg := MsgCodeDiscovery{
		MsgHeader:    msgHdr,
		MsgDiscovery: discBody,
	}
	Me.NodeMsgSeq++
	msg, _ := TBmarshal(myMsg)

	// fmt.Println(" BROADCAST DISCOVERY", Drone.BroadcastTxStruct)
	ControlPlaneBroadcastSend(msg, Drone.BroadcastTxStruct)

}

//=================================================================================
//=================================================================================
func forwardUnicastDiscoveryPacket(msgHeader *MessageHeader,
	discoveryMsg *DiscoveryMsgBody, groundDistance int) {
	// groundDistance - from this sender to ground, not from origin
	// use the header fields from original msg, except distance
	port, _ := strconv.Atoi(Drone.UnicastTxPort)
	// latitude,longitude := ConvertXYZtoLatLong(Me.MyX, Me.MyY, Me.MyZ, orbitHeightFromEartCenter)
	msgHdr := MessageHeader{
		MsgCode:     "DISCOVERY",
		Ttl:         1,
		TimeSent:    msgHeader.TimeSent,
		LocationX:   msgHeader.LocationX,
		LocationY:   msgHeader.LocationY,
		LocationZ:   msgHeader.LocationZ,
		Longitude:   msgHeader.Longitude,
		Latitude:    msgHeader.Latitude,
		RadioRange:  msgHeader.RadioRange,
		SrcSeq:      msgHeader.SrcSeq,
		SrcRole:     msgHeader.SrcRole,
		SrcMAC:      msgHeader.SrcMAC,
		SrcName:     msgHeader.SrcName,
		SrcId:       msgHeader.SrcId, // node ids are 1 based
		SrcIP:       msgHeader.SrcIP,
		SrcPort:     msgHeader.SrcPort,
		DstName:     "UNICAST",
		DstId:       0,
		DstIP:       Drone.GroundIP,
		DstPort:     port,
		GroundRange: groundDistance, // use my distance
		Hash:        0,
	}

	discBody := DiscoveryMsgBody{
		TimeCreated:    discoveryMsg.TimeCreated,
		LastChangeTime: discoveryMsg.LastChangeTime,
		NodeActive:     discoveryMsg.NodeActive,
		MsgsSent:       discoveryMsg.MsgsSent,
		MsgsRcvd:       discoveryMsg.MsgsRcvd,
		MsgLastSentAt:  discoveryMsg.MsgLastSentAt,
		MsgLastRcvdAt:  discoveryMsg.MsgLastRcvdAt,
		Gateways:       discoveryMsg.Gateways,     // Me.GatewayList.M_Mask,
		Subscribers:    discoveryMsg.Subscribers,  // Me.SubscriberList.M_Mask,
		BaseStations:   discoveryMsg.BaseStations, // Me.BaseStationList.M_Mask,

		Velocity:  discoveryMsg.Velocity,  // float64
		VelocityX: discoveryMsg.VelocityX, // float64
		VelocityY: discoveryMsg.VelocityY, // float64
		VelocityZ: discoveryMsg.VelocityZ, // float64
		BwRequest: 0,
	}
	myMsg := MsgCodeDiscovery{
		MsgHeader:    msgHdr,
		MsgDiscovery: discBody,
	}
	Me.NodeMsgSeq++
	msg, _ := TBmarshal(myMsg)

	fmt.Println("FORWARD(", Me.NodeDistanceToGround, ") UNICAST DISCOVERY for ",
		msgHeader.SrcId, " to GROUND at ", Drone.GroundIP)
	ControlPlaneUnicastSend(msg, Drone.GroundIP+":"+Drone.UnicastTxPort)
}

//=================================================================================
//=================================================================================
func sendUnicastStatusReplyPacket(msgHeader *MessageHeader) {
	fmt.Println("...... STATUS REPLY: srcIP=", msgHeader.SrcIP, " SrcMAC=", msgHeader.SrcMAC,
		" DstID=", msgHeader.DstId, " SrcID=", msgHeader.SrcId)

	// port, _ :=strconv.Atoi(Drone.UnicastTxPort)
	// latitude,longitude := ConvertXYZtoLatLong(Me.MyX, Me.MyY, Me.MyZ, orbitHeightFromEartCenter)
	x, y, z := ConvertLatLongToXYZ(Me.NodeLatitude, Me.NodeLongitude, Me.NodeDistanceToGround)
	msgHdr := MessageHeader{
		MsgCode:     "STATUS_REPLY",
		Ttl:         1,
		TimeSent:    float64(TBtimestampNano()),
		LocationX:   x,
		LocationY:   y,
		LocationZ:   z,
		Longitude:   Me.NodeLongitude,
		Latitude:    Me.NodeLatitude,
		RadioRange:  Me.NodeRadioRange,
		SrcSeq:      Me.NodeMsgSeq,
		SrcRole:     Me.NodeRole,
		IamAlone:    Me.NodeIamAlone,
		SrcMAC:      Me.NodeMac,
		SrcName:     Me.NodeName,
		SrcId:       Me.NodeId, // node ids are 1 based
		SrcIP:       Me.NodeIP,
		SrcPort:     Me.NodePort,
		DstName:     "UNICAST",
		DstId:       msgHeader.SrcId,
		DstIP:       msgHeader.SrcIP,
		DstPort:     msgHeader.SrcPort,
		GroundRange: int(Me.NodeDistanceToGround),
		Hash:        0,
	}

	// Win C++ does not support uint64 fields
	g := strconv.FormatUint(Me.GatewayList.M_Mask, 10)
	s := strconv.FormatUint(Me.SubscriberList.M_Mask, 10)
	b := strconv.FormatUint(Me.BaseStationList.M_Mask, 10)

	statusReplyBody := StatusReplyMsgBody{
		TimeCreated:    Me.NodeTimeCreated,
		LastChangeTime: Me.NodeLastChangeTime,
		NodeActive:     Me.NodeActive,
		MsgsSent:       Me.NodeMsgsSent,
		MsgsRcvd:       Me.NodeMsgsRcvd,
		MsgLastSentAt:  Me.NodeMsgLastSentAt,
		MsgLastRcvdAt:  Me.NodeMsgLastRcvdRemote,
		Gateways:       g, // Me.GatewayList.M_Mask,
		Subscribers:    s, // Me.SubscriberList.M_Mask,
		BaseStations:   b, // Me.BaseStationList.M_Mask,

		Velocity:  Me.Velocity,  // float64
		VelocityX: Me.VelocityX, // float64
		VelocityY: Me.VelocityY, // float64
		VelocityZ: Me.VelocityZ, // float64
		BwRequest: 0,
	}

	myMsg := MsgCodeStatusReply{
		MsgHeader:      msgHdr,
		MsgStatusReply: statusReplyBody,
	}
	Me.NodeMsgSeq++
	msg, _ := TBmarshal(myMsg)

	fmt.Println("STATUS_REPLY(", int(Me.NodeDistanceToGround), ") UNICAST to GROUND at ", Drone.GroundIP)
	ControlPlaneUnicastSend(msg, Drone.GroundIP+":"+Drone.UnicastTxPort)
}

//=================================================================================
//=================================================================================
func sendUnicastDiscoveryPacket(unicastIP string, distance int) {
	Me.NodeMsgLastSentAt = float64(TBtimestampNano()) //time.Now() //strconv.FormatInt(TBtimestampNano(), 10)

	port, _ := strconv.Atoi(Drone.UnicastTxPort)
	// latitude,longitude := ConvertXYZtoLatLong(Me.MyX, Me.MyY, Me.MyZ, orbitHeightFromEartCenter)
	msgHdr := MessageHeader{
		MsgCode:     "DISCOVERY",
		Ttl:         3,
		TimeSent:    float64(TBtimestampNano()), // timeNow, //Me.MsgLastSentAt, // time.Now().String()
		LocationX:   Me.MyX,
		LocationY:   Me.MyY,
		LocationZ:   Me.MyZ,
		Longitude:   Me.NodeLongitude,
		Latitude:    Me.NodeLatitude,
		RadioRange:  int(Me.NodeRadioRange),
		SrcSeq:      Me.NodeMsgSeq,
		SrcRole:     Me.NodeRole,
		SrcMAC:      Me.NodeMac,
		SrcName:     Me.NodeName,
		SrcId:       Me.NodeId, // node ids are 1 based
		SrcIP:       Me.NodeIP,
		SrcPort:     Me.NodePort,
		DstName:     "UNICAST",
		DstId:       0,
		DstIP:       Drone.BroadcastTxIP,
		DstPort:     port,
		GroundRange: distance,
		Hash:        0,
	}
	g := strconv.FormatUint(Me.GatewayList.M_Mask, 10)
	s := strconv.FormatUint(Me.SubscriberList.M_Mask, 10)
	b := strconv.FormatUint(Me.BaseStationList.M_Mask, 10)

	discBody := DiscoveryMsgBody{
		NodeActive:   Me.NodeActive,
		MsgsSent:     Me.NodeMsgsSent,
		MsgsRcvd:     Me.NodeMsgsRcvd,
		Gateways:     g, // Me.GatewayList.M_Mask,
		Subscribers:  s, // Me.SubscriberList.M_Mask,
		BaseStations: b, // Me.BaseStationList.M_Mask,

		Velocity:  Me.Velocity,  // float64
		VelocityX: Me.VelocityX, // float64
		VelocityY: Me.VelocityY, // float64
		VelocityZ: Me.VelocityZ, // float64
		BwRequest: 0,
	}
	myMsg := MsgCodeDiscovery{
		MsgHeader:    msgHdr,
		MsgDiscovery: discBody,
	}
	Me.NodeMsgSeq++
	msg, _ := TBmarshal(myMsg)

	//fmt.Println( "SEND UNICAST DISCOVERY to ", unicastIP)
	ControlPlaneUnicastSend(msg, unicastIP)
}

//====================================================================================
// Check if any err, and exit
//====================================================================================
func checkErrorNode(err error) {
	if err != nil {
		_, _ = fmt.Fprintf(os.Stderr, "Fatal error %s", err.Error())
		os.Exit(1)
	}
}

//============================================================================
// Locate specific drone in the slice of all learned drones by NAME
// Return nil if row not found
//============================================================================
func locateOtherDrone(slice []NodeInfo, otherDrone string) *NodeInfo {
	for index := range slice {
		if slice[index].NodeName == otherDrone {
			return &slice[index]
		}
	}
	return nil
}
