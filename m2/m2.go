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
// SYNAPSE VERSION
//================================================================================
/* UDPDaytimeClient
make function allocates and initializes an object of type slice, map, or chan only.
Like new, the first argument is a type. But, it can also take a second argument, the size.
Unlike new, make’s return type is the same as the type of its argument, not a pointer to it.
And the allocated value is initialized (not set to zero value like in new).
The reason is that slice, map and chan are data structures.
They need to be initialized, otherwise they won't be usable.
This is the reason new() and make() need to be different.
p := new(chan int)   // p has type: *chan int
c := make(chan int)  // c has type: chan int
p *[]int = new([]int) // *p = nil, which makes p useless
v []int = make([]int, 100) // creates v structure that has pointer to an array,
            length field, and capacity field. So, v is immediately usable
*/
package main

import (
	"fmt"
        "github.com/igismo/synapse/commonTB"
	"github.com/spf13/viper"
	"net"
	"os"
	"os/exec"
	"runtime"
	"strconv"
	"strings"
	"time"
)

//const DEBUG_ROLE = 0
//const DEBUG_DISCOVERY = 0

var M2 common.M2Info           // all info about the Node
var Log = common.LogInstance{} // for log file storage

const StateDown = "DOWN"
const StateConnecting = "CONNECTING"
const StateConnected = "CONNECTED"

// REMOTE_CMD Command from local terminal, or rcvd from ground controller
// const LOCAL_CMD = "LOCAL"
const REMOTE_CMD = "REMOTE"

// InitM2Configuration InitDroneConfiguration ======================================
// READ ARGUMENTS IF ANY
//====================================================================================
func InitM2Configuration() {

	M2.M2TerminalName = "termM2A" // will be overwritten by config
	M2.M2TerminalId = 2           // will be overwritten by config
	M2.M2TerminalIP = ""
	M2.M2TerminalConnectionTimer = common.DRONE_KEEPALIVE_TIMER // 5 sec

	M2.M2Channels.CmdChannel = nil            // so that all local threads can talk back
	M2.M2Channels.UnicastRcvCtrlChannel = nil // to send control msgs to Recv Thread
	M2.M2Channels.BroadcastRcvCtrlChannel = nil
	M2.M2Channels.MulticastRcvCtrlChannel = nil

	// M2.TerminalConnection = nil

	M2.M2TerminalReceiveCount = 0
	M2.M2TerminalSendCount = 0
	Log.DebugLog = true
	Log.WarningLog = true
	Log.ErrorLog = true

	M2.M2Channels.UnicastRcvCtrlChannel = make(chan []byte) //
	M2.M2Channels.BroadcastRcvCtrlChannel = make(chan []byte)
	M2.M2Channels.MulticastRcvCtrlChannel = make(chan []byte)
	M2.M2Channels.CmdChannel = make(chan []string) // receive command line cmnds

	M2.M2Connectivity.BroadcastRxAddress = ":48999"
	M2.M2Connectivity.BroadcastRxPort = "48999"
	M2.M2Connectivity.BroadcastRxIP = ""
	M2.M2Connectivity.BroadcastTxPort = "48888"
	M2.M2Connectivity.BroadcastConnection = nil
	M2.M2Connectivity.BroadcastTxStruct = new(net.UDPAddr)

	M2.M2Connectivity.UnicastRxAddress = ":48888"
	M2.M2Connectivity.UnicastRxPort = "48888"
	M2.M2Connectivity.UnicastRxIP = ""
	M2.M2Connectivity.UnicastTxPort = "48888"
	M2.M2Connectivity.UnicastConnection = nil
	M2.M2Connectivity.UnicastRxStruct = nil
	M2.M2Connectivity.UnicastTxStruct = new(net.UDPAddr)

	//M2.GroundIsKnown = false
	//M2.GroundUdpPort = 0
	//M2.GroundIP = ""
	M2.M2TerminalPort = "8888"
	//M2.GroundIPandPort = "" //M2.GroundIP + ":" + M2.GroundUdpPort
	M2.M2TerminalUdpAddrStructure = new(net.UDPAddr)
	//M2.GroundUdpAddrSTR = new(net.UDPAddr)
	M2.M2TerminalTimeCreated = time.Now() // strconv.FormatInt(common.TBtimestampNano(), 10)
}

// InitFromConfigFile ================================================================
// InitFromConfigFile() - Set configuration from config file
//====================================================================================
func InitFromConfigFile() {
	var fileName string
	argNum := len(os.Args) // Number of arguments supplied, including the command
	fmt.Println("Number of Arguments = ", argNum)
	if argNum > 2 && os.Args[1] != "" && os.Args[1] != "0" {
		M2.M2TerminalName = os.Args[1]
		M2.M2TerminalId, _ = strconv.Atoi(os.Args[2])
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
	err := viper.Unmarshal(&M2) //&droneStruct)
	if err != nil {
		fmt.Printf("Unable to decode into struct, %v", err)
	}

	M2.M2Connectivity.UnicastRxIP = M2.M2UnicastRxIP
	M2.M2Connectivity.UnicastRxPort = M2.M2UnicastRxPort
	// M2.M2Connectivity.UnicastRxAddress		=
	M2.M2Connectivity.UnicastTxPort = M2.M2UnicastTxPort
	M2.M2Connectivity.BroadcastTxIP = M2.M2BroadcastTxIP
	M2.M2Connectivity.BroadcastTxPort = M2.M2BroadcastTxPort
	// M2.M2Connectivity.BroadcastTxAddress	=
	M2.M2Connectivity.BroadcastRxIP = M2.M2BroadcastRxIP
	M2.M2Connectivity.BroadcastRxPort = M2.M2BroadcastRxPort
	// M2.M2Connectivity.BroadcastRxAddress	=
	M2.M3TerminalPort = M2.M2UnicastRxPort // Unless M3 tells are otherwise

	M2.M2Connectivity.BroadcastTxAddress =
		M2.M2Connectivity.BroadcastTxIP + ":" + M2.M2Connectivity.BroadcastTxPort
	M2.M2Connectivity.BroadcastRxAddress =
		M2.M2Connectivity.BroadcastRxIP + ":" + M2.M2Connectivity.BroadcastRxPort
	M2.M2Connectivity.UnicastRxAddress =
		M2.M2Connectivity.UnicastRxIP + ":" + M2.M2Connectivity.UnicastRxPort

	fmt.Println("Reading variables from ... config", fileName)
	fmt.Println("*************************************************************")
	fmt.Println("M2TerminalId             = ", M2.M2TerminalId)
	fmt.Println("M2TerminalName           = ", M2.M2TerminalName)
	fmt.Println("M2TerminalIP             = ", M2.M2TerminalIP)
	fmt.Println("M2TerminalLogPath        = ", M2.M2TerminalLogPath)
	fmt.Println("TerminalConnectionTimer  = ", M2.M2TerminalConnectionTimer)
	fmt.Println("M2TerminalPort           = ", M2.M2TerminalPort)

	fmt.Println("BroadcastTxIP            = ", M2.M2Connectivity.BroadcastTxIP)
	fmt.Println("BroadcastTxPort          = ", M2.M2Connectivity.BroadcastTxPort)
	fmt.Println("BroadcastRxIP            = ", M2.M2Connectivity.BroadcastRxIP)
	fmt.Println("BroadcastRxPort          = ", M2.M2Connectivity.BroadcastRxPort)
	fmt.Println("BroadcastRxAddress       = ", M2.M2Connectivity.BroadcastRxAddress)
	fmt.Println("BroadcastTxAddress       = ", M2.M2Connectivity.BroadcastTxAddress)

	fmt.Println("UnicastRxIP              = ", M2.M2Connectivity.UnicastRxIP)
	fmt.Println("UnicastRxPort            = ", M2.M2Connectivity.UnicastRxPort)
	fmt.Println("UnicastRxAddress         = ", M2.M2Connectivity.UnicastRxAddress)

	fmt.Println("M3TerminalIP             = ", M2.M3TerminalIP)
	fmt.Println("M3TerminalPort     	  = ", M2.M3TerminalPort)
}

// InitFromCommandLine ====================================================================================
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
		fmt.Println("M2 NAME = ", os.Args[1])
		M2.M2TerminalName = os.Args[1]
		M2.M2TerminalId, _ = strconv.Atoi(os.Args[2])
	}
	if argNum > 3 && os.Args[3] != "" && os.Args[3] != "0" {
		fmt.Println("Satellite IP   = ", os.Args[3])
		M2.M2TerminalIP = os.Args[3]
		// TODO: Set eth0 IP address to M2.M2IpAddress !!!
		var ifCmd = exec.Command("sudo", "ifconfig", "eth0",
			M2.M2TerminalIP, "up", "", "")
		output, err := ifCmd.Output()
		fmt.Println("SET MY IP=", "sudo", "ifconfig", "eth0", M2.M2TerminalIP, "up",
			" -OUTPUT:", string(output), " ERR:", err)
	}
	if argNum > 4 && os.Args[4] != "" && os.Args[4] != "0" {
		M2.M2TerminalPort = os.Args[4] // strconv.ParseInt(os.Args[3], 10, 64)
	}

	if argNum > 5 && os.Args[5] != "" && os.Args[5] != "0" && argNum > 6 && os.Args[6] != "" && os.Args[6] != "0" {
		//	M2.GroundIPandPort = os.Args[5] + ":" + os.Args[6]
	}
}

// SetFinalM2Info ===============================================================
//  Initialize my own info in the M2 structure
//===============================================================================
func SetFinalM2Info() {
	fmt.Println("SetFinalM2Info ")
	if M2.M2TerminalIP == "" {
		M2.M2TerminalIP, M2.M2TerminalMac = common.GetLocalIp() // get IP and MAC
		fmt.Println("M2 Local IP=", M2.M2TerminalIP, " MAC=", M2.M2TerminalMac)
	}

	M2.M2TerminalLastChangeTime = float64(common.TBtimestampNano()) // timeNow().String()
	changeState(StateDown)
}

// InitM2Connectivity ================================================================
//  Initialize IP and UDP addressing
//====================================================================================
func InitM2Connectivity() {
	M2.M2TerminalIPandPort = M2.M2TerminalIP + ":" + M2.M2TerminalPort
	var err3 error
	M2.M2TerminalUdpAddrStructure, err3 = net.ResolveUDPAddr("udp", M2.M2TerminalIPandPort)
	if err3 != nil {
		fmt.Println("ERROR ResolveUDPAddr: ", err3)
	} else {
		M2.M2TerminalFullName = common.NameId{Name: M2.M2TerminalName,
			Address: *M2.M2TerminalUdpAddrStructure}
	}
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

//====================================================================================
//
//====================================================================================
func periodicFunc(tick time.Time) {
	// TODO - figure out reasonable timer period to process these two
	// Note that we may have not receive messages from some terminals for some time
	// So maybe wait until the end of processing period and figure out who we
	// received discovery msgs from and based on that figure out the connectivity

	fmt.Println("TICK: UPDATE CONNECTIVITY -----  ", tick)

	currTimeMilliSec := common.TBtimestampMilli()
	elapsedTimeSinceLastHelloSent := currTimeMilliSec - M2.M2TerminalLastHelloSendTime
	elapsedTimeSinceLastHelloReceived := currTimeMilliSec - M2.M2TerminalLastHelloReceiveTime

	if elapsedTimeSinceLastHelloReceived > M2.M2TerminalHelloTimerLength {
		// No word from M3 fro a long time, resend our hello
		sendBroadcastHelloPacket()
		M2.M2TerminalLastHelloSendTime = common.TBtimestampMilli()
		changeState(StateConnecting)
	} else if elapsedTimeSinceLastHelloSent > M2.M2TerminalHelloTimerLength {
		// time to resend our hello, in either connecting or connected state
		sendUnicastHelloPacket(M2.M3TerminalIP, M2.M3TerminalPort)
		M2.M2TerminalLastHelloSendTime = common.TBtimestampMilli()
	}
}

//===============================================================================
// M2 == Satellite, really M2
//===============================================================================
func main() {
	myOS := runtime.GOOS
	fmt.Println(M2.M2TerminalName, " ========= START on ", myOS, " at ", time.Now(), "==========")
	//===============================================================================
	// UPDATE RELEVANT VARIABLES and structures in proper order
	//===============================================================================
	InitM2Configuration()
	// Then try to read config file
	InitFromConfigFile()
	// Finally overwrite if any command arguments given
	InitFromCommandLine()

	SetFinalM2Info()
	// Create LOG file
	common.CreateLog(&Log, M2.M2TerminalName, M2.M2TerminalLogPath)
	Log.Warning(&Log, "Warning test:this will be printed anyway")

	InitM2Connectivity()

	// Make this work one of these days ...
	var err error
	checkErrorNode(err)

	common.ControlPlaneInit(&M2.M2Connectivity, M2.M2Channels)

	// START SEND AND RECEIVE THREADS:
	err2 := common.ControlPlaneRecvThread(&M2.M2Connectivity, M2.M2Channels)
	if err2 != nil {
		fmt.Println(M2.M2TerminalName, "INIT: Error creating Broadcast/Unicast RX thread")
		panic(err2)
	}

	//================================================================================
	// START TIMER : Call periodicFunc on every timerTick
	//================================================================================
	tick := 3000 * time.Millisecond
	fmt.Println(M2.M2TerminalName, "MAIN: Starting a new Timer Ticker at ", tick, " msec")
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
		fmt.Println("------------------------------------------------------------------")
		select {
		case UnicastMsg := <-M2.M2Channels.UnicastRcvCtrlChannel:
			fmt.Println(M2.M2TerminalName, "====> MAIN: Unicast MSG in state", M2.M2TerminalState, "MSG=", string(UnicastMsg))
			// these include text messages from the ground/controller
			ControlPlaneMessages(UnicastMsg)
		case BroadcastMsg := <-M2.M2Channels.BroadcastRcvCtrlChannel:
			fmt.Println(M2.M2TerminalName, "====> MAIN: Broadcast MSG in state", M2.M2TerminalState, "MSG=", string(BroadcastMsg))
			// these include text messages from the ground/controller
			ControlPlaneMessages(BroadcastMsg)
		case MulticastMsg := <-M2.M2Channels.MulticastRcvCtrlChannel:
			fmt.Println(M2.M2TerminalName, "====> MAIN: Multicast MSG in state", M2.M2TerminalState, "MSG=", string(MulticastMsg))
			// these include text messages from the ground/controller
			ControlPlaneMessages(MulticastMsg)
		case CmdText, ok := <-ConsoleInput: // These are messsages from local M2 console
			fmt.Println(M2.M2TerminalName, "====> MAIN: Console Input: ", CmdText)
			if !ok {
				fmt.Println("ERROR Reading input from stdin:", CmdText)
				break
			} else {
				// INPUT, cmd and parameters are separated in CmdText[] string,
				// PROCESS / PARSE
				//fmt.Println("Console: SEND Unicast Packet, stdin:", CmdText)
				//sendUnicastHelloPacket(ip, port)
				// SendTextMsg(stdin)
				// fmt.Println("Console input sent to ground");
				switch CmdText[0] { // switch on console command
				case "status":
					fmt.Println("STATUS REPLY: Name=", M2.M2TerminalName, " State=", M2.M2TerminalState)
				}
			}
			//default:
			//	fmt.Println("done and exit select")
		} // EndOfSelect
	} // EndOfFOR

	// common.ControlPlaneCloseConnections(M2.Connectivity)
	// os.Exit(0)
}

// ControlPlaneMessages ====================================================================================
// ControlPlaneMessages() - handle Control Plane messages
//====================================================================================
func ControlPlaneMessages(message []byte) {
	msg := new(common.Msg)
	err1 := common.TBunmarshal(message, &msg)
	if err1 != nil {
		println("Error unmarshalling message: ", msg.MsgHeader.MsgCode)
		return
	}
	msgHeader := &msg.MsgHeader
	sender := msgHeader.SrcId
	//msglen := len(message)
	//fmt.Println("ControlPlaneMessages: Msg=", msg) //.MsgCode, " msglen=", msglen)

	// Was this msg originated by us ?
	if strings.Contains(msgHeader.SrcIP, M2.M2TerminalIP) && sender == M2.M2TerminalId {
		// println("My own message: MsgCode=", msgHeader.MsgCode, " M2.NodeId=", M2.NodeId)
		return
	}
	//============================================================================
	// Is the other side within the RF range ?
	// we need to do this for ethernet connectivity as we receive everything
	//============================================================================
	// First check that the senders id is in valid range
	if sender == M2.M2TerminalId || sender < 1 || sender > 5 {
		println("Sender id WRONG: ", sender, " MsgCode=", msgHeader.MsgCode)
		return
	}
	//node 	:= &M2.NodeList[sender -1]
	fmt.Println("CHECK MESSAGE CODE ", msgHeader.MsgCode)
	switch msgHeader.MsgCode {
	case common.MSG_TYPE_DISCOVERY: // from another M2
		var discoveryMsg = new(common.MsgCodeDiscovery)
		err := common.TBunmarshal(message, discoveryMsg) //message
		if err != nil {
			println("ControlPlaneMessages: ERR=", err)
			return
		}
		ControlPlaneProcessDiscoveryMessage(msgHeader, &discoveryMsg.MsgDiscovery)
		break
	case common.MSG_TYPE_GROUND_INFO: // info from ground
		// TODO: will require some rethinking how to handle
		// TODO: may need to rebroadcast for nodes that aare out of range
		// Note that in order to cure the situation where a node might have been out of reach
		// at the time the STEP message was sent, GROUND will insert the latest value for
		//the StepMode in all GROUNDINFO messages .... but we need to process those ...
		handleGroundInfoMsg(msgHeader)
		break
	case common.MSG_TYPE_STATUS_REQ: // command from ground
		handleGroundStatusRequest(msgHeader)
		break
	case "UPDATE":
		break
	default:
		fmt.Println("ControlPlaneMessages:  UNKNOWN Message")
		break
	}
}

//====================================================================================
// ControlPlaneMessage STATUS REQ
//====================================================================================
func handleGroundStatusRequest(msgHeader *common.MessageHeader) {
	fmt.Println("...... STATUS REQUEST: srcIP=", msgHeader.SrcIP, " SrcMAC=", msgHeader.SrcMAC,
		" DstID=", msgHeader.DstId, " SrcID=", msgHeader.SrcId)

	// REPLY
	sendUnicastStatusReplyPacket(msgHeader)
}

//====================================================================================
// ControlPlaneMessage   GROUNDINFO
//====================================================================================
func handleGroundInfoMsg(msgHeader *common.MessageHeader) {
	/*
		var err error
		// TODO  ... add to msg the playing field size .... hmmm ?? relation to random etc
		M2.GroundFullName.Name = msgHeader.SrcName //.DstName
		M2.GroundIP = msgHeader.SrcIP
		M2.GroundIPandPort = string(msgHeader.SrcIP) + ":" + msgHeader.SrcPort
		myPort, _ := strconv.Atoi(msgHeader.SrcPort)
		M2.GroundUdpPort = myPort
		M2.GroundIsKnown = true //msg.GroundUp

		// M2.GroundUdpAddrSTR.IP = msg.TxIP;
		//fmt.Println(TERMCOLOR, "... GROUNDINFO: Name=", M2.GroundFullName.Name,
		//	"  IP:Port=", M2.GroundIPandPort, " StepMode=", msgHeader.StepMode)
		// fmt.Println("Port=", msg.TxPort, " txIP=", msg.TxIP, " " +
		//	"groundIP=",M2.GroundIP)

		M2.GroundUdpAddrSTR, err = net.ResolveUDPAddr("udp", M2.GroundIPandPort)
		//M2.GroundUdpAddrSTR.Port = msg.TxPort
		//M2.GroundUdpAddrSTR.IP   = net.IP(msg.TxIP)
		//M2.GroundUdpAddrSTR.IP 			 = net.IP((M2.GroundIP))
		//M2.GroundUdpAddrSTR.Port, _ 	 = strconv.Atoi(M2.GroundUdpPort)
		//M2.GroundUdpAddrSTR.IP	 = M2.GroundIP // net.IP(M2.GroundIP)

		//fmt.Println("1 M2.GroundUdpAddrSTR=", M2.GroundUdpAddrSTR)
		myPort, _ = strconv.Atoi(msgHeader.DstPort)
		M2.GroundUdpAddrSTR = &net.UDPAddr{IP: net.ParseIP(msgHeader.DstIP), Port: myPort}
		M2.GroundFullName = common.NameId{Name: msgHeader.DstName, Address: *M2.GroundUdpAddrSTR}
		//fmt.Println("2 M2.GroundUdpAddrSTR=", M2.GroundUdpAddrSTR)

		if err != nil {
			fmt.Println("ERROR in net.ResolveUDPAddr = ", err)
			fmt.Println("ERROR locating master, will retry")
			return
		} else {
			// fmt.Println("GROUND INFO: Name=", M2.GroundFullName.Name, "  IP:Port=", M2.GroundIPandPort)
		}
	*/
}

// ControlPlaneProcessDiscoveryMessage ===============================================
// Handle DISCOVERY messages in all states
//====================================================================================
func ControlPlaneProcessDiscoveryMessage(msgHeader *common.MessageHeader,

	// TODO: handle unicast and broadcast separatelly ??
	discoveryMsg *common.DiscoveryMsgBody) {
	//fmt.Println("Discovery MSG in state ", M2.M2State)
	switch M2.M2TerminalState {
	case StateDown:
		stateConnectedHelloMessage(msgHeader, discoveryMsg)
		break
	case StateConnecting:
		stateConnectedHelloMessage(msgHeader, discoveryMsg)
		break
	case StateConnected:
		stateConnectedHelloMessage(msgHeader, discoveryMsg)
		break
	default:
	}
}

//==========================================================================
// Me=0, M1=1, M2=2..5
//===========================================================================
func stateConnectedHelloMessage(msgHeader *common.MessageHeader,
	discoveryMsg *common.DiscoveryMsgBody) {
	sender := msgHeader.SrcId
	// TODO ... make sure we only handle configured M1 and M2s
	if sender < 1 || sender > 5 {
		// fmt.Println("DISCARD MSG: invalid senderId=", sender)
		return
	}

	M2.M2TerminalMsgLastSentAt = discoveryMsg.MsgLastSentAt
	M2.M2TerminalLastHelloReceiveTime = common.TBtimestampNano() // time.Now()
	/*
		if M2.GroundIsKnown {
			// did this DISCOVERY reach the ground as well

				if node.NodeDistanceToGround > M2.GroundRadioRange {
					fmt.Println("=== NEED TO FORWARD, OTHER NODE ", int(node.NodeDistanceToGround),
						" AWAY FROM GROUND, Ground at ", M2.GroundRadioRange)
					theNode, distance := FindShortestConnectionToGround()
					fmt.Println("====== Me=", M2.NodeId, " MY DISTANCE=",M2.NodeDistanceToGround,
						" HIS DISTANCE=", node.NodeDistanceToGround," SHORTEST=", distance,
						" Forwarder=", theNode.NodeId)
					if Me == theNode && M2.NodeDistanceToGround <= M2.GroundRadioRange {
						fmt.Println("====== I AM THE FORWARDER ===========================")
						// forward the DISCOVERY as UNICAST to GROUND
						forwardUnicastDiscoveryPacket(msgHeader, discoveryMsg, int(distance))
					}
				} else {
					//fmt.Println("==NODE ", node.NodeId, " CAN REACH GROUND AT",node.DistanceToGround ,
					//	" Ground at ", M2.GroundRadioRange)
				}
		}
	*/
}

/*
func FindShortestConnectionToGround() (*common.NodeInfo, float64) {
	var theNode *common.NodeInfo = nil
	var distance float64 = 1000000000
	for j:=0; j<64; j++ {
		if M2.NodeList[j].NodeActive == true {
			if distance > M2.NodeList[j].NodeDistanceToGround {
				distance = M2.NodeList[j].NodeDistanceToGround
				theNode = &M2.NodeList[j]
			}
		}
	}
	return theNode, distance
}

//====================================================================================
// Handle messages received in the CONNECTED state
//====================================================================================
func RemoteCommandMessages(msg *common.Msg) {
	var cmds []common.LinuxCommand
	// _ = TBunmarshal(msg.MsgBody, &cmds)

	for cmdIndex := range cmds {
		var cmd common.LinuxCommand
		cmd = cmds[cmdIndex]
		err := RunLinuxCommand(REMOTE_CMD, cmd.Cmd, cmd.Par1, cmd.Par2, cmd.Par3, cmd.Par4, cmd.Par5, cmd.Par6)
		if err != nil {
			fmt.Printf("%T\n", err)
		}
	}
}
*/
//=======================================================================
//
//=======================================================================
func LocalCommandMessages(cmdText string) {
	//var cmd []string
	//cmd = strings.Split(cmdText, " ")

	switch cmdText {
	case "enable":
		M2.M2TerminalActive = true
	case "disable":
		M2.M2TerminalActive = false
	case "help":
		fmt.Println("MAIN ......... No Help yet")
	default:
	}
	//fmt.Println("RCVD CONSOLE INPUT =", cmdText, " M2 ACTIVE=", M2.M2TerminalActive)
	// TODO figure out the bellow line
	//err := RunLinuxCommand(LOCAL_CMD, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6])
	//if err != nil {fmt.Printf("%T\n", err)}

}

// RunLinuxCommand ========================================================
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
	fmt.Println(M2.M2TerminalName, "OldState=", M2.M2TerminalState, " NewState=", newState)
	M2.M2TerminalState = newState
}

//====================================================================================
//  Format and send DISCOVERY msg
//====================================================================================
func sendBroadcastHelloPacket() {

	dstPort := M2.M2Connectivity.BroadcastTxPort

	msgHdr := common.MessageHeader{
		MsgCode:  "DISCOVERY",
		Ttl:      3,
		TimeSent: float64(common.TBtimestampNano()), // timeNow, //M2.MsgLastSentAt, // time.Now().String()
		SrcSeq:   M2.M2TerminalNextMsgSeq,
		SrcMAC:   M2.M2TerminalMac,
		SrcName:  M2.M2TerminalName,
		SrcId:    M2.M2TerminalId, // node ids are 1 based
		SrcIP:    M2.M2TerminalIP,
		SrcPort:  M2.M2TerminalPort,
		DstName:  "BROADCAST",
		DstId:    0,
		DstIP:    M2.M2Connectivity.BroadcastTxIP,
		DstPort:  dstPort,
		Hash:     0,
	}

	discBody := common.DiscoveryMsgBody{
		NodeActive: M2.M2TerminalActive,
		MsgsSent:   M2.M2TerminalMsgsSent,
		MsgsRcvd:   M2.M2TerminalMsgsRcvd,
	}

	myMsg := common.MsgCodeDiscovery{
		MsgHeader:    msgHdr,
		MsgDiscovery: discBody,
	}

	M2.M2TerminalNextMsgSeq++
	msg, _ := common.TBmarshal(myMsg)

	fmt.Println("SEND BROADCAST DISCOVERY", M2.M2Connectivity.BroadcastTxStruct)
	if M2.M2Connectivity.BroadcastConnection != nil {
		common.ControlPlaneBroadcastSend(M2.M2Connectivity, msg, M2.M2Connectivity.BroadcastTxStruct)
	} else {
		// TODO complain
	}
}

//=================================================================================
//=================================================================================
func sendUnicastStatusReplyPacket(msgHeader *common.MessageHeader) {
	fmt.Println("...... STATUS REPLY: srcIP=", msgHeader.SrcIP, " SrcMAC=", msgHeader.SrcMAC,
		" DstID=", msgHeader.DstId, " SrcID=", msgHeader.SrcId)
	/*
		// port, _ :=strconv.Atoi(M2.UnicastTxPort)
		// latitude,longitude := ConvertXYZtoLatLong(M2.MyX, M2.MyY, M2.MyZ, orbitHeightFromEartCenter)
		msgHdr := common.MessageHeader{
			MsgCode:  "STATUS_REPLY",
			Ttl:      1,
			TimeSent: float64(common.TBtimestampNano()),
			SrcSeq:   M2.M2TerminalNextMsgSeq,
			SrcMAC:   M2.M2TerminalMac,
			SrcName:  M2.M2TerminalName,
			SrcId:    M2.M2TerminalId, // node ids are 1 based
			SrcIP:    M2.M2TerminalIP,
			SrcPort:  M2.M2TerminalPort,
			DstName:  "UNICAST",
			DstId:    msgHeader.SrcId,
			DstIP:    msgHeader.SrcIP,
			DstPort:  msgHeader.SrcPort,
			Hash:     0,
		}

		statusReplyBody := common.StatusReplyMsgBody{
			//TimeCreated:	M2.TerminalTimeCreated,
			LastChangeTime: M2.M2TerminalLastChangeTime,
			NodeActive:     M2.M2TerminalActive,
			MsgsSent:       M2.M2TerminalMsgsSent,
			MsgsRcvd:       M2.M2TerminalMsgsRcvd,
			//MsgLastSentAt:	 M2.TerminalMsgLastSentAt,
		}

		myMsg := common.MsgCodeStatusReply{
			MsgHeader:      msgHdr,
			MsgStatusReply: statusReplyBody,
		}
		M2.M2TerminalNextMsgSeq++
		msg, _ := common.TBmarshal(myMsg)

		common.ControlPlaneUnicastSend(M2.M2Connectivity, msg, M2.GroundIP+":"+M2.Connectivity.UnicastTxPort)
	*/
}

//=================================================================================
//=================================================================================
func sendUnicastHelloPacket(unicastIP, port string) {
	M2.M2TerminalMsgLastSentAt = float64(common.TBtimestampNano()) //time.Now()

	msgHdr := common.MessageHeader{
		MsgCode:  "DISCOVERY",
		Ttl:      3,
		TimeSent: float64(common.TBtimestampNano()), // timeNow, //M2.MsgLastSentAt, // time.Now().String()
		SrcSeq:   M2.M2TerminalNextMsgSeq,
		SrcMAC:   M2.M2TerminalMac,
		SrcName:  M2.M2TerminalName,
		SrcId:    M2.M2TerminalId, // node ids are 1 based
		SrcIP:    M2.M2TerminalIP,
		SrcPort:  M2.M2TerminalPort,
		DstName:  "UNICAST",
		DstId:    0,
		DstIP:    unicastIP, //M2.M2Connectivity.BroadcastTxIP,
		DstPort:  port,
		Hash:     0,
	}

	discBody := common.DiscoveryMsgBody{
		NodeActive: M2.M2TerminalActive,
		MsgsSent:   M2.M2TerminalMsgsSent,
		MsgsRcvd:   M2.M2TerminalMsgsRcvd,
	}
	myMsg := common.MsgCodeDiscovery{
		MsgHeader:    msgHdr,
		MsgDiscovery: discBody,
	}
	M2.M2TerminalNextMsgSeq++
	msg, _ := common.TBmarshal(myMsg)

	fmt.Println("SEND UNICAST DISCOVERY to ", unicastIP)
	common.ControlPlaneUnicastSend(M2.M2Connectivity, msg, M2.M3TerminalIP)
}
