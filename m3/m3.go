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
// SYNAPSE VERSION 2021/22
//================================================================================
package main

import (
	"fmt"
	"github.com/igismo/synapse/commonTB"
	"github.com/spf13/viper"
	"net"
	"os"
	"os/exec"
	//"reflect"
	"runtime"
	"strconv"
	"strings"
	"time"
)

//const DEBUG_ROLE = 0
//const DEBUG_DISCOVERY = 0

var M3 common.M3Info           // all info about the Node
var Log = common.LogInstance{} // for log file storage

const StateDown = "DOWN"
const StateConnecting = "CONNECTING"
const StateConnected = "CONNECTED"

// REMOTE_CMD Command from local terminal, or rcvd from ground controller
// const LOCAL_CMD = "LOCAL"
const REMOTE_CMD = "REMOTE"

// InitM3Configuration InitDroneConfiguration ======================================
// READ ARGUMENTS IF ANY
//====================================================================================
func InitM3Configuration() {

	M3.M3TerminalName = "Node01" // will be overwritten by config
	M3.M3TerminalId = 1          // will be overwritten by config
	M3.M3TerminalIP = ""
	M3.TerminalConnectionTimer = common.DRONE_KEEPALIVE_TIMER // 5 sec
	M3.TerminalDiscoveryTimeout = 5000                        // milli sec

	M3.Channels.CmdChannel = nil            // so that all local threads can talk back
	M3.Channels.UnicastRcvCtrlChannel = nil // to send control msgs to Recv Thread
	M3.Channels.BroadcastRcvCtrlChannel = nil
	M3.Channels.MulticastRcvCtrlChannel = nil

	// M3.TerminalConnection = nil

	M3.TerminalReceiveCount = 0
	M3.TerminalSendCount = 0
	Log.DebugLog = true
	Log.WarningLog = true
	Log.ErrorLog = true

	M3.Channels.UnicastRcvCtrlChannel = make(chan []byte) //
	M3.Channels.BroadcastRcvCtrlChannel = make(chan []byte)
	M3.Channels.MulticastRcvCtrlChannel = make(chan []byte)
	M3.Channels.CmdChannel = make(chan []string) // receive command line cmnds

	M3.Connectivity.BroadcastRxAddress = ":48999"
	M3.Connectivity.BroadcastRxPort = "48999"
	M3.Connectivity.BroadcastRxIP = ""
	M3.Connectivity.BroadcastTxPort = "48888"
	M3.Connectivity.BroadcastConnection = nil
	M3.Connectivity.BroadcastTxStruct = new(net.UDPAddr)

	M3.Connectivity.UnicastRxAddress = ":48888"
	M3.Connectivity.UnicastRxPort = "48888"
	M3.Connectivity.UnicastRxIP = ""
	M3.Connectivity.UnicastTxPort = "48888"
	M3.Connectivity.UnicastConnection = nil
	M3.Connectivity.UnicastRxStruct = nil
	M3.Connectivity.UnicastTxStruct = new(net.UDPAddr)

	M3.GroundIsKnown = false
	M3.GroundUdpPort = 0
	M3.GroundIP = ""
	M3.M3TerminalPort = "8888"
	M3.GroundIPandPort = "" //M3.GroundIP + ":" + M3.GroundUdpPort
	M3.TerminalUdpAddrStructure = new(net.UDPAddr)
	M3.GroundUdpAddrSTR = new(net.UDPAddr)
	M3.TerminalTimeCreated = time.Now() // strconv.FormatInt(common.TBtimestampNano(), 10)
}

// InitFromConfigFile ================================================================
// InitFromConfigFile() - Set configuration from config file
//====================================================================================
func InitFromConfigFile() {
	var fileName string
	argNum := len(os.Args) // Number of arguments supplied, including the command
	fmt.Println("Number of Arguments = ", argNum)
	if argNum > 2 && os.Args[1] != "" && os.Args[1] != "0" {
		M3.M3TerminalName = os.Args[1]
		M3.M3TerminalId, _ = strconv.Atoi(os.Args[2])
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
	err := viper.Unmarshal(&M3) //&droneStruct)
	if err != nil {
		fmt.Printf("Unable to decode into struct, %v", err)
	}

	fmt.Println("Reading variables from ... config", fileName)
	fmt.Println("*************************************************************")
	fmt.Println("TerminalId                 = ", M3.M3TerminalId)
	fmt.Println("TerminalName               = ", M3.M3TerminalName)
	fmt.Println("TerminalIP            		= ", M3.M3TerminalIP)
	fmt.Println("TerminalLogPath            = ", M3.TerminalLogPath)
	fmt.Println("TerminalConnectionTimer    = ", M3.TerminalConnectionTimer)
	fmt.Println("TerminalPort               = ", M3.M3TerminalPort)

	fmt.Println("BroadcastRxIP              = ", M3.Connectivity.BroadcastRxIP)
	fmt.Println("BroadcastRxPort            = ", M3.Connectivity.BroadcastRxPort)
	M3.Connectivity.BroadcastRxAddress =
		M3.Connectivity.BroadcastRxIP + ":" + M3.Connectivity.BroadcastRxPort
	fmt.Println("BroadcastRxAddress        = ", M3.Connectivity.BroadcastRxAddress)

	fmt.Println("UnicastRxIP             = ", M3.Connectivity.UnicastRxIP)
	fmt.Println("UnicastRxPort           = ", M3.Connectivity.UnicastRxPort)
	M3.Connectivity.UnicastRxAddress =
		M3.Connectivity.UnicastRxIP + ":" + M3.Connectivity.UnicastRxPort
	fmt.Println("UnicastRxAddress        = ", M3.Connectivity.UnicastRxAddress)

	fmt.Println("BroadcastTxIP             = ", M3.Connectivity.BroadcastTxIP)
	fmt.Println("BroadcastTxPort          = ", M3.Connectivity.BroadcastTxPort)
	M3.Connectivity.BroadcastTxAddress =
		M3.Connectivity.BroadcastTxIP + ":" + M3.Connectivity.BroadcastTxPort
	fmt.Println("BroadcastTxAddress        = ", M3.Connectivity.BroadcastTxAddress)

	fmt.Println("GroundId                  = ", M3.GroundIP)
	fmt.Println("GroundUdpPort             = ", M3.GroundUdpPort)
	fmt.Println("GroundIPandPort           = ", M3.GroundIPandPort)
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
		fmt.Println("M3 NAME = ", os.Args[1])
		M3.M3TerminalName = os.Args[1]
		M3.M3TerminalId, _ = strconv.Atoi(os.Args[2])
	}
	if argNum > 3 && os.Args[3] != "" && os.Args[3] != "0" {
		fmt.Println("Satellite IP   = ", os.Args[3])
		M3.M3TerminalIP = os.Args[3]
		// TODO: Set eth0 IP address to M3.M3IpAddress !!!
		var ifCmd = exec.Command("sudo", "ifconfig", "eth0",
			M3.M3TerminalIP, "up", "", "")
		output, err := ifCmd.Output()
		fmt.Println("SET MY IP=", "sudo", "ifconfig", "eth0", M3.M3TerminalIP, "up",
			" -OUTPUT:", string(output), " ERR:", err)
	}
	if argNum > 4 && os.Args[4] != "" && os.Args[4] != "0" {
		M3.M3TerminalPort = os.Args[4] // strconv.ParseInt(os.Args[3], 10, 64)
	}
	if argNum > 5 && os.Args[5] != "" && os.Args[5] != "0" && argNum > 6 && os.Args[6] != "" && os.Args[6] != "0" {
		M3.GroundIPandPort = os.Args[5] + ":" + os.Args[6]
	}
}

// SetFinalM3Info ===============================================================
//  Initialize my own info in the M3 structure
//===============================================================================
func SetFinalM3Info() {

	if M3.M3TerminalIP == "" {
		M3.M3TerminalIP, M3.M3TerminalMac = common.GetLocalIp() // get IP and MAC
	}

	// TODO memset(&distanceVector, 0, sizeof(distanceVector))
	M3.TerminalLastChangeTime = float64(common.TBtimestampNano()) // tiM3Now().String()
	changeState(StateDown)

	var bit uint64 = 0x1
	for j := 0; j < 64; j++ {
		// TODO .. from prev coded, should be just = 0xffffffffffffffff
		bit <<= 1
	}

	for m2Number := 0; m2Number < 4; m2Number++ {
		M3.Terminal[m2Number].TerminalMsgLastRcvdAt = time.Now() // TBtimestampNano() //time.Now()
	}
}

// InitM3Connectivity ================================================================
//  Initialize IP and UDP addressing
//====================================================================================
func InitM3Connectivity() {
	M3.M3TerminalIPandPort = M3.M3TerminalIP + ":" + M3.M3TerminalPort
	var err3 error
	M3.TerminalUdpAddrStructure, err3 = net.ResolveUDPAddr("udp", M3.M3TerminalIPandPort)
	if err3 != nil {
		fmt.Println("ERROR ResolveUDPAddr: ", err3)
	} else {
		M3.M3TerminalFullName = common.NameId{Name: M3.M3TerminalName,
			Address: *M3.TerminalUdpAddrStructure}
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

	for i := 1; i < 5; i++ {
		if M3.Terminal[i].TerminalActive == true {
			elapsedTimeSinceLastSend := currTimeMilliSec - M3.Terminal[i].TerminalLastHelloSendTime
			timeSinceLastHelloReceived := currTimeMilliSec - M3.Terminal[i].TerminalLastHelloReceiveTime

			if timeSinceLastHelloReceived > M3.TerminalHelloTimerLength {
				M3.Terminal[i].TerminalActive = false
			} else if elapsedTimeSinceLastSend >= M3.TerminalHelloTimerLength {
				M3.Terminal[i].TerminalLastHelloSendTime = currTimeMilliSec
				// send hello
			}
		}
	}
}

//===============================================================================
// M3 == Satellite, really M3
//===============================================================================
func main() {
	myOS := runtime.GOOS
	fmt.Println("========= M3 START on ", myOS, " at ", time.Now(), "==========")
	//===============================================================================
	// UPDATE RELEVANT VARIABLES and structures in proper order
	//===============================================================================
	InitM3Configuration()
	// Then try to read config file
	InitFromConfigFile()
	// Finally overwrite if any command arguments given
	InitFromCommandLine()

	M3.Terminal[0].TerminalActive = true  // M1
	M3.Terminal[1].TerminalActive = false // M2 1
	M3.Terminal[2].TerminalActive = false // M2 2
	M3.Terminal[3].TerminalActive = false // M2 3
	M3.Terminal[4].TerminalActive = false // M2 4

	SetFinalM3Info()
	// Create LOG file
	common.CreateLog(&Log, M3.M3TerminalName, M3.TerminalLogPath)
	Log.Warning(&Log, "Warning test:this will be printed anyway")

	InitM3Connectivity()

	// Make this work one of these days ...
	var err error
	checkErrorNode(err)

	common.ControlPlaneInit(&(M3.Connectivity), M3.Channels)

	// START SEND AND RECEIVE THREADS:
	err2 := common.ControlPlaneRecvThread(&M3.Connectivity, M3.Channels)
	if err2 != nil {
		fmt.Println(M3.M3TerminalName, "INIT: Error creating Broadcast/Unicast RX thread")
		panic(err2)
	}

	// TODO: Make this work later
	//if M3.GroundIsKnown == true {
	//	fmt.Println(M3.Name, ":",M3.FullName, "INIT: GROUND LOCATED")
	//	changeState(StateConnected)
	//	M3.KeepAliveRcvdTime = time.Now()
	//}

	//================================================================================
	// START TIMER : Call periodicFunc on every timerTick
	//================================================================================
	tick := 300 * time.Millisecond
	fmt.Println(M3.M3TerminalName, "MAIN: Starting a new Timer Ticker at ", tick, " msec")
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
		case UnicastMsg := <-M3.Channels.UnicastRcvCtrlChannel:
			fmt.Println(M3.M3TerminalName, "MAIN: Unicast MSG in state", M3.M3TerminalState, "MSG=", string(UnicastMsg))
			// these include text messages from the ground/controller
			ControlPlaneMessages(UnicastMsg)
		case BroadcastMsg := <-M3.Channels.BroadcastRcvCtrlChannel:
			//fmt.Println(M3.M3Name, "MAIN: Broadcast MSG in state", M3.M3State, "MSG=",string(BroadcastMsg))
			// these include text messages from the ground/controller
			ControlPlaneMessages(BroadcastMsg)
		case MulticastMsg := <-M3.Channels.MulticastRcvCtrlChannel:
			//fmt.Println(M3.M3Name, "MAIN: Multicast MSG in state", M3.M3State, "MSG=",string(MulticastMsg))
			// these include text messages from the ground/controller
			ControlPlaneMessages(MulticastMsg)
		case CmdText, ok := <-ConsoleInput: // These are messsages from local M3 console
			fmt.Println("====> MAIN: Console Input: ", CmdText)

			if !ok {
				fmt.Println("ERROR Reading input from stdin:", CmdText)
				break
			} else {
				// fmt.Println("Read input from stdin:", CmdText)
				// LocalCommandMessages(CmdText)
				// SendTextMsg(stdin)
				// fmt.Println("Console input sent to ground");
				switch string(CmdText[0]) { // switch on console command
				case "status":
					fmt.Println("STATUS REPLY: Name=", M3.M3TerminalName, " State=", M3.M3TerminalState)
				}

			}
			//default:
			//	fmt.Println("done and exit select")
		} // EndOfSelect
	} // EndOfFOR

	// common.ControlPlaneCloseConnections(M3.Connectivity)
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
	fmt.Println("ControlPlaneMessages: Msg=", msg) //.MsgCode, " msglen=", msglen)

	// Was this msg originated by us ?
	if strings.Contains(msgHeader.SrcIP, M3.M3TerminalIP) && sender == M3.M3TerminalId {
		// println("My own message: MsgCode=", msgHeader.MsgCode, " M3.NodeId=", M3.NodeId)
		return
	}
	//============================================================================
	// Is the other side within the RF range ?
	// we need to do this for ethernet connectivity as we receive everything
	//============================================================================
	// First check that the senders id is in valid range
	if sender == M3.M3TerminalId || sender < 1 || sender > 5 {
		println("Sender id WRONG: ", sender, " MsgCode=", msgHeader.MsgCode)
		return
	}
	//node 	:= &M3.NodeList[sender -1]
	fmt.Println("CHECK MESSAGE CODE ", msgHeader.MsgCode)
	switch msgHeader.MsgCode {
	case common.MSG_TYPE_DISCOVERY: // from another M3
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
	var err error
	// TODO  ... add to msg the playing field size .... hmmm ?? relation to random etc
	M3.GroundFullName.Name = msgHeader.SrcName //.DstName
	M3.GroundIP = msgHeader.SrcIP
	M3.GroundIPandPort = string(msgHeader.SrcIP) + ":" + msgHeader.SrcPort
	myPort, _ := strconv.Atoi(msgHeader.SrcPort)
	M3.GroundUdpPort = myPort
	M3.GroundIsKnown = true //msg.GroundUp

	// M3.GroundUdpAddrSTR.IP = msg.TxIP;
	//fmt.Println(TERMCOLOR, "... GROUNDINFO: Name=", M3.GroundFullName.Name,
	//	"  IP:Port=", M3.GroundIPandPort, " StepMode=", msgHeader.StepMode)
	// fmt.Println("Port=", msg.TxPort, " txIP=", msg.TxIP, " " +
	//	"groundIP=",M3.GroundIP)

	M3.GroundUdpAddrSTR, err = net.ResolveUDPAddr("udp", M3.GroundIPandPort)
	//M3.GroundUdpAddrSTR.Port = msg.TxPort
	//M3.GroundUdpAddrSTR.IP   = net.IP(msg.TxIP)
	//M3.GroundUdpAddrSTR.IP 			 = net.IP((M3.GroundIP))
	//M3.GroundUdpAddrSTR.Port, _ 	 = strconv.Atoi(M3.GroundUdpPort)
	//M3.GroundUdpAddrSTR.IP	 = M3.GroundIP // net.IP(M3.GroundIP)

	//fmt.Println("1 M3.GroundUdpAddrSTR=", M3.GroundUdpAddrSTR)
	myPort, _ = strconv.Atoi(msgHeader.DstPort)
	M3.GroundUdpAddrSTR = &net.UDPAddr{IP: net.ParseIP(msgHeader.DstIP), Port: myPort}
	M3.GroundFullName = common.NameId{Name: msgHeader.DstName, Address: *M3.GroundUdpAddrSTR}
	//fmt.Println("2 M3.GroundUdpAddrSTR=", M3.GroundUdpAddrSTR)

	if err != nil {
		fmt.Println("ERROR in net.ResolveUDPAddr = ", err)
		fmt.Println("ERROR locating master, will retry")
		return
	} else {
		// fmt.Println("GROUND INFO: Name=", M3.GroundFullName.Name, "  IP:Port=", M3.GroundIPandPort)
	}
}

// ControlPlaneProcessDiscoveryMessage ===============================================
// Handle DISCOVERY messages in all states
//====================================================================================
func ControlPlaneProcessDiscoveryMessage(msgHeader *common.MessageHeader,
	discoveryMsg *common.DiscoveryMsgBody) {
	//fmt.Println("Discovery MSG in state ", M3.M3State)
	switch M3.M3TerminalState {
	case StateDown:
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
// Me=0, M1=1, M2=2..5
//===========================================================================
func stateConnectedDiscoveryMessage(msgHeader *common.MessageHeader,
	discoveryMsg *common.DiscoveryMsgBody) {
	sender := msgHeader.SrcId
	// TODO ... make sure we only handle configured M1 and M2s
	if sender < 1 || sender > 5 {
		// fmt.Println("DISCARD MSG: invalid senderId=", sender)
		return
	}
	term := &M3.Terminal[sender-1]

	// update info for the sending terminal
	term.TerminalName = msgHeader.SrcName
	term.TerminalId = msgHeader.SrcId
	term.TerminalIP = msgHeader.SrcIP
	term.TerminalMac = msgHeader.SrcMAC
	term.TerminalPort = msgHeader.SrcPort
	term.TerminalNextMsgSeq = msgHeader.SrcSeq
	// Check if terminal was rebooted
	term.TerminalTimeCreated = discoveryMsg.TimeCreated // incarnation #
	term.TerminalLastChangeTime = discoveryMsg.LastChangeTime
	term.TerminalActive = discoveryMsg.NodeActive
	term.TerminalMsgsSent = discoveryMsg.MsgsSent
	term.TerminalMsgsRcvd = discoveryMsg.MsgsRcvd
	term.TerminalMsgLastSentAt = discoveryMsg.MsgLastSentAt
	term.TerminalMsgLastRcvdAt = time.Now() // TBtimestampNano() // time.Now()

	term.TerminalActive = true

	if M3.GroundIsKnown {
		// did this DISCOVERY reach the ground as well
		/*
			if node.NodeDistanceToGround > M3.GroundRadioRange {
				fmt.Println("=== NEED TO FORWARD, OTHER NODE ", int(node.NodeDistanceToGround),
					" AWAY FROM GROUND, Ground at ", M3.GroundRadioRange)
				theNode, distance := FindShortestConnectionToGround()
				fmt.Println("====== Me=", M3.NodeId, " MY DISTANCE=",M3.NodeDistanceToGround,
					" HIS DISTANCE=", node.NodeDistanceToGround," SHORTEST=", distance,
					" Forwarder=", theNode.NodeId)
				if Me == theNode && M3.NodeDistanceToGround <= M3.GroundRadioRange {
					fmt.Println("====== I AM THE FORWARDER ===========================")
					// forward the DISCOVERY as UNICAST to GROUND
					forwardUnicastDiscoveryPacket(msgHeader, discoveryMsg, int(distance))
				}
			} else {
				//fmt.Println("==NODE ", node.NodeId, " CAN REACH GROUND AT",node.DistanceToGround ,
				//	" Ground at ", M3.GroundRadioRange)
			}
		*/
	}
}

/*
func FindShortestConnectionToGround() (*common.NodeInfo, float64) {
	var theNode *common.NodeInfo = nil
	var distance float64 = 1000000000
	for j:=0; j<64; j++ {
		if M3.NodeList[j].NodeActive == true {
			if distance > M3.NodeList[j].NodeDistanceToGround {
				distance = M3.NodeList[j].NodeDistanceToGround
				theNode = &M3.NodeList[j]
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
		M3.M3TerminalActive = true
	case "disable":
		M3.M3TerminalActive = false
	default:
	}
	fmt.Println("RCVD CONSOLE INPUT =", cmdText, " M3 ACTIVE=", M3.M3TerminalActive)
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
	fmt.Println(M3.M3TerminalName, "OldState=", M3.M3TerminalState, " NewState=", newState)
	M3.M3TerminalState = newState
}

//====================================================================================
//  Format and send DISCOVERY msg
//====================================================================================
func sendBroadcastDiscoveryPacket() {
	// ... done in updateConnectivity ...MoveNode(Me)
	//M3.TerminalMsgLastSentAt = float64(common.TBtimestampNano()) // time.Now() //TBtimestampNano()
	//strconv.FormatInt(TBtimestampNano(), 10)

	dstPort := M3.Connectivity.BroadcastTxPort
	// TODO add heightFromEarthCenter as configuration ?? what if it wobbles?
	// timeNow := tiM3.Now()
	msgHdr := common.MessageHeader{
		MsgCode:  "DISCOVERY",
		Ttl:      3,
		TimeSent: float64(common.TBtimestampNano()), // timeNow, //M3.MsgLastSentAt, // time.Now().String()
		SrcSeq:   M3.M3TerminalNextMsgSeq,
		SrcMAC:   M3.M3TerminalMac,
		SrcName:  M3.M3TerminalName,
		SrcId:    M3.M3TerminalId, // node ids are 1 based
		SrcIP:    M3.M3TerminalIP,
		SrcPort:  M3.M3TerminalPort,
		DstName:  "BROADCAST",
		DstId:    0,
		DstIP:    M3.Connectivity.BroadcastTxIP,
		DstPort:  dstPort,
		Hash:     0,
	}
	discBody := common.DiscoveryMsgBody{
		NodeActive: M3.M3TerminalActive,
		MsgsSent:   M3.M3TerminalMsgsSent,
		MsgsRcvd:   M3.M3TerminalMsgsRcvd,
	}
	myMsg := common.MsgCodeDiscovery{
		MsgHeader:    msgHdr,
		MsgDiscovery: discBody,
	}
	M3.M3TerminalNextMsgSeq++
	msg, _ := common.TBmarshal(myMsg)

	// fmt.Println(" BROADCAST DISCOVERY", M3.BroadcastTxStruct)
	common.ControlPlaneBroadcastSend(M3.Connectivity, msg, M3.Connectivity.BroadcastTxStruct)

}

//=================================================================================
//=================================================================================
func sendUnicastStatusReplyPacket(msgHeader *common.MessageHeader) {
	fmt.Println("...... STATUS REPLY: srcIP=", msgHeader.SrcIP, " SrcMAC=", msgHeader.SrcMAC,
		" DstID=", msgHeader.DstId, " SrcID=", msgHeader.SrcId)

	// port, _ :=strconv.Atoi(M3.UnicastTxPort)
	// latitude,longitude := ConvertXYZtoLatLong(M3.MyX, M3.MyY, M3.MyZ, orbitHeightFromEartCenter)
	msgHdr := common.MessageHeader{
		MsgCode:  "STATUS_REPLY",
		Ttl:      1,
		TimeSent: float64(common.TBtimestampNano()),
		SrcSeq:   M3.M3TerminalNextMsgSeq,
		SrcMAC:   M3.M3TerminalMac,
		SrcName:  M3.M3TerminalName,
		SrcId:    M3.M3TerminalId, // node ids are 1 based
		SrcIP:    M3.M3TerminalIP,
		SrcPort:  M3.M3TerminalPort,
		DstName:  "UNICAST",
		DstId:    msgHeader.SrcId,
		DstIP:    msgHeader.SrcIP,
		DstPort:  msgHeader.SrcPort,
		Hash:     0,
	}

	statusReplyBody := common.StatusReplyMsgBody{
		//TimeCreated:	M3.TerminalTimeCreated,
		LastChangeTime: M3.TerminalLastChangeTime,
		NodeActive:     M3.M3TerminalActive,
		MsgsSent:       M3.M3TerminalMsgsSent,
		MsgsRcvd:       M3.M3TerminalMsgsRcvd,
		//MsgLastSentAt:	 M3.TerminalMsgLastSentAt,
	}

	myMsg := common.MsgCodeStatusReply{
		MsgHeader:      msgHdr,
		MsgStatusReply: statusReplyBody,
	}
	M3.M3TerminalNextMsgSeq++
	msg, _ := common.TBmarshal(myMsg)

	common.ControlPlaneUnicastSend(M3.Connectivity, msg, M3.GroundIP+":"+M3.Connectivity.UnicastTxPort)
}

//=================================================================================
//=================================================================================
func sendUnicastDiscoveryPacket(unicastIP string) {
	M3.M3TerminalMsgLastSentAt = float64(common.TBtimestampNano()) //time.Now()

	port := M3.Connectivity.UnicastTxPort //   strconv.Atoi(M3.Connectivity.UnicastTxPort)
	// latitude,longitude := ConvertXYZtoLatLong(M3.MyX, M3.MyY, M3.MyZ, orbitHeightFromEartCenter)
	msgHdr := common.MessageHeader{
		MsgCode:  "DISCOVERY",
		Ttl:      3,
		TimeSent: float64(common.TBtimestampNano()), // timeNow, //M3.MsgLastSentAt, // time.Now().String()
		SrcSeq:   M3.M3TerminalNextMsgSeq,
		SrcMAC:   M3.M3TerminalMac,
		SrcName:  M3.M3TerminalName,
		SrcId:    M3.M3TerminalId, // node ids are 1 based
		SrcIP:    M3.M3TerminalIP,
		SrcPort:  M3.M3TerminalPort,
		DstName:  "UNICAST",
		DstId:    0,
		DstIP:    M3.Connectivity.BroadcastTxIP,
		DstPort:  port,
		Hash:     0,
	}

	discBody := common.DiscoveryMsgBody{
		NodeActive: M3.M3TerminalActive,
		MsgsSent:   M3.M3TerminalMsgsSent,
		MsgsRcvd:   M3.M3TerminalMsgsRcvd,
	}
	myMsg := common.MsgCodeDiscovery{
		MsgHeader:    msgHdr,
		MsgDiscovery: discBody,
	}
	M3.M3TerminalNextMsgSeq++
	msg, _ := common.TBmarshal(myMsg)

	//fmt.Println( "SEND UNICAST DISCOVERY to ", unicastIP)
	common.ControlPlaneUnicastSend(M3.Connectivity, msg, unicastIP)
}

//====================================================================================
//
//====================================================================================
/*
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
		for i := range(keys) {
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
*/

//====================================================================================
// Check if we are talking to the ground/controller station
//====================================================================================
/*
   func isGroundKnown() bool {

	return M3.GroundIsKnown
	// NOTE that this will fail if M3.GroundUdpAddress has not been initialized due
	// to master not being up . Add state and try again to Resolve before doing this
	//var err error
	//fmt.Println("Locate master control, not ground")
	//M3.GroundUdpAddrSTR, err = net.ResolveUDPAddr("udp", M3.GroundIPandPort)
	//if err != nil {
	//	fmt.Println("ERROR in net.ResolveUDPAddr = ", err)
	//	fmt.Println("ERROR locating master, will retry")
	//	return false
	//}
	// TODO check if this is needed
	//M3.GroundFullName = tbMessages.NameId{Name:M3.GroundIPandPort,OsId:0,TimeCreated:"0",Address:*M3.GroundUdpAddress}
	//fmt.Println(M3.M3Name, "INIT: masterFullName=", M3.GroundFullName)
	//mastersEntry := tbMessages.TBmgr{Name:M3.GroundFullName,Up:true,LastChangeTime:"0",MsgsSent:0,LastSentAt:"0",MsgsRcvd:0,LastRcvdAt:"0"}
	//M3.KnownM3s = append(M3.KnownM3s, mastersEntry)
	// fmt.Println("Records of Known Satellites=", M3.KnownM3s)
	// check the master is there:
	//theGround := locateOtherM3(M3.KnownM3s, M3.GroundIPandPort)
	//if theGround != nil {
	//	fmt.Println("Ground at:", theGround.NodeName, "ADDRESS:", theGround.NodeIP, "Port:", theGround.NodePort,
	//		"MSGSRCVD:", theGround.MsgsRcvd)
	//} else {
	//	fmt.Println("GROUND Station Not Detected YET at ", M3.GroundIPandPort)
	//}

	//return true
   }
*/
