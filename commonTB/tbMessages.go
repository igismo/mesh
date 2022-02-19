//=============================================================================
// FILE NAME: tbMessages.go
// DESCRIPTION:
// Contains description of all possible messages
//
// NAME              REV  DATE       REMARKS			@
// Goran Scuric      1.0  01012019  Initial design
//================================================================================

package common

import (
	"net"
)

const MAX_MSG_SEQUENCE = 1024

type hashArray struct {
	hashValue [MAX_MSG_SEQUENCE]uint16
	state     [MAX_MSG_SEQUENCE]uint8
}

type BitMask struct {
	M_Mask     uint64
	M_TempMask uint64
	M_Offset   int
	M_BitMask  [MAX_NODES]uint64
}

type NameId struct {
	Name string // Name String
	//OsId        int    // Task id, if known
	//TimeCreated string // my incarnation time
	Address net.UDPAddr
	//Terminate   bool
}

type MessageHeader struct {
	MsgCode  string
	Ttl      int
	StepMode int
	TimeSent float64 // int64  // in milli sec
	SrcSeq   int
	SrcRole  int // node type
	IamAlone bool
	SrcMAC   string

	SrcName string
	SrcId   int // src node
	SrcIP   string
	SrcPort string

	DstName     string
	DstId       int // destination node
	DstIP       string
	DstPort     string
	GroundRange int // are we in range to ground - true or false, or distance?
	Hash        int // hash value for the packet header
}

//type MessageTypeCode struct {0
//	MsgCode string
//}

type Msg struct {
	//	MsgCode	string
	MsgHeader MessageHeader
	MsgBody   []byte
}

type LinuxCommand struct {
	Cmd  string
	Par1 string
	Par2 string
	Par3 string
	Par4 string
	Par5 string
	Par6 string
}
type CommandList []LinuxCommand

//----------------------------------------------------------------------------
// .... MsgCode ...
//----------------------------------------------------------------------------
const MSG_TYPE_CTRL_ADD_DRONE = "DRONE_ADD"
const MSG_TYPE_CTRL_DELETE_DRONE = "DRONE_DELETE"
const MSG_TYPE_CTRL_SET_CANVAS = "SET_CANVAS"
const MSG_TYPE_CTRL_1 = "CTRL_1"
const MSG_TYPE_CTRL_2 = "CTRL_2"
const MSG_TYPE_CTRL_3 = "CTRL_3"

// Messages from ground control
const MSG_TYPE_DRONE_MOVE = "DRONE_MOVE"

type MoveMsgBody struct {
	longitude float64
	latitude  float64
}

const MSG_TYPE_STATUS_REQ = "DRONE_STATUS_REQ"

type MsgCodeStatusRequest struct {
	MsgHeader      MessageHeader
	MsgStatusReply StatusRequestMsgBody
}
type StatusRequestMsgBody struct { // TODO
	seq int    // sender seq number
	cmd string // any sub command
}

const MSG_TYPE_STATUS_REPLY = "DRONE_STATUS_REPLY"

type MsgCodeStatusReply struct {
	MsgHeader      MessageHeader
	MsgStatusReply StatusReplyMsgBody
}
type StatusReplyMsgBody struct {
	TimeCreated    float64 // nanosec
	NodeActive     bool
	LastChangeTime float64 // string	// last time our role changed
	MsgLastSentAt  float64 //time.Time //string // time
	MsgLastRcvdAt  float64 // time.Time //string // time
	MsgsSent       int64
	MsgsRcvd       int64
	Gateways       string // no support in C++ for uint64 // contains all possible relay nodes
	Subscribers    string //uint64
	BaseStations   string //uint64
}

const MSG_TYPE_DRONE_TERMINATE = "DRONE_TERMINATE"

type MsgCodeTerminate struct {
	when float64
}

const MSG_TYPE_GROUND_INFO = "GROUNDINFO"

type MsgCodeGroundInfo struct {
	MsgHeader MessageHeader
}

const MSG_TYPE_STEP = "STEP"

type StepMsgBody struct {
	Steps int
}
type MsgCodeStep struct {
	MsgHeader MessageHeader
	MsgStep   StepMsgBody
}

//====================================
const MSG_TYPE_DISCOVERY = "DISCOVERY"

type MsgCodeDiscovery struct {
	MsgHeader    MessageHeader
	MsgDiscovery DiscoveryMsgBody
}
type DiscoveryMsgBody struct {
	TimeCreated    float64 // nanosec
	NodeActive     bool
	LastChangeTime float64 // string	// last time our role changed
	MsgLastSentAt  float64 //time.Time //string // time
	MsgLastRcvdAt  float64 // time.Time //string // time
	MsgsSent       int64
	MsgsRcvd       int64
	Gateways       string // no support in C++ for uint64 // contains all possible relay nodes
	Subscribers    string //uint64
	BaseStations   string //uint64
}

const MSG_TYPE_CMD = "COMMANDS"

type MsgCmd struct {
	Mgr  TerminalInfo
	cmds []LinuxCommand
}

const MSG_TYPE_CMD_REPLY = "CMD_REPLY"

type MsgCmdReply struct {
	Mgr      TerminalInfo
	CmdReply string
}

const MSG_TYPE_CONNECT = "CONNECT"

type MsgConnect struct {
}

const MSG_TYPE_CONNECTING = "CONNECTING"

type MsgConnecting struct {
}

const MSG_TYPE_CONNECTED = "CONNECTED"

type MsgConnected struct {
}

const MSG_TYPE_DISCONNECT = "DISCONNECT"

type MsgDisconnect struct {
}

const MSG_TYPE_DISCONNECTING = "DISCONNECTING"

type MsgDisconnecting struct {
}

const MSG_TYPE_DISCONNECTED = "MSG_DISCONNECTED"

type MsgDisconnected struct {
}

const MSG_TYPE_TERMINATING = "MSG_TERMINATING"

type MsgTerminating struct {
}

const MSG_TYPE_TERMINATED = "MSG_TERMINATED"

type MsgTerminated struct {
}

//--------------------------------------------------
