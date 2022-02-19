package main

import (
	"github.com/StefanSchroeder/Golang-Ellipsoid/ellipsoid"
	"net"
	"time"
)

type NodeInfo struct {
	NodeTimeCreated    float64 // int64 // string    // time
	NodeLastChangeTime float64 // // string // time.Time // time
	NodeActive         bool
	NodeRole           int
	NodeRoleIndicated  int // whatever other node told us
	NodeIamAlone       bool
	NodeName           string
	NodeId             int
	NodeIP             string
	NodeMac            string
	NodePort           int

	NodeMsgSeq            int
	NodeMsgsSent          int64
	NodeMsgsRcvd          int64
	NodeMsgLastSentAt     float64 //  //time.Time
	NodeMsgLastRcvdLocal  time.Time
	NodeMsgLastRcvdRemote float64 // //time.Time
	NodeLongitude         float64 // ddmm.mmmmm degrees, minutes and minute fractions
	NodeLatitude          float64 // ddmm.mmmmm degrees, minutes and minute fractions
	NodeAltitude          float64 // kilometers
	NodeCourse            float64 // degrees
	NodeRadioRange        int
	NodeDistanceToGround  float64

	// Drones position
	NodeLastMove int64 // time.Time
	MyX          float64
	MyY          float64
	MyZ          float64
	// 1 or -1
	Xdirection int
	Ydirection int
	Zdirection int
	// Velocity components
	Velocity  float64
	VelocityX float64
	VelocityY float64
	VelocityZ float64

	ResidualHop     [MAX_NODES]int8 // residual ttl from remote node to this forwarding node
	SubscriberList  BitMask
	BaseStationList BitMask
	GatewayList     BitMask // contains all current relay nodes
	// TODO DO WE NEED THESE ?? link calculations ??
	NewConnectivity      BitMask
	PreviousConnectivity BitMask
}

// DroneInfo ===============================================================
// The following structure is host drone's owned structure
// NodeInfo (above) is one per each drone
//===============================================================
type DroneInfo struct {
	DroneLogPath      string
	DroneId           int // same as NodeId
	DroneName         string
	DroneState        string
	DroneIpAddress    string
	DroneMacAddress   string
	DroneUdpPort      string
	DroneUdpAddrSTR   *net.UDPAddr
	DroneConnection   *net.UDPConn
	DroneIPandPort    string
	DroneFullName     NameId
	DroneCreationTime string

	DroneConnTimer        int
	DroneReceiveCount     int64
	DroneSendCount        int64
	DroneActivityTimer    int
	DroneDiscoveryTimeout int
	DroneRadioRange       float64
	DroneSpaceDimension   string // "2D or 3D"
	GroundFullName        NameId
	GroundIsKnown         bool
	GroundIP              string
	GroundUdpPort         int
	GroundIPandPort       string
	GroundUdpAddrSTR      *net.UDPAddr
	GroundLatitude        float64
	GroundLongitude       float64
	GroundX               float64
	GroundY               float64
	GroundZ               float64
	GroundRadioRange      float64
	// KnownDrones                  []NodeInfo
	DroneControlChannel          chan []byte
	DroneCmdChannel              chan []byte
	DroneUnicastRcvCtrlChannel   chan []byte
	DroneBroadcastRcvCtrlChannel chan []byte
	DroneMulticastRcvCtrlChannel chan []byte
	GpsDeviceChannel             chan string
	//--------------------------------------
	BroadcastTxIP       string
	BroadcastTxPort     string
	BroadcastTxAddress  string // IP:Port
	BroadcastRxIP       string
	BroadcastRxPort     string
	BroadcastRxAddress  string // IP:Port
	BroadcastFullName   NameId
	BroadcastConnection net.PacketConn
	BroadcastTxStruct   *net.UDPAddr
	//--------------------------------------
	UnicastRxIP         string
	UnicastRxPort       string
	UnicastRxAddress    string
	UnicastRxStruct     *net.UDPAddr
	UnicastTxStruct     *net.UDPAddr
	UnicastRxConnection net.PacketConn //*net.UDPConn
	UnicastTxPort       string
	//--------------------------------------
	MaxDatagramSize        int
	DroneKeepAliveRcvdTime time.Time

	DroneActive          bool
	NodeList             [MAX_NODES]NodeInfo // node struct nodeArray
	DistanceVector       [MAX_NODES]int
	MsgHash              [MAX_NODES]hashArray
	NumberOfNodesLearned int
	// local calculations of where we are
	Velocity          float64
	VelocityScale     float64
	VelocityScaleX    int
	VelocityScaleY    int
	VelocityScaleZ    int
	LastFrameRecvd    [MAX_NODES]string // last time we saw these nodes
	StepMode          int               // how many steps left to execute before freeze
	LastDiscoverySent int64
	DiscoveryInterval int64
	//------------------------------------------------------------
	GpsGroundHeightFromEartCenter  float64
	GpsOrbitAltitudeFromGround     float64
	GpsDevice                      string  // USB or Simulator
	GpsDevicePort                  string  // win:COM11 linux:
	GpsReferenceLatitude           float64 // center of playing field
	GpsReferenceLongitude          float64 // center of playing field
	GpsPlayingFieldSide            float64 // playing field is square eith side equal to this, meters
	GpsPlayingFieldCenterX         float64
	GpsPlayingFieldCenterY         float64
	GpsPlayingFieldCenterZ         float64
	GpsPlayingFieldX               [4]float64 // These are coordinates of 4 corners
	GpsPlayingFieldY               [4]float64
	GpsPlayingFieldZ               [4]float64
	GpsPlayingFieldCornerLatitude  [4]float64
	GpsPlayingFieldCornerLongitude [4]float64
	GpsMyGeometry                  ellipsoid.Ellipsoid
	//------------------------------------------------------------
	// The following are parameters set either by USB GPS device
	// or by software simulator based on random movements
	//------------------------------------------------------------

	GpsCourseOverGround         float64 //
	GpsCourseOverGroundString   string  //
	GpsCourseOverGroundMagnetic float64 //
	GpsMagneticVariation        float64
	GpsSpeedKnots               float64 //
	GpsSpeedKnotsString         string
	GpsSpeedKM                  float64 //
	GpsSpeedKMString            string
	GpsLatitude                 float64 //
	GpsLongitude                float64 //
	GpsAltitude                 float64 // Altitude above mean sea level
	GpsLatitudeString           string  //
	GpsLongitudeString          string  //
	GpsAltitudeString           string  // Altitude above mean sea level
	GpsLastUpdateTime           float64 // NEMA format hhmmss.ss
	GpsLastUpdateDate           float64 // NEMA format ddmm.mmmmm
	GpsLastUpdateTimeString     string  // NEMA format hhmmss.ss
	GpsLastUpdateDateString     string  // NEMA format ddmm.mmmmm
	GpsQualityIndicator         int     // Quality indicator for position fix,
	GpsNumberSatellitesUsed     int     // Number of satellites used (range: 0-12)
	GpsPDOP                     float64 // Position dilution of precision
	GpsVDOP                     float64 // Vertical dilution of precision
	GpsHDOP                     float64 // Horizontal Dilution of Precisio
	GpsGEODseparation           float64 // Difference between geoid and mean sea level, note units
	GpsDiffAge                  int     // Age of differential corrections
	GpsDiffStation              int     // ID of station providing differential corrections
	GpsOperationMode            string
	GpsNavigationMode           string
	GpsLastMessageId            string
	GpsSatellite                [12]int // list of current known satellites
	//----------------------------------------------
}
