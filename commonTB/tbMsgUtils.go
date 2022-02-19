//=============================================================================
// FILE NAME: tbMsgUtils.go
// DESCRIPTION:
// Utilities for creating and handling messages used by the
// Experiment Master and Experiment Controllers.
// Contains description of all possible messages related to Experiments
//
// NAME              REV  DATE       REMARKS			@
// Goran Scuric      1.0  01012019  Initial design
//================================================================================
package common

import (
	"fmt"
	"net"
	"time"
)

//============================================================================
// Create a Hello message
//============================================================================
func TBtimestampNano() int64 {
	nano := time.Now().UnixNano() / 1000 // / (int64(time.Millisecond)/int64(time.Nanosecond))
	return nano
}

func TBtimestampMilli() int64 {
	milli := time.Now().UnixNano() / 1000000 // / (int64(time.Millisecond)/int64(time.Nanosecond))
	return milli
}

//====================================================================================
//
//====================================================================================
func TBsendMsgOut(msgOut []byte, udpAddress net.UDPAddr, udpConnection *net.UDPConn) {
	// _, err := Drone.BroadcastConnection.WriteTo(pkt, address)
	if udpConnection != nil { // returns numBytes, err
		_, _ = udpConnection.WriteToUDP(msgOut, &udpAddress)
		// fmt.Println("UNICAST: Sending Out To ", udpAddress, "msg:", msgOut, " CONN=", udpConnection)
	} else {
		fmt.Println("ERROR UNICAST Sending Out to", udpAddress, ": udpConnection = nil")
	}
}

//====================================================================================
// send keep alive messages to everybody registered
// We have to timeout records that have not been renewed for a while - whatever the
// for a while is. We should do it at the time we create the keep alive ....
//====================================================================================
/*func sendKeepAliveMsg() {
	// GS also send only to modules that are up ....
	//{TB-EXPMASTER 1 1522878314281123 {172.18.0.3 1200 }} true 1522878314281123 0 1522880258356310 0 0}}
	// TODO a lots of cleanup and better logic btw this, registration and peiodic time

	//	var names = ""
	//	if len(Drone.KnownDrones) > 0 {
	//		fmt.Println("sendKeepAlive: LENGTH of sliceOfMgrs=", len(Drone.KnownDrones))
	//		msgBody, _ := TBmarshal(Drone.KnownDrones)
	//		fmt.Println("sendKeepAlive: LENGTH of msgBody=", len(msgBody))
	//		for nodeIndex := range Drone.KnownDrones {
	//			receiver := Drone.KnownDrones[nodeIndex].NodeName
	//			if receiver != Drone.DroneName { // Do not send to self
	//				udpAddress := Drone.KnownDrones[nodeIndex].NodeIP
	//				newMsg := TBkeepAliveMsg(Drone.DroneFullName, receiver, string(msgBody))
	//				TBsendMsgOut(newMsg, udpAddress, Drone.DroneConnection)
	//				names += " " + receiver
	//			}
	//		}
	//		fmt.Println(Drone.DroneName,": KNOWN MODULES=", names)
	//	}
}
*/
//because all global variables are defined "static" in the harmony libs, the function
//must be added to udp.c, and cannot "live" outside the udp.c
/*
bool TCPIP_UDP_GetUDPMACHeader(UDP_SOCKET s, TCPIP_MAC_ETHERNET_HEADER* MACHeader)
{
	UDP_SOCKET_DCPT* pSkt = _UDPSocketDcpt(s);

	if((pSkt == 0) || (MACHeader == 0))
	return false;

	#if defined (TCPIP_STACK_USE_IPV4)
	if (pSkt->addType == IP_ADDRESS_TYPE_IPV4)
	{
		if(pSkt->pCurrRxPkt != 0)
		{
			memcpy(MACHeader, (TCPIP_MAC_ETHERNET_HEADER*)pSkt->pCurrRxPkt->pMacLayer, sizeof(TCPIP_MAC_ETHERNET_HEADER));
			return true;
		}
	}
	#endif  // defined (TCPIP_STACK_USE_IPV4)
	return false;
}
*/
//====================================================================================
// sendThread() - Thread sending our messages out
// The caller supplies the control channel over which
// control messages can be received by this thread
// Parameters:	service - 10.0.0.2:1200
// 				sendControlChannel - channel
//====================================================================================
/* KEEP AS REFERENCE
func sendThread(conn *net.UDPConn, sendChannel, sendControlChannel chan []byte) error {
	var err error = nil
	fmt.Println(Drone.DroneName, "SendThread: Start SEND THRED")
	go func() {
		connection := conn
		var controlMsg TBmessage
		fmt.Println(Drone.DroneName, "SendThread: connected")

		Drone.DroneControlChannel <- TBConnectedMsg(Drone.DroneFullName, Drone.DroneFullName, "")

		for {
			select {
			case msgOut := <-sendChannel: // got msg to send out
				// fmt.Println(Drone.DroneName, "SendThread: Sending MSG=",msgOut)
				fmt.Println(Drone.DroneName, "SendThread: Sending MSG out to", Drone.GroundUdpAddress)
				// _, err = connection.Write([]byte(msgOut))
				_, err = connection.WriteToUDP(msgOut, Drone.GroundUdpAddress)
				if err != nil {
					_, _ = fmt.Fprintf(os.Stderr, "Error Sending %s", err.Error())
					// create more descriptive msg
					// send msg up to indicate a problem ?
				}

			case ctrlMsg := <-sendControlChannel: //
				_ = TBunmarshal(ctrlMsg, &controlMsg)
				fmt.Println(Drone.DroneName, "SendThread got control MSG=", controlMsg)

				if strings.Contains(controlMsg.MsgType, "TERMINATE") {
					fmt.Println(Drone.DroneName, "SendThread rcvd control MSG=", controlMsg)
					return
				}
			}
		}

	}()

	return err
}
*/

/*
//====================================================================================
// recvThread - Thread receiving messages from others
//====================================================================================
func RecvThread(conn *net.UDPConn, recvControlChannel <-chan []byte) error {
	var err error = nil

	//fmt.Println(Drone.DroneName,"RecvThread: Start RECV THRED")
	go func() {
		connection := conn

		fmt.Println(Drone.DroneName, "RecvThread: Start Receiving")
		var controlMsg TBmessage
		var oobBuffer [3000]byte
		// Tell main we are coonected all is good
		// Drone.DroneControlChannel <- tbMsgUtils.TBConnectedMsg(Drone.DroneFullName, Drone.DroneFullName, "")

		for {
			recvBuffer := make([]byte, 3000)
			// length, oobn, flags, addr, err := connection.ReadMsgUDP(recvBuffer[0:], oobBuffer[0:])

			length, _, _, _, _ := connection.ReadMsgUDP(recvBuffer[0:], oobBuffer[0:])
			Drone.DroneReceiveCount++
			//fmt.Println(Drone.DroneName, "\n============== Receive Count=", myReceiveCount,
			//	"\nRecvThread UDP MSG from", addr, "len=", length, "oobLen=", oobn, "flags=", flags, "ERR=", err)
			// fmt.Println(Drone.DroneName,"RecvThread MSG=", string(recvBuffer[0:length]))

			Drone.DroneRcvCtrlChannel <- recvBuffer[0:length]

			if len(recvControlChannel) != 0 {
				ctrlMsg := <-recvControlChannel
				_ = TBunmarshal(ctrlMsg, &controlMsg)
				fmt.Println("RecvThread got CONTROL MSG=", controlMsg)
				if strings.Contains(controlMsg.MsgType, "TERMINATE") {
					fmt.Println("RecvThread rcvd control MSG=", controlMsg)
					return
				}
			}
		}
	}()

	return err
}

//====================================================================================
//  S
//====================================================================================
func sendUDPHello(conn *net.UDPConn, addr *net.UDPAddr) {
	n, err := conn.WriteTo([]byte("hello"), addr)
	if err != nil {
		log.Fatal("Write:", err)
	}
	fmt.Println("Sent", n, "bytes", conn.LocalAddr(), "->", addr)
}
*/
//=============================================================================
// Function:    expMessage
// Description:
//         Create marshaled message ready to be sent out.
// Input:  sender,receiver    = name of the sender and receiver
//         mclass,mtype,mbody = message class and type, message body string
// Output: msg = marshalled message
// Error Conditions:
//      None [or state condition for each possible error]
//=============================================================================
/*
func TBmarschalMessage(sender, receiver NameId, mtype, mbody string) []byte {
	// currentTime := strconv.FormatInt(Nano(), 10)

	msgBytes := []byte(mbody)
	myMsg := TBmessage{
		TxName:   sender.Name,
		TxIP:     string(sender.Address.IP),
		TxPort:   sender.Address.Port,
		RxName:   receiver.Name,
		RxIP:     string(receiver.Address.IP),
		RxPort:   receiver.Address.Port,
		MsgType:  mtype,
		TimeSent: time.Now().String(),
		MsgBody:  msgBytes,
	}

	msg, _ := TBmarshal(myMsg)
	return msg
}
*/

/*
//============================================================================
// Create a Hello message (to send to office mgr)
//============================================================================
func TBConnectedMsg(sender, receiver NameId, mBody string) []byte {
	//fmt.Println("MsgUtils: create CONNECTED msg")
	msg := TBmarschalMessage(sender, receiver, MSG_TYPE_CONNECTED, mBody)
	return msg
}

//============================================================================
// Create a Hello message (to send to office mgr)
//============================================================================
func TBkeepAliveMsg(sender, receiver NameId, mBody string) []byte {
	//fmt.Println("MsgUtils: create KEEP ALIVE msg, mBody=", mBody)
	msg := TBmarschalMessage(sender, receiver, MSG_TYPE_KEEPALIVE, mBody)
	return msg
}

//============================================================================
// Create a Hello message (to send to office mgr)
//============================================================================
func BiRouteUpdateMsg(sender, receiver NameId, mBody string) []byte {
	// fmt.Println("MsgUtils: create Route Update for ", receiver, " mBody=", mBody)
	msg := TBmarschalMessage(sender, receiver, MSG_TYPE_CMD, mBody)
	return msg
}

//============================================================================
// Create a special command message
//============================================================================
func BiControlMsg(sender, receiver NameId, mBody string) []byte {
	fmt.Println("MsgUtils: create COMMANDS msg for ", receiver.Name, "  mBody=", mBody)
	replyBuffer := TBmarschalMessage(sender, receiver, MSG_TYPE_TERMINATE, mBody)
	return replyBuffer
}

//============================================================================
// Create a Hello message (to send to office mgr)
//============================================================================

// INSTEAD OF  receiver provide *msg.MsgReceiver
// AND extract all of receivers fields

func TBhelloMsg(sender, receiver NameId, mBody string) []byte {
	//fmt.Println("MsgUtils: create HELLO msg, mBody=", mBody)
	msg := TBmarschalMessage(sender, receiver, MSG_TYPE_HELLO, mBody)
	return msg
}

//============================================================================
// Create a REGISTER message (to send to office mgr)
//============================================================================
func TBregisterMsg(sender, receiver NameId, mBody string) []byte {
	//fmt.Println("MsgUtils: create REGISTER msg, mBody=",mBody)
	msg := TBmarschalMessage(sender, receiver, MSG_TYPE_REGISTER, mBody)
	return msg
}

//============================================================================
// Create a Hello message (to send to office mgr)
//============================================================================
func TBhelloReplyMsg(sender, receiver NameId, mBody string) []byte {
	//fmt.Println("MsgUtils: create HELLO REPPLY msg, mBody=", mBody)
	msg := TBmarschalMessage(sender, receiver, MSG_TYPE_HELLO_REPPLY, mBody)
	return msg

}

//====================================================================================
//
//====================================================================================
func sendHelloReplyMsg(senderName NameId, msg *TBmessage) {
	//	receiver := msg.MsgSender
	//	newMsg := TBhelloMsg(senderName, receiver, senderName.Name + " is alive")
	//	TBsendMsgOut(newMsg, receiver.Address, myConnection)
}
*/
//============================================================================
// Create a Hello message (to send to office mgr)
//============================================================================
/*
func TBmsgSample(sender, receiver NameId, mBody string) []byte {


	msgBytes := []byte(mBody)

	myMsg := TBmessage{
		TxName: sender.Name,
		TxIP:   string(sender.Address.IP),
		TxPort: sender.Address.Port,
		RxName: receiver.Name,
		RxIP:   string(receiver.Address.IP),
		RxPort: receiver.Address.Port,
		// MsgClass:    tbMessages.MSG_CLASS_CONTROL,
		MsgType: MSG_TYPE_INIT,
		MsgLen:  1,
		MsgBody: msgBytes,
	}

	msg, _ := TBmarshal(myMsg)
	// fmt.Println("JSON=", string(msg))
	return msg
} */
//====================================================================================
//
//====================================================================================
//func sendHelloReplyMsg(msg *tbMessages.TBmessage) {
//	receiver := msg.MsgSender
//	newMsg := tbMsgUtils.TBhelloMsg(Drone.DroneFullName, receiver, "ABCDEFG")
//	tbMsgUtils.TBsendMsgOut(newMsg, receiver.Address, Drone.DroneConnection)
//}
//func SendTextMsg(str string) {
//	m := str // "HELLO THERE AGAIN:" +str
//	//receiver := sliceOfSatellites[1].Name
//	// udpAddress := sliceOfSatellites[1].Name.Address
//	//newMsg := tbMsgUtils.TBkeepAliveMsg(masterFullName, receiver, str)
//	// tbMsgUtils.TBsendMsgOut(newMsg, udpAddress, masterConnection)
//	TBsendMsgOut([]byte(m), *Drone.GroundUdpAddress, Drone.DroneConnection)
//	// fmt.Println("msg sent to MASTER/GROUND")
//}
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
//====================================================================================
// ???
//====================================================================================
//func formatReceiver(name string, osId int, udpAddress net.UDPAddr) tbMessages.NameId {
//	receiver := tbMessages.NameId{Name: name, OsId: osId,
//		TimeCreated: "", Address: udpAddress}
//	return receiver
//}
