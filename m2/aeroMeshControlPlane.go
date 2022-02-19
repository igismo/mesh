package main

import (
	//"encoding/hex"
	"fmt"
	"github.com/libp2p/go-reuseport"

	//	"github.com/go-reuseport"
	"log"
	"net"
	"time"
)

//====================================================================================
//
//====================================================================================
func ControlPlaneInit() {
	var err error

	Drone.DroneControlChannel = make(chan []byte) // so that all threads can talk to us
	// UNICAST Drone.UnicastRxPort = "8888",  UnicastRxAddress = "0.0.0.0:8888"
	/*
		//----------------------------------------------------------------------------------------------
		Drone.UnicastRxStruct, err = net.ResolveUDPAddr("udp", Drone.UnicastRxAddress)
		if err != nil { log.Fatal(err) }
		fmt.Println(Drone.DroneName, "1Init ControlPlane UnicastRxStruct", Drone.UnicastRxStruct)
		Drone.UnicastRxConnection, err = net.ListenMulticastUDP("udp", nil, Drone.UnicastRxStruct)
		if err != nil {
			fmt.Println(Drone.DroneName, "2Init ControlPlane ERROR UnicastRxConnection err=", err)
			fmt.Println(Drone.DroneName, "3Init ControlPlane ERROR UnicastRxConnection =", Drone.UnicastRxConnection)
		} else {
			Drone.UnicastRxConnection.SetReadBuffer(Drone.MaxDatagramSize)
			fmt.Println(Drone.DroneName, "4Init ControlPlane OK: UnicastRxAddrStruct=", Drone.UnicastRxStruct)
			fmt.Println(Drone.DroneName, "5Init ControlPlane OK: UnicastRxConnection=", Drone.UnicastRxConnection.LocalAddr())
		}
	*/
	Drone.UnicastRxStruct, err = net.ResolveUDPAddr("udp", ":"+Drone.UnicastRxPort) //Drone.UnicastRxAddress)
	if err != nil {
		log.Fatal(err)
	} else {
		fmt.Println(Drone.DroneName, "1Init OK ControlPlane UnicastRxStruct", Drone.UnicastRxStruct)
		// Drone.UnicastRxConnection, err = net.ListenUDP("udp", Drone.UnicastRxStruct)
		Drone.UnicastRxConnection, err = reuseport.ListenPacket("udp", ":"+Drone.UnicastRxPort) //Drone.UnicastRxAddress)
		if err != nil {
			fmt.Println(Drone.DroneName, "2Init ControlPlane ERROR UnicastRxConnection err=", err)
			panic(err)
		} else {
			//Drone.UnicastRxConnection.SetReadBuffer(Drone.MaxDatagramSize)
			fmt.Println(Drone.DroneName, "3Init ControlPlane OK: UnicastRxConnection=", Drone.UnicastRxConnection.LocalAddr())
			//defer Drone.UnicastRxConnection.Close()
		}
	}
	//BROADCAST
	//----------------------------------------------------------------------------------------------------
	Drone.BroadcastTxStruct, err = net.ResolveUDPAddr("udp", Drone.BroadcastTxAddress) // TODO
	if err != nil {
		fmt.Println(Drone.DroneName, "4Init ControlPlane ERROR BroadcastTxStruct err=", err)
		panic(err)
	}
	fmt.Println(Drone.DroneName, "5Init ControlPlane OK: BroadcastTxStruct=", Drone.BroadcastTxStruct)
	fmt.Println(Drone.DroneName, "6Init ControlPlane Broadcast socket listen on ", Drone.BroadcastRxAddress)
	Drone.BroadcastConnection, err = reuseport.ListenPacket("udp", Drone.BroadcastRxAddress)
	if err != nil {
		fmt.Println(Drone.DroneName, "7Init ControlPlane ERROR BroadcastConnection err=", err)
		panic(err)
	} else {
		fmt.Println(Drone.DroneName, "8Init ControlPlane OK: BroadcastConnection=", Drone.BroadcastConnection.LocalAddr())
		//defer Drone.BroadcastConnection.Close()
	}
	// CHECK IF SAME ADDRESS
	/*
		//----------------------------------------------------------------------------------------------
		if strings.Compare(Drone.BroadcastRxAddress, Drone.UnicastRxAddress) != 0 { // NOT SAME
			//  Initialize Broadcast socket
			fmt.Println(Drone.DroneName, "9Init ControlPlane Broadcast socket listen on ", Drone.BroadcastRxAddress)
			Drone.BroadcastConnection, err = reuseport.ListenPacket("udp", Drone.BroadcastRxAddress)
			if err != nil {
				fmt.Println(Drone.DroneName, "10Init ControlPlane ERROR BroadcastConnection err=", err)
				fmt.Println(Drone.DroneName, "11Init ControlPlane ERROR BroadcastConnection =", Drone.BroadcastConnection)
				panic(err)
			}
		} else { // SAME
			fmt.Println(Drone.DroneName, "12Init ControlPlane Broadcast socket listen on unicast ", Drone.BroadcastRxAddress)
			Drone.BroadcastConnection = Drone.UnicastRxConnection
		}
	*/
	// DONE
	//fmt.Println(Drone.DroneName, "9Init ControlPlane BroadcastRxIP= ",Drone.BroadcastRxIP,"UnicastRxIP=", Drone.UnicastRxIP)
}
func ControlPlaneCloseConnections() {
	fmt.Println(Drone.DroneName, " CLOSE NETWORK CONNECTIONS")
	defer Drone.UnicastRxConnection.Close()
	defer Drone.BroadcastConnection.Close()
}

//====================================================================================
//  Control Plane Listen To Unicast UDP
//====================================================================================
func ControlPlaneListenToUnicastUDP() {
	fmt.Println(Drone.DroneName, "UNICAST *** Start Receiving on ", Drone.UnicastRxConnection.LocalAddr())
	go func() {
		for {
			unicastBuffer := make([]byte, 2400) // Drone.MaxDatagramSize)
			length, sender, err := Drone.UnicastRxConnection.ReadFrom(unicastBuffer)

			if err != nil {
				log.Fatal(sender, "UNICAST ReadFromUDP failed:", err)
			}
			fmt.Println("=====>> UNI-CAST rcv Count=", Drone.DroneReceiveCount,
				" from ", sender.String(), "  ", sender.Network(), "len=", length, "ERR=", err)
			Drone.DroneUnicastRcvCtrlChannel <- unicastBuffer[0:length]
		}
	}()
}

//====================================================================================
//  Control Plane Listen To Broadcast UDP
//====================================================================================
func ControlPlaneListenToBroadcastUDP() {
	fmt.Println(Drone.DroneName, "BROADCAST *** Start Receiving on ", Drone.BroadcastConnection.LocalAddr())
	go func() {
		for {
			broadcastBuffer := make([]byte, 2400)
			length, sender, err := Drone.BroadcastConnection.ReadFrom(broadcastBuffer)
			if err != nil {
				fmt.Println(sender, "ERROR BROADCAST rcv Count=", Drone.DroneReceiveCount, err,
					" from ", sender.String(), "  ", sender.Network(), "len=", length, "ERR=", err)
				panic(err)
			}
			//fmt.Println("*****>> BROADCAST rcv Count=", Drone.DroneReceiveCount,
			//	" from ", sender.String(), "  ", sender.Network(), "len=", length, "ERR=", err)
			Drone.DroneBroadcastRcvCtrlChannel <- broadcastBuffer[0:length]
		}
	}()
}

//====================================================================================
//  Rcv  BROADCAST thread
//====================================================================================
func ControlPlaneRecvThread() error {
	var err error = nil

	fmt.Println(Drone.DroneName, "UDP Receive: ControlPlaneRecvThread: Start RECV THRED")

	if Drone.UnicastRxConnection != nil {
		ControlPlaneListenToUnicastUDP()
	}
	ControlPlaneListenToBroadcastUDP()

	fmt.Println(Drone.DroneName, "==============================================================")
	return err
}

//====================================================================================
//  Control Plane Broadcast Send
//====================================================================================
func ControlPlaneBroadcastSend(pkt []byte, address *net.UDPAddr) {
	//fmt.Println(Drone.DroneName, "ControlPlane SEND to IP=", address)

	_, err := Drone.BroadcastConnection.WriteTo(pkt, address)
	if err != nil {
		fmt.Println(Drone.DroneName, "BROADCAST SEND to ", address.String(), "FAILED: \n ERROR=", err)
	} else {
		//fmt.Println("BROADCAST to", address.String(), " CONN=", Drone.BroadcastConnection)
		// fmt.Print("*")
	}
}

//====================================================================================
// Control Plane Unicast Send
//====================================================================================
func ControlPlaneUnicastSend(msgOut []byte, unicastIPandPort string) {
	var err3 error
	Drone.UnicastTxStruct, err3 = net.ResolveUDPAddr("udp", unicastIPandPort) // TODO
	if err3 != nil {
		fmt.Println("ERROR UNICAST Set UnicastTxStruct Out to ", unicastIPandPort, " Err=", err3)
	}
	_, err4 := Drone.UnicastRxConnection.WriteTo(msgOut, Drone.UnicastTxStruct)
	if err4 != nil {
		fmt.Println("ERROR UNICAST Sending Out to", unicastIPandPort, ": UnicastTxStruct=",
			Drone.UnicastTxStruct, " Err=", err4)
	}
}

//====================================================================================
//
//====================================================================================
func listenToXcastUDP() {
	/*
		fmt.Println(Drone.DroneName, "xCAST   Start Receiving on ", Drone.UnicastConnection.LocalAddr())
		fmt.Println(Drone.DroneName, "xCAST   Start Receiving CONN= ", Drone.UnicastConnection)
		go func() {
			var controlMsg Msg

			for {
				recvBuffer := make([]byte, 3000)
				length, addr, err := Drone.UnicastConnection.ReadFrom(recvBuffer)
				Drone.DroneReceiveCount++
				fmt.Println("*****>> UNICAST RCVD:from", addr.String(), " rxConn",
					Drone.UnicastConnection.LocalAddr().Network(), " ", Drone.UnicastConnection.LocalAddr().String())
				if strings.Compare(addr.String(), Drone.DroneIPandPort) != 0 { // DROP IF FROM US
					fmt.Println("=====>> UNICAST rcv   Count=", Drone.DroneReceiveCount,
						" from ", addr.String(), "  ", addr.Network(), "len=", length, "ERR=", err)
					// Send to channel to be processed by main() loop
					Drone.DroneUnicastRcvCtrlChannel <- recvBuffer[0:length]
				}

				// CHECK CLI CHANNEL
				if len(Drone.DroneCmdChannel) != 0 {
					ctrlMsg := <-Drone.DroneCmdChannel
					_ = TBunmarshal(ctrlMsg, &controlMsg)
					fmt.Println("RecvThread got CONTROL MSG=", controlMsg)
					// check if we are told to terminate
					if strings.Contains(controlMsg.MsgCode, "TERMINATE") {
						fmt.Println("RecvThread rcvd control MSG=", controlMsg)
						return //
					}
				}
			}
		}()

	*/
}

//====================================================================================
//
//====================================================================================
func MulticastPing(serverAddr string) {
	addr, err := net.ResolveUDPAddr("udp", serverAddr)
	if err != nil {
		log.Fatal(err)
	}
	c, err := net.DialUDP("udp", nil, addr)
	for {
		c.Write([]byte("hello, world\n"))
		time.Sleep(1 * time.Second)
	}
}
