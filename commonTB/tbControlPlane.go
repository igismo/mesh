package common

import (
	//"encoding/hex"
	"fmt"
	"github.com/libp2p/go-reuseport"
	"log"
	"net"
	"time"
)

//====================================================================================
//
//====================================================================================
func ControlPlaneInit(connectivity ConnectivityInfo, channels MyChannels) {
	var err error

	channels.ControlChannel = make(chan []byte) // so that all threads can talk to us
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
	connectivity.UnicastRxStruct, err = net.ResolveUDPAddr("udp", ":"+connectivity.UnicastRxPort) //Drone.UnicastRxAddress)
	if err != nil {
		log.Fatal(err)
	} else {
		fmt.Println("1Init OK ControlPlane UnicastRxStruct", connectivity.UnicastRxStruct)
		// Drone.UnicastRxConnection, err = net.ListenUDP("udp", Drone.UnicastRxStruct)
		connectivity.UnicastRxConnection, err =
			reuseport.ListenPacket("udp", ":"+connectivity.UnicastRxPort)
		if err != nil {
			fmt.Println("2Init ControlPlane ERROR UnicastRxConnection err=", err)
			panic(err)
		} else {
			//Drone.UnicastRxConnection.SetReadBuffer(Drone.MaxDatagramSize)
			fmt.Println("3Init ControlPlane OK: UnicastRxConnection=",
				connectivity.UnicastRxConnection.LocalAddr())
			//defer Drone.UnicastRxConnection.Close()
		}
	}
	//BROADCAST
	//----------------------------------------------------------------------------------------------------
	connectivity.BroadcastTxStruct, err = net.ResolveUDPAddr("udp", connectivity.BroadcastTxAddress) // TODO
	if err != nil {
		fmt.Println("4Init ControlPlane ERROR BroadcastTxStruct err=", err)
		panic(err)
	}
	fmt.Println("5Init ControlPlane OK: BroadcastTxStruct=", connectivity.BroadcastTxStruct)
	fmt.Println("6Init ControlPlane Broadcast socket listen on ", connectivity.BroadcastRxAddress)
	connectivity.BroadcastConnection, err =
		reuseport.ListenPacket("udp", connectivity.BroadcastRxAddress)
	if err != nil {
		fmt.Println("7Init ControlPlane ERROR BroadcastConnection err=", err)
		panic(err)
	} else {
		fmt.Println("8Init ControlPlane OK: BroadcastConnection=",
			connectivity.BroadcastConnection.LocalAddr())
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
func ControlPlaneCloseConnections(connectivity ConnectivityInfo) {
	fmt.Println(" CLOSE NETWORK CONNECTIONS")
	defer connectivity.UnicastRxConnection.Close()
	defer connectivity.BroadcastConnection.Close()
}

//====================================================================================
//  Control Plane Listen To Unicast UDP
//====================================================================================
func ControlPlaneListenToUnicastUDP(connectivity ConnectivityInfo, channels MyChannels) {
	fmt.Println("UNICAST *** Start Receiving on ", connectivity.UnicastRxConnection.LocalAddr())
	go func() {
		for {
			unicastBuffer := make([]byte, 2400) // Drone.MaxDatagramSize)
			length, sender, err := connectivity.UnicastRxConnection.ReadFrom(unicastBuffer)

			if err != nil {
				log.Fatal(sender, "UNICAST ReadFromUDP failed:", err)
			}
			fmt.Println("=====>> UNI-CAST rcv ",
				" from ", sender.String(), "  ", sender.Network(), "len=", length, "ERR=", err)
			channels.UnicastRcvCtrlChannel <- unicastBuffer[0:length]
		}
	}()
}

//====================================================================================
//  Control Plane Listen To Broadcast UDP
//====================================================================================
func ControlPlaneListenToBroadcastUDP(connectivity ConnectivityInfo, channels MyChannels) {
	if connectivity.BroadcastConnection != nil {
		fmt.Println("BROADCAST *** Start Receiving on ", connectivity.BroadcastConnection.LocalAddr())
	} else {
		fmt.Println("BROADCAST *** Start Receiving - Connection NOT initialized ")
		return
	}
	go func() {
		for {
			broadcastBuffer := make([]byte, 2400)
			length, sender, err := connectivity.BroadcastConnection.ReadFrom(broadcastBuffer)
			if err != nil {
				fmt.Println(sender, "ERROR BROADCAST rcv err=", err,
					" from ", sender.String(), "  ", sender.Network(), "len=", length, "ERR=", err)
				panic(err)
			}
			//fmt.Println("*****>> BROADCAST rcv Count=", Drone.DroneReceiveCount,
			//	" from ", sender.String(), "  ", sender.Network(), "len=", length, "ERR=", err)
			channels.BroadcastRcvCtrlChannel <- broadcastBuffer[0:length]
		}
	}()
}

//====================================================================================
//  Rcv  BROADCAST thread
//====================================================================================
func ControlPlaneRecvThread(connectivity ConnectivityInfo, channels MyChannels) error {
	var err error = nil

	fmt.Println("UDP Receive: ControlPlaneRecvThread: Start RECV THRED")

	if connectivity.UnicastRxConnection != nil {
		ControlPlaneListenToUnicastUDP(connectivity, channels)
	}
	ControlPlaneListenToBroadcastUDP(connectivity, channels)

	return err
}

//====================================================================================
//  Control Plane Broadcast Send
//====================================================================================
func ControlPlaneBroadcastSend(connectivity ConnectivityInfo, pkt []byte, address *net.UDPAddr) {
	//fmt.Println(Drone.DroneName, "ControlPlane SEND to IP=", address)

	_, err := connectivity.BroadcastConnection.WriteTo(pkt, address)
	if err != nil {
		fmt.Println("BROADCAST SEND to ", address.String(), "FAILED: \n ERROR=", err)
	} else {
		//fmt.Println("BROADCAST to", address.String(), " CONN=", Drone.BroadcastConnection)
		// fmt.Print("*")
	}
}

//====================================================================================
// Control Plane Unicast Send
//====================================================================================
func ControlPlaneUnicastSend(connectivity ConnectivityInfo, msgOut []byte, unicastIPandPort string) {
	var err3 error
	connectivity.UnicastTxStruct, err3 = net.ResolveUDPAddr("udp", unicastIPandPort) // TODO
	if err3 != nil {
		fmt.Println("ERROR UNICAST Set UnicastTxStruct Out to ", unicastIPandPort, " Err=", err3)
	}
	_, err4 := connectivity.UnicastRxConnection.WriteTo(msgOut, connectivity.UnicastTxStruct)
	if err4 != nil {
		fmt.Println("ERROR UNICAST Sending Out to", unicastIPandPort, ": UnicastTxStruct=",
			connectivity.UnicastTxStruct, " Err=", err4)
	}
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
