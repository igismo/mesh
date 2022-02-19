//*********************************************************************************/
// Copyright 2017 www.igismo.com.  All rights reserved. See open source license
// HISTORY:
// NAME              REV  DATE       REMARKS			@
// Goran Scuric      1.0  01012018  Initial design     goran@usa.net  by igismo
// Goran Scuric		 2.0  09012019  Adapted for bifrost project
// Goran Scuric      3.0  12012020  Adapted for AeroMesh Space Mesh Networking
//================================================================================
package main

import (
	"bufio"
	"flag"
	"fmt"
	"os"
	"runtime"

	//"strconv"
	"strings"
)

var ConsoleInput = make(chan string)

//=======================================================================
// Early work to test some options for officeMaster console
//=======================================================================
func StartConsole(consoleInput <-chan string) {
	// runtime.GOMAXPROCS(2)
	go func(ch <-chan string) {
		// reader := bufio.NewReader(os.Stdin)
		scanner := bufio.NewScanner(os.Stdin)

		for scanner.Scan() {
			commandOK := ""
			//s, err := reader.ReadString('\n')
			//if err != nil { // Maybe log non io.EOF errors, if you want
			//		fmt.Println("Console:Error during ReadString=", err)
			//}
			//fmt.Println("OS=", runtime.GOOS, "  LENGTH=" ,len(s), " INPUT=", s)

			//scanner.Scan()
			fmt.Println("STDIN: LENGTH=", len(scanner.Text()), "INPUT=", scanner.Text())
			s := scanner.Text()
			//fi, err := os.Stdin.Stat()

			s = strings.Replace(s, "\n", "", -1)
			s = strings.Replace(s, "\r", "", -1) // just in a case
			fmt.Println("OS=", runtime.GOOS, " PROCS=", runtime.GOMAXPROCS(0), " LENGTH=", len(s), " INPUT=", s)
			if len(s) < 2 {
				continue
			}

			// SendTextMsg(s)
			// myRecvChannel <- recvBuffer[0:length]
			ConsoleInput <- s
			s1 := strings.Split(s, "\n")
			sa := strings.Split(s1[0], " ")
			//for i := 0; i < len; i++ {println("START SA[", i, "]=", sa[i])}

			switch sa[0] {
			case "quit":
				ControlPlaneCloseConnections()
				fmt.Printf("Exiting\n")
				os.Exit(0)
			case "exit":
				ControlPlaneCloseConnections()
				fmt.Printf("Exiting\n")
				os.Exit(0)
			case "help":
				fmt.Printf("No HELP available yet\n")
			case "enable":
				// Me.NodeActive = true
				//var _ = swapinCommand.Parse(sa[1:])
			case "start":
				//if sa[1]  {
				if sa[1] != "" {
					//RotationPeriod, _ = strconv.ParseFloat(sa[1], 64)
				}
				//}
			case "terminate":
				if sa[1] != "" {
					//sat := MasterLocateSatellite(sliceOfSatellites, sa[1])
					//if sat != nil {
					//	sat.Name.Terminate = true
					//}
				}
			default:
				flag.PrintDefaults()
				fmt.Printf("%q is not valid command.\n", sa[0])
				commandOK = "Error Bad Command"
			}

			if commandOK == "" {
				// fmt.Printf("%q is a GOOD valid command.\n", sa[0])
			}
		} // end of for ever
		fmt.Printf("CONSOLE INPUT SCAN ENDED ...........")
		//close(ch)
	}(consoleInput)
}

//=======================================================================
//
//=======================================================================
func sendStartMsg(project, experiment, userName, fileName string) {
	//	expMgrName := tbConfig.BifrostMasterURL
	//	expMgrFullName := TBmasterLocateMngr(masterSliceOfMgrs, expMgrName)
	//	if expMgrFullName != nil && expMgrFullName.Up == true {
	//		swapIn := tbMessages.SwapIn{Project:project, Experiment:experiment,
	//						UserName:userName, FileName: fileName}
	//		messageBody, _ := tbJsonUtils.TBmarshal(swapIn)
	//		newMsg := tbMsgUtils.TBswapinMsg(masterFullName, expMgrFullName.Name, string(messageBody))
	//
	//		tbMsgUtils.TBsendMsgOut(newMsg, expMgrFullName.Name.Address, masterConnection)
	//	} else {
	//		fmt.Println("Console: Exp Master not available - try later")
	//	}
}

//=======================================================================
//
//=======================================================================
func sendStopMsg(project, experiment, userName, fileName string) {
	//	expMgrName := tbConfig.BifrostMasterURL
	//	expMgrFullName := TBmasterLocateMngr(masterSliceOfMgrs, expMgrName)
	//	if expMgrFullName != nil && expMgrFullName.Up == true {
	//		swapOut := tbMessages.SwapOut{Project:project, Experiment:experiment,
	//			UserName:userName, FileName: fileName}
	//		messageBody, _ := tbJsonUtils.TBmarshal(swapOut)
	//		newMsg := tbMsgUtils.TBswapoutMsg(masterFullName, expMgrFullName.Name, string(messageBody))
	//
	//		tbMsgUtils.TBsendMsgOut(newMsg, expMgrFullName.Name.Address, masterConnection)
	//	} else {
	//		fmt.Println("Console: Exp Master not available - try later")
	//	}
}
