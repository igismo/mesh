//=============================================================================
// FILE NAME: tbConfig.go
// DESCRIPTION: configuration, IP addresses, ...
// Notice that Docker provides resolution within created networks
// By default we create "TB-NETWORK" network where all testbed modules live.
// Each docker testbed module will have a preassigned name like "aeroMeshNodex",
// which will be translated by the dockers DNS into its IP address
//
// Copyright 2017 www.igismo.com.  All rights reserved. See license
// HISTORY:
// NAME              REV  DATE       REMARKS			@
// Goran Scuric      1.0  01012018  Initial design     goran@usa.net  by igismo
// Goran Scuric		 2.0  09012019  Adapted for bifrost project
// Goran Scuric      3.0  12012020  Adapted for AeroMesh Space Mesh Networking
//================================================================================
package common

const DRONE_KEEPALIVE_TIMER = 2 // was 5
const MAX_NODES = 64            // anything bigger would require bitMap structure upgrades
// Playing field in km
const X_WIDTH = 100                // 100 km per side
const Y_WIDTH = 100                // 200 km per side
const Z_WIDTH = 100                // 100 km per side
const DEFAULT_VELOCITY = 100       //  km/hour
const DEFAULT_VELOCITY_SCALE = 100 // 0
const THREE_DIMENSIONAL = false
const RANGE_3D = 100 // 170
const RANGE_2D = 100 // 100
const GROUND_STATION_ID = 64
