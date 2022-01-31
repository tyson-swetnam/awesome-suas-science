# awesome-suas-science [![Awesome](https://cdn.rawgit.com/sindresorhus/awesome/d7305f38d29fed78fa85652e3a63e154dd8e8829/media/badge.svg)](https://github.com/sindresorhus/awesome)  

Awesome List of the scientific applications for small uncrewed aerial systems (drones) and their data

#### More Awesome Drone lists I drew upon for this list:

* [janesmae/awesome-drones](https://github.com/janesmae/awesome-drones)
* [michidk/awesome-fpv](https://github.com/michidk/awesome-fpv)
* [philgineer/awesome-drone-vision](https://github.com/philgineer/awesome-drone-vision)

## Table of Contents

- [Getting Started](#getting-started)
- [YouTube](#youtube)
- [Software](#software)
  - [Simulators](#simulators)
  - [Mission Planners](#mission-planners)
  - [Firmware](#firmware)
    - [Transmitters](#transmitters)
    - [Flight Controllers](#flight-controllers)  
- [Hardware](#hardware)
  - [Platforms](#platforms)
  - [Remote Transmitters](#remote-transmitters)
  - [Headsets](#headsets)
  - [Lidar](#lidar)
  - [Cameras](#cameras)  
- [Cyberinfrastructure](#data)
- [Data](#data)
- [Community](#community)

- [Free and Open Source Software](#free-and-open-source-software)

## Getting Started

If you're planning on flying drones for professional research, you are going to need to get certified. In the United States, these are the resources you need to know about:

- [Federal Aviation Administration Certified Remote Pilot](https://www.faa.gov/uas/commercial_operators/) 
- [B4UFly](https://www.aloft.ai/b4ufly/) - App for monitoring sUAS flight restrictions and approvals

Internationally, check out these resources:

- [Drone Laws](https://drone-laws.com/)

- [Australia](https://www.casa.gov.au/drones/get-your-operator-credentials/remote-pilot-licence)
- [Canada](https://tc.canada.ca/en/aviation/drone-safety/drone-pilot-licensing/getting-drone-pilot-certificate)
- [Mexico](https://www.gob.mx/tramites/ficha/autorizacion-para-toma-fotografica-filmacion-y-o-videograbacion-en-zonas-monumentos-y-museos-del-inah/INAH1287)
- [New Zealand](https://www.aviation.govt.nz/drones/training-to-fly-unmanned-aircraft/)
- [European Union](https://www.easa.europa.eu/domains/civil-drones)

## YouTube

- [Indiana Drones](https://www.youtube.com/c/IndianaDrones) - drone enthusiast and commercial survey company
- [DroneBot Workshop](https://www.youtube.com/c/Dronebotworkshop1) - DIY drone builds
- [ReadySetDrone](https://www.youtube.com/c/readysetdrone) - drone enthusiasts and DIY drone builds
- [DJI](https://www.youtube.com/c/Dji) - DJI commercials and products

## Software

## Simulators

* [AirSim](https://microsoft.github.io/AirSim/) - Open source simulator based on Unreal Engine for autonomous vehicles.
* [Drone Racing Arcade](https://thedroneracingleague.com/play/) - FPV Racing game and simulator with official DRL tracks.
* [FPV Air 2](https://store.steampowered.com/app/889040/FPV_Air_2/) - :dollar: - Basic FPV simulator, runs on slower hardware. Available on Steam.
* [FPV Freerider](https://fpv-freerider.itch.io/fpv-freerider) - :dollar: FPV (first person view) and LOS (line of sight) racing simulator.
* [FPV Freerider Recharged](https://fpv-freerider.itch.io/fpv-freerider-recharged) - :dollar: FPV (first person view) and LOS (line of sight) racing simulator.
* [LiftOff](https://www.immersionrc.com/fpv-products/liftoff-drone-race-simulator/) - :dollar: FPV racing simulator with realistic OSD (on-screen display) experience.
* [Orqa FPV.SkyDive](https://skydive.orqafpv.com/) - Orqa FPV's racing and freestyle simulator.

## Mission Planners

* [ArduPilot Mission Planner](https://github.com/ArduPilot/MissionPlanner) - Mission planner software.
* [Paparazzi](http://wiki.paparazziuav.org/wiki/Main_Page) - Software suite for UAVs, including ground control and autopilot.
* [QGroundControl](http://qgroundcontrol.com/) - Ground Control Station for PX4 and ArduPilot based UAVs.

## Firmware

### Transmitters

* [FreedomTX](https://github.com/tbs-fpv/freedomtx) - Custom firmware for TBS Tango 2 based on OpenTX.
* [OpenTX](http://www.open-tx.org/) - Highly configurable open source firmware for RC radio transmitters.

### Flight Controllers

* [Ardupilot](https://github.com/ArduPilot/ardupilot)
* [BaseFlight](https://github.com/multiwii/baseflight) - (Outdated/Inactive)
* [Betaflight](https://github.com/betaflight/betaflight) - Fork of Cleanflight.
* [ButterFlight](https://github.com/ButterFlight/butterflight) - Fork of Betaflight. Firmware focusing on Mini Quads.
* [Cleanflight](https://github.com/cleanflight/cleanflight) - Fork of BaseFlight. Supports more FCs and has additional PID contollers.
* [dRonin](https://dronin.org) - Autopilot/flight controller firmware for controllers in the OpenPilot/Tau Labs family.
* [EmuFlight](https://github.com/emuflight/EmuFlight) - FC Firmware focusing on flight performance, innovative filtering, leading-edge feature additions, and wide target support.
* [FlightOne](https://flightone.com/download) - Formerly known as Raceflight one.
* [INAV](https://github.com/iNavFlight/inav)
* [Kiss](https://kiss.flyduino.net) - Firmware for KISS FCs.
* [LibrePilot](https://github.com/librepilot/LibrePilot)
* [Open Source Rover Control Code](https://github.com/nasa-jpl/osr-rover-code) - Nasa JPL command firmware for the OSR.
* [PX4](https://github.com/PX4/Firmware)

## Libraries

* [DJI Onboard SDK](https://github.com/dji-sdk/Onboard-SDK) - The Onboard SDK allows you to connect to a supported DJI flight controller using a serial port (TTL UART).
* [GoBot](https://github.com/hybridgroup/gobot) - Golang framework for robotics, drones, and the Internet of Things (IoT)
* [LibUAVCAN](https://github.com/UAVCAN/libuavcan) - Portable reference implementation of the UAVCAN protocol stack in C++ for embedded systems and Linux
* [MAVLink](https://github.com/mavlink/mavlink) - Micro Air Vehicle Message Marshalling Library
* [MAVROS](https://github.com/mavlink/mavros) - MAVLink to ROS gateway with a proxy for Ground Control Station

## Ground Control Stations

* [QGroundControl](https://github.com/mavlink/qgroundcontrol) - Cross-platform ground control station for drones (Android, iOS, Mac OS, Linux, Windows).
* [Arduleader](https://github.com/geeksville/arduleader) - An android ground controller (and other things) for Mavlink/Arduplane.
* [Tower](https://github.com/DroidPlanner/Tower) - Ground Control Station for Android Devices.
* [MAVProxy](http://ardupilot.github.io/MAVProxy/) - A UAV ground station software package for MAVLink based systems.
* [Ardupilot Mission Planner](https://ardupilot.org/planner/index.html) - A full-featured ground station application for the ArduPilot open source autopilot project.
* [APM Planner 2](https://ardupilot.org/planner2/) - An open-source ground station application for MAVlink based autopilots including APM and PX4/Pixhawk that can be run on Windows, Mac OSX, and Linux.

## Services

* [AirMap](https://www.airmap.com/) - Aeronautical data & services to unmanned aircraft.
* [Airware](https://www.airware.com/) - Data processing platfrom for turning aerial data into business insights.
* [DroneBase](https://dronebase.com/) - Online marketplace for Drone services.
* [DroneDeploy](https://www.dronedeploy.com/) - Drone & UAV Mapping Software.
* [RotorBuilds](https://rotorbuilds.com/) - FPV Part lists and Build Logs.

## Hardware

### Platforms

* [DIY Drones](https://diydrones.com/)
* [OpenUAV](https://openuav.eu/) - Open-souce UAV platform for research and development

### Remote Control Transmitters

* [FlySky](http://www.flyskyrc.com/) - Entry level transmitters.
* [FrSky](https://www.frsky-rc.com/) - Taranis and Horus line of transmitters powered by OpenTX firmware.
* [Futaba](https://www.futabarc.com)
* [Spektrum](https://www.spektrumrc.com)
* [Team Blacksheep](https://team-blacksheep.com) - Tango 1 and 2 transmitters.

### Headsets

* [DJI FPV System](https://www.dji.com/fpv/) - FPV goggles
* [FatShark](https://www.fatshark.com) - FPV googles and controllers
* [ORQA FPV.One](https://orqafpv.com) - FPV goggles and controllers

### Lidar

* [DJI Zenmuze L1](https://www.dji.com/zenmuse-l1)
* [Leica](https://leica-geosystems.com/en-us/products/uav-systems)
* [RockRobotic](https://www.rockrobotic.com/) - Livox platform with integrated camera
* [LiVox](https://www.livoxtech.com)
* [Velodyne](https://velodynelidar.com/)

### Cameras

- [Modus AI drone camera list](https://www.modus-ai.com/drone-mapping-cameras/)

## Structure from Motion

Structure from Motion photogrammetry is a technique, most commercial drones now have integrated cameras and metadata which annotates images, here are some OpenSource Hardware and software integrations:

### Commercial Software

- [Agisoft](https://agisoft.com) :dollar:
- [Pix4D](https://www.pix4d.com/) :dollar:
- [Correlator3DTM](https://cartocanada.ca/product/correlator3dtm-software/) :dollar:

## Free and Open Source Software

- [WebODM](https://www.opendronemap.org/webodm/)

# Data

- [OpenTopography](https://opentopography.org/)
- [Open Drone Map Commmunity](https://community.opendronemap.org/c/datasets/10)
- [Geo Nadir](https://data.geonadir.com/)
- [Drone Mapper](https://dronemapper.com/sample_data/) - sample data :dollar:

## Community
