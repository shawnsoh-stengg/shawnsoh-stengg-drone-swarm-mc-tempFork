# Hardware Interface Document
## Swarm Search Drone — Mission Computer (AI MC)

| Field | Value |
|---|---|
| Document | MC-HID-001 |
| Version | 0.1-DRAFT |
| Date | 2026-06-20 |
| Parent SRS | MC-SRS-001 v0.1-DRAFT |
| Parent ICD | MC-ICD-001 v0.1-DRAFT |
| Platform | Jetson Orin NX 16GB on Holybro Pixhawk Jetson Baseboard |
| FCC | Pixhawk 6X · PX4 v1.17 · NuttX |

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Platform Overview](#2-platform-overview)
3. [HIF-001: MC ↔ FCC Serial Interface (ttyTHS1)](#3-hif-001-mc--fcc-serial-interface-ttytths1)
4. [HIF-002: MC ↔ FCC Ethernet Interface](#4-hif-002-mc--fcc-ethernet-interface)
5. [HIF-003: MC ↔ IMX219 Camera (CSI-2)](#5-hif-003-mc--imx219-camera-csi-2)
6. [HIF-004: MC ↔ MANET Radio (External Ethernet)](#6-hif-004-mc--manet-radio-external-ethernet)
7. [HIF-005: Power and Electrical](#7-hif-005-power-and-electrical)
8. [HIF-006: Debug and Maintenance Interfaces](#8-hif-006-debug-and-maintenance-interfaces)
9. [HIF-007: MC ↔ Gimbal IP Camera (ETH Port 1)](#9-hif-007-mc--gimbal-ip-camera-eth-port-1)
10. [HIF-008: FCC ↔ H-Flow Optical Flow / Rangefinder Sensor (CAN1)](#10-hif-008-fcc--h-flow-optical-flow--rangefinder-sensor-can1)
11. [Environmental and Mechanical](#11-environmental-and-mechanical)
12. [Connector Quick Reference](#12-connector-quick-reference)
13. [Configuration and Bring-Up Checklist](#13-configuration-and-bring-up-checklist)

---

## 1. Introduction

### 1.1 Purpose

This document defines all physical hardware interfaces on the Mission Computer (MC). It specifies the electrical characteristics, connector types, signal assignments, cable requirements, and configuration procedures for each interface. It is the reference for hardware integration, lab bring-up, and troubleshooting.

### 1.2 Scope

This document covers interfaces at the boundary of the MC assembly: the Holybro Pixhawk Jetson Baseboard populated with a Jetson Orin NX 16GB module and a Pixhawk 6X flight controller. It does not cover:

- Internal FCC-to-peripheral interfaces (GPS, RC receiver, ESCs, power module) — see Holybro Pixhawk 6X documentation.
- Airframe mechanical integration — see airframe drawing package.
- MANET radio internal configuration — see radio manufacturer documentation.
- Software interface behaviour — see MC-ICD-001.

### 1.3 Definitions

| Term | Meaning |
|---|---|
| MC | Mission Computer — Jetson Orin NX 16GB module + carrier (baseboard) |
| FCC | Flight Control Computer — Pixhawk 6X mounted on the baseboard |
| Baseboard | Holybro Pixhawk Jetson Baseboard — the carrier PCB hosting both MC and FCC |
| AU | Air Unit — the MANET mesh datalink radio connected to the MC |
| CSI-2 | MIPI Camera Serial Interface 2 — differential serial bus for camera data |
| JST-GH | JST GH series 1.25 mm pitch connector (standard Pixhawk connector family) |
| FPC | Flat-Flex (Flexible Printed Circuit) cable and connector, used for CSI-2 |
| VBAT | Battery voltage input to the baseboard |
| 5 V rail | Regulated 5 V supply derived on baseboard from VBAT |
| 3.3 V rail | Regulated 3.3 V supply on baseboard for logic interfaces |

---

## 2. Platform Overview

### 2.1 Assembly Description

The MC assembly is a single PCB stack consisting of:

1. **Holybro Pixhawk Jetson Baseboard** — the carrier PCB providing power regulation, connector routing, and inter-module connections.
2. **Jetson Orin NX 16GB module** — mounted on the baseboard via the 260-pin SO-DIMM edge connector (Jetson Orin NX System-on-Module). Runs Ubuntu 22.04 aarch64, JetPack 6.2.2.
3. **Pixhawk 6X** — mounted on the baseboard via the standard Pixhawk autoPilot Module (PAM) connector. Runs PX4 v1.17 on NuttX.

The baseboard bridges power, serial, and Ethernet signals between the two compute modules and exposes external connectors for peripherals.

### 2.2 Interface Overview

```
External peripherals              Baseboard                 Internal modules
────────────────────────────────────────────────────────────────────────────
LiPo battery (4S–6S) ─────────► Power regulation ────────► Jetson (5 V, 3 A)
                                                   ────────► Pixhawk (5 V, 2 A)

MANET radio ──────── RJ45/SFP ──► Ethernet switch (10/100/1000, ─► Jetson eth0 (192.168.10.11)
                                    single switch, no 2nd PHY) ──► Pixhawk Ethernet port (192.168.10.31, 100BASE-T)

                                 UART bridge ──────────────► Jetson ttyTHS1 (uXRCE-DDS primary)
                                 (on-board trace)          ► Pixhawk TELEM2 / UART B

IMX219 camera ─── 22-pin FPC ──► CSI-2 mux/connector ────► Jetson CSI port 0

Gimbal IP camera ─ JST-GH (ETH1) ► Ethernet switch (shared with HIF-002/HIF-004) ──► Jetson eth0 (192.168.10.11)

GCS / AU ─────── see HIF-004 ──► (via MANET radio, HIF-006)

USB-C (debug) ─── USB-C port ──► Jetson USB-C (device / host)
MicroSD ──────── slot on board ► Jetson SDMMC

GPS, RC, ESC, telemetry ────────► Pixhawk JST-GH ports (see Pixhawk 6X docs)

Optical flow + rangefinder ── CAN1 ──► Pixhawk FCC (DroneCAN, IGS-002/
(Holybro H-Flow, downward-facing)       HIF-008 — existing Pixhawk CAN1
                                         port, not a new Jetson connector;
                                         MC-side relay: dedicated MAVLink
                                         channel to fcc_bridge, see §10)
```

![Diagram 22 — Hardware Assembly and Connector Map](docs/diagrams/22-hid-board-assembly.png)

*Diagram 22 — Hardware Assembly and Connector Map: Holybro Pixhawk Jetson Baseboard with all external connectors, internal modules (Jetson Orin NX + Pixhawk 6X), and signal paths.*

### 2.3 Key Interface Summary

| ID | Interface | Path | Primary use |
|---|---|---|---|
| HIF-001 | Serial UART | Baseboard trace: Jetson ttyTHS1 ↔ Pixhawk UART B | uXRCE-DDS primary transport |
| HIF-002 | Ethernet | Baseboard Ethernet switch: Jetson eth0 ↔ Pixhawk Ethernet | uXRCE-DDS secondary / MAVLink fallback |
| HIF-003 | MIPI CSI-2 | FPC 22-pin: IMX219 camera ↔ Jetson CSI port 0 | AI inference, visual odometry |
| HIF-004 | Ethernet (ext.) | RJ45 on baseboard: Jetson ↔ MANET radio AU | GCS link, swarm air-to-air comms |
| HIF-005 | DC power | XT60 battery connector → baseboard power input | Main power for entire assembly |
| HIF-006 | USB-C | USB-C connector on baseboard → Jetson USB3.2 Gen2 | Debug, flashing, peripheral |
| HIF-007 | Ethernet (switched) | JST-GH (ETH port 1) on baseboard's internal Ethernet switch: shared Jetson eth0 ↔ Gimbal IP camera | Gimbal RTSP/H.265 video ingest for AI perception |
| HIF-008 | CAN (DroneCAN) | Pixhawk's existing CAN1 port ↔ Holybro H-Flow optical flow / rangefinder module | Downward optical-flow velocity + rangefinder input, relayed to `gnss_sim_node` (IGS-001) over a dedicated MAVLink channel (§10, ICD §3.3a) |

![Diagram 23 — Signal Interface Block Diagram](docs/diagrams/23-hid-signal-interfaces.png)

*Diagram 23 — Signal Interface Block Diagram: All six hardware interfaces with electrical parameters, connector types, and signal directions.*

---

## 3. HIF-001: MC ↔ FCC Serial Interface (ttyTHS1)

**Primary function:** Micro XRCE-DDS transport between Jetson (uXRCE-DDS Agent) and Pixhawk (uXRCE-DDS Client embedded in PX4 firmware). This is the primary, latency-optimised path for uORB topic bridging.

### 3.1 Physical Layer

| Parameter | Value |
|---|---|
| Interface type | UART (3.3 V logic, async serial) |
| On-board routing | PCB trace on Holybro baseboard — no external cable required |
| Jetson device node | `/dev/ttyTHS1` (Tegra High-Speed UART 1) |
| Pixhawk port | UART B (TELEM2 equivalent) |
| Signal lines used | TX, RX, RTS, CTS (hardware flow control) |
| Signal voltage | 3.3 V CMOS (active high) |
| Maximum cable length | N/A — on-board trace |

> **Note:** On the Holybro Pixhawk Jetson Baseboard, the Jetson ttyTHS1 lines are routed directly to the Pixhawk UART B pads via board traces. No external cable or JST connector is required. The connection is present by default on the assembled board.

### 3.2 Electrical Characteristics

| Signal | Direction | Logic level | Idle state |
|---|---|---|---|
| TX (Jetson → Pixhawk) | Jetson output | 3.3 V CMOS | High |
| RX (Jetson ← Pixhawk) | Jetson input | 3.3 V CMOS | High |
| RTS (Jetson → Pixhawk) | Jetson output | 3.3 V CMOS, active low | High (de-asserted) |
| CTS (Jetson ← Pixhawk) | Jetson input | 3.3 V CMOS, active low | High |

Both Jetson and Pixhawk UART pins are 3.3 V — no level shifting is required.

### 3.3 Baud Rate and Protocol Configuration

| Parameter | Value |
|---|---|
| Baud rate | 921600 bps |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Flow control | RTS/CTS hardware |
| Protocol | Micro XRCE-DDS serial framing over UART |

**PX4 firmware parameter required:**

```
UXRCE_DDS_CFG = 102   (UART B / TELEM2)
UXRCE_DDS_BAUD = 921600
```

Set via QGroundControl → Parameters → UXRCE_DDS_CFG and UXRCE_DDS_BAUD before first flight.

**Jetson Linux device tree:**

ttyTHS1 is exposed under `/dev/ttyTHS1` without additional configuration under JetPack 6.2.2. The device requires `dialout` group membership or a `udev` rule to be accessible within the Docker container:

```
# /etc/udev/rules.d/99-drone-mc.rules
KERNEL=="ttyTHS1", MODE="0660", GROUP="dialout"
```

**Micro XRCE-DDS Agent launch command:**

```bash
MicroXRCEAgent serial --dev /dev/ttyTHS1 \
                       --baudrate 921600 \
                       --rts-cts
```

### 3.4 Link Verification

After power-on, verify the link is operational:

```bash
# On Jetson — check DDS bridge traffic
ros2 topic hz /fmu/out/vehicle_status   # expect ≥5 Hz
ros2 topic hz /fmu/out/vehicle_local_position   # expect ≥10 Hz
```

If the topics are absent, check:
1. `systemctl status drone-mc-fcc-bridge.service` — is the container running?
2. `ls /dev/ttyTHS1` — does the device node exist?
3. PX4 parameter `UXRCE_DDS_CFG` — is it set to UART B?
4. Check for serial framing errors in the Micro XRCE-DDS Agent log.

---

## 4. HIF-002: MC ↔ FCC Ethernet Interface

**Primary function:** Secondary transport path for Micro XRCE-DDS (fallback when HIF-001 is unavailable or for higher-throughput topic streams). Also always carries the MAVLink mission-protocol channel (ICD §3.1, §3.3) regardless of which link is currently active for uXRCE-DDS — stock PX4 has no DDS-writable mission topic, so mission upload/replace has no serial path.

### 4.1 Physical Layer

| Parameter | Value |
|---|---|
| Interface type | 100BASE-T Ethernet (IEEE 802.3u) |
| Routing | On-board Ethernet switch / direct trace on baseboard |
| Jetson interface | `eth0` on Jetson Orin NX (Realtek RTL8211F, 1000BASE-T capable; baseboard may limit to 100 Mbps). Kernel/systemd assigns a PCIe-path predictable name (observed: `enP8p1s0`, board-dependent); Netplan renames it to `eth0` — see §4.2. |
| Pixhawk interface | Pixhawk 6X Ethernet port (100BASE-T, Microchip LAN8742A) |
| Connector (external) | Holybro baseboard provides an RJ45 port shared with external LAN; internal Pixhawk port is connected via baseboard switch fabric |
| Cable (if external jumper used) | Cat 5e or better, shielded, ≤ 1 m |

### 4.2 Network Configuration

**Dedicated internal subnet for MC↔FCC:**

| Interface | IP Address | Subnet | Purpose |
|---|---|---|---|
| Jetson eth0 (MC side) | `192.168.12.1` | 255.255.255.0 | MC endpoint on internal subnet |
| Pixhawk Ethernet (FCC side) | `192.168.12.31` | 255.255.255.0 | FCC endpoint on internal subnet |

This subnet is isolated from the MANET radio subnet (192.168.10.x). The Jetson has two IP addresses on `eth0` — one on each subnet — configured via Netplan. Because the Orin NX built-in NIC does not enumerate as `eth0` (it gets a PCIe-path predictable name from the kernel, e.g. `enP8p1s0`), the config uses `match`/`set-name` to rename it before assigning addresses, so `eth0` stays the stable logical name used everywhere else in this document:

```yaml
# /etc/netplan/01-drone-mc.yaml
network:
  version: 2
  ethernets:
    eth0:
      match:
        name: enP8p1s0        # actual kernel name; update if it differs per baseboard
      set-name: eth0
      addresses:
        - 192.168.10.11/24    # MANET-facing address (HIF-004)
        - 192.168.12.1/24     # FCC-facing address (HIF-002)
      nameservers:
        addresses: []
```

> Renaming an interface that is already up (e.g. via DHCP fallback at boot) can require a reboot before `set-name` takes effect. Verify with `ip -4 addr show eth0` after the first apply; reboot if it's not present yet.

**PX4 Ethernet configuration:**

```
MAV_0_CONFIG = 1000   (Ethernet)
UXRCE_DDS_AG_IP = 3232238593   (= 192.168.12.1 as uint32 big-endian)
UXRCE_DDS_PORT_CFG = 8888
```

**Micro XRCE-DDS Agent (Ethernet transport):**

```bash
MicroXRCEAgent udp4 --port 8888
```

**Mission protocol (always active on this link, independent of §4.3 below):**

```bash
# fcc_bridge_node opens this directly — not routed through mavlink-router
MAVLink 2/UDP  192.168.12.31:14555
```

### 4.3 Transport Selection

Two independent environment variables in `/etc/drone-mc/environment` (ICD §9.4) control transport, and neither affects the mission-protocol socket above:

```
SIMULATION_MODE=false   # false → XrceDdsBackend (production); true → MavlinkUdpBackend (SITL)
FCC_TRANSPORT=serial    # serial (HIF-001, production default) or ethernet (HIF-002, fallback) —
                         # only meaningful when SIMULATION_MODE=false; selects the uXRCE-DDS link
                         # used for vehicle_state/vehicle_command, not the mission-protocol channel
```

The `FccBackend` factory reads `SIMULATION_MODE` to choose the implementation (FCC-001). `XrceDdsBackend` itself only logs `FCC_TRANSPORT`'s value — the DDS link it depends on is made real by a Micro XRCE-DDS Agent process, launched by `docker/fcc-bridge/entrypoint.sh` *before* `fcc_bridge_node` starts (FCC-008). The entrypoint reads `FCC_TRANSPORT` and runs the same two commands documented in §3.3/§4.2 above:

```
serial   → MicroXRCEAgent serial --dev $FCC_SERIAL_DEVICE --baudrate $FCC_SERIAL_BAUD --rts-cts
ethernet → MicroXRCEAgent udp4 --port $FCC_ETH_PORT
```

Both processes run inside the single `mc-fcc-bridge` container and share its lifecycle: either one dying takes the container down, and `drone-mc-fcc-bridge.service`'s `Restart=on-failure` brings both back together. `fcc_bridge_node` itself never spawns or supervises the Agent — that stays systemd's job, not the application's.

![Diagram 25 — Network Interface Topology](docs/diagrams/25-hid-network-topology.png)

*Diagram 25 — Network Interface Topology: IP addressing, subnets, and port map for HIF-002 (internal MC↔FCC subnet) and HIF-004 (MANET external subnet).*

---

## 5. HIF-003: MC ↔ IMX219 Camera (CSI-2)

**Primary function:** High-speed image data transfer from the IMX219 image sensor to the Jetson ISP (Image Signal Processor) and GPU for AI inference and visual odometry.

### 5.1 Physical Layer

| Parameter | Value |
|---|---|
| Interface type | MIPI CSI-2 (Camera Serial Interface 2) |
| Lane configuration | 2-lane CSI-2 |
| Connector type (camera end) | 22-pin FPC (0.5 mm pitch), standard Raspberry Pi CSI-2 pinout |
| Connector type (baseboard end) | 22-pin FPC zero-insertion-force (ZIF) connector on Holybro baseboard |
| Cable type | 15 cm or 30 cm FPC flex cable, 22-pin, 0.5 mm pitch |
| Jetson CSI port | CSI port 0 (sensor-id 0) |
| Maximum cable length | 15 cm recommended; 30 cm maximum |

> **Caution:** The FPC cable is fragile and the ZIF connector latch is easily damaged. Lock the latch before applying power. Ensure the cable is not twisted or kinked. Inspect before each flight.

### 5.2 Connector Pinout (22-pin FPC, CSI-2)

Standard Raspberry Pi v2 CSI-2 pinout, used by both IMX219 and Holybro baseboard connector:

| Pin | Signal | Description |
|---|---|---|
| 1 | GND | Ground |
| 2 | CSI0_DN0 | CSI data lane 0, negative |
| 3 | CSI0_DP0 | CSI data lane 0, positive |
| 4 | GND | Ground |
| 5 | CSI0_DN1 | CSI data lane 1, negative |
| 6 | CSI0_DP1 | CSI data lane 1, positive |
| 7 | GND | Ground |
| 8 | CSI0_CN | CSI clock lane, negative |
| 9 | CSI0_CP | CSI clock lane, positive |
| 10 | GND | Ground |
| 11 | CAM_GPIO | Camera GPIO (reset / standby) |
| 12 | (NC) | Not connected |
| 13 | CAM_GPIO2 | Camera GPIO 2 (optional) |
| 14 | GND | Ground |
| 15 | SCL | I²C clock (camera register config) |
| 16 | SDA | I²C data (camera register config) |
| 17 | CAM_VSYNC | Vertical sync (optional) |
| 18 | CAM_CLKIN | Camera reference clock input from Jetson (24 MHz) |
| 19 | GND | Ground |
| 20 | VDD_3V3 | 3.3 V power from baseboard to camera |
| 21 | VDD_1V8 | 1.8 V power from baseboard to camera (IMX219 core) |
| 22 | GND | Ground |

### 5.3 Electrical Characteristics

| Parameter | Value |
|---|---|
| CSI-2 lane voltage | MIPI D-PHY HS mode 200 mV differential |
| CSI-2 LP voltage | 1.2 V single-ended |
| Camera supply voltage | 3.3 V (analog) + 1.8 V (digital core) |
| Camera supply current (active) | 280 mA @ 3.3 V; 150 mA @ 1.8 V |
| Reference clock | 24 MHz ± 200 ppm (provided by Jetson via baseboard) |
| I²C address (IMX219) | 0x10 |
| I²C bus speed | 400 kHz (Fast mode) |

![Diagram 26 — CSI-2 Camera Interface Detail](docs/diagrams/26-hid-csi2-interface.png)

*Diagram 26 — CSI-2 Camera Interface Detail: HIF-003 signal path from IMX219 camera through 22-pin FPC to Jetson CSI port 0, with D-PHY lane assignments, power rails, and I²C control signals.*

### 5.4 Camera Configuration

The IMX219 is configured via I²C at startup by the GStreamer nvarguscamerasrc driver. No manual register programming is required for the default 1280×720 @ 15 FPS mode.

**Device tree overlay (required under JetPack 6.2.2):**

```
# Verify overlay is loaded
sudo /opt/nvidia/jetson-io/jetson-io.py
# Select: Configure Jetson 24-pin CSI Connector → IMX219
```

**GStreamer pipeline verification:**

```bash
gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=10 ! \
  nvvidconv ! video/x-raw,format=BGRx ! \
  fakesink sync=false -v
```

Expected output: `Got EOS from element "pipeline0"` without errors after 10 frames.

**GStreamer pipeline used in production:**

```bash
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! \
  'video/x-raw(memory:NVMM),width=1280,height=720,framerate=15/1' ! \
  nvvidconv ! video/x-raw,format=BGR ! \
  appsink name=appsink max-buffers=2 drop=true
```

### 5.5 Physical Installation Notes

- Route the FPC cable away from ESC and motor wiring to avoid EMI interference on CSI-2 differential pairs.
- The cable must not be bent at a radius tighter than 5 mm.
- Secure with a cable tie or foam bracket at the mid-point to prevent vibration-induced connector wear.
- Apply a thin layer of conformal coating on the ZIF connector latch area if operating in high-humidity environments.

---

## 6. HIF-004: MC ↔ MANET Radio (External Ethernet)

**Primary function:** All external network traffic: GCS↔MC (MAVLink 2/UDP), GCS↔FCC (MAVLink 2/UDP, via mavlink-router), and air-to-air swarm peer communication (Swarm Protocol/UDP).

### 6.1 Physical Layer

| Parameter | Value |
|---|---|
| Interface type | 1000BASE-T Ethernet (IEEE 802.3ab) |
| Jetson interface | `eth0` on Jetson Orin NX (same physical port as HIF-002 on baseboard switch) |
| Connector on baseboard | RJ45 (8P8C), standard Ethernet jack with integrated magnetics |
| Cable type | Cat 5e or Cat 6, shielded (STP), TIA-568 straight-through |
| Maximum cable length | ≤ 3 m (baseboard to MANET radio AU chassis connector) |
| MANET radio connector | RJ45 (depends on AU model) |

### 6.2 Network Configuration

| Interface | IP Address | Subnet | Gateway |
|---|---|---|---|
| Jetson eth0 | `192.168.10.11` | 255.255.255.0 | `192.168.10.1` (AU routing) |
| Pixhawk FCC (via baseboard) | `192.168.10.31` | 255.255.255.0 | — |
| GCS laptop | `192.168.10.10` | 255.255.255.0 | — |
| MANET AU (IP) | `192.168.10.1` | 255.255.255.0 | — |

All swarm peers use the same 192.168.10.x scheme with sequential `.11`, `.12`, `.13`, … for drone MC instances.

### 6.3 Logical Traffic Map

| Traffic type | Source:port | Destination:port | Interface |
|---|---|---|---|
| GCS → MC MAVLink | 192.168.10.10:any | 192.168.10.11:14551 | eth0 via MANET |
| MC → GCS MAVLink | 192.168.10.11:any | 192.168.10.10:14552 | eth0 via MANET |
| GCS → FCC MAVLink | 192.168.10.10:any | 192.168.10.31:14550 | eth0 via MANET (mavlink-router passes through) |
| Swarm heartbeat | 192.168.10.x:any | 192.168.10.y:14900 | eth0 via MANET |
| Swarm coordination | 192.168.10.x:any | 192.168.10.y:14901 | eth0 via MANET |

### 6.4 Link Verification

```bash
# Ping GCS
ping -c 4 192.168.10.10

# Ping FCC (Pixhawk Ethernet — only if HIF-002 also routed via switch)
ping -c 4 192.168.10.31

# Check Ethernet link state
ip link show eth0 | grep -E "UP|LOWER_UP"

# Check inbound MAVLink traffic
tcpdump -i eth0 -n udp port 14551 -c 20
```

### 6.5 MANET Radio Configuration Requirements

The MANET radio AU must be configured by the radio administrator before integration:

| Setting | Required value |
|---|---|
| Operating mode | Infrastructure (AP or mesh — specific to radio model) |
| Frequency band | Per local spectrum authority |
| Network SSID / mesh ID | Fixed per programme |
| IP address (AU management) | `192.168.10.1` |
| DHCP | Disabled — all endpoints use static IPs |
| Layer 2 bridge | Enabled — AU acts as transparent L2 bridge for the 192.168.10.x subnet |
| MTU | ≥ 1500 bytes |
| QoS | Priority to UDP ports 14550–14902 |

---

## 7. HIF-005: Power and Electrical

### 7.1 Main Power Input

| Parameter | Value |
|---|---|
| Connector type | XT60 (male on baseboard) |
| Input voltage range | 14.8 V – 25.2 V (4S–6S LiPo) |
| Nominal operating voltage | 22.2 V (6S LiPo, 3.7 V/cell × 6) or 14.8 V (4S) |
| Maximum continuous current | 60 A (XT60 connector rated) |
| Maximum peak current | 120 A (≤ 10 s) |

> **Polarity:** XT60 connector is keyed. Positive pin is the larger round conductor. Verify polarity before first connection to a new battery.

### 7.2 Internal Power Rails

The baseboard generates all required supply voltages from VBAT:

| Rail | Voltage | Max current | Consumers |
|---|---|---|---|
| 5V_SYS | 5.0 V ± 5% | 5 A | Jetson Orin NX (primary supply) |
| 5V_FC | 5.0 V ± 5% | 2 A | Pixhawk 6X, GPS module |
| 3.3V_PERIPH | 3.3 V ± 3% | 1 A | Camera VDD_3V3, UART logic pull-ups |
| 1.8V_PERIPH | 1.8 V ± 3% | 500 mA | Camera VDD_1V8 (IMX219 core) |

![Diagram 24 — Power Rail Distribution](docs/diagrams/24-hid-power-distribution.png)

*Diagram 24 — Power Rail Distribution: HIF-005 power path from LiPo battery through XT60 to all onboard consumers (Jetson, Pixhawk, camera rails), with rail voltages, current ratings, and INA226 monitoring.*

### 7.3 Jetson Orin NX Power Budget

| Component | Idle current @5 V | Peak current @5 V |
|---|---|---|
| Jetson SoC (CPU+GPU, MAXN mode) | 0.6 A | 2.5 A |
| LPDDR5 RAM | 0.2 A | 0.8 A |
| NVMe SSD (if fitted) | 0.1 A | 0.6 A |
| HDMI / USB (when active) | 0.1 A | 0.3 A |
| **Total Jetson** | **~1.0 A** | **~4.2 A** |

The 5V_SYS rail is rated at 5 A. Peak transient demand during GPU inference is covered by the baseboard's bulk capacitance (typically 2 × 470 µF on 5V_SYS).

### 7.4 Pixhawk 6X Power Budget

| Mode | Current @5 V |
|---|---|
| Boot / idle | 200 mA |
| Flying (active control, GPS, USB active) | 450 mA |
| Peak (all sensors active, USB, telemetry) | 700 mA |

### 7.5 Inrush Current Handling

On power application, the Jetson Orin NX draws a brief inrush current of ~8 A for ≤ 2 ms as onboard capacitors charge. The baseboard includes an inrush limiter circuit (NTC thermistor in series with 5V_SYS) to limit di/dt. Do not power cycle the assembly faster than 10 s intervals.

### 7.6 Low-Battery Detection

The baseboard power monitor IC (INA226 or equivalent) provides voltage and current measurements accessible via I²C. PX4 reads these via its power monitor driver and publishes `battery_status` at ≥ 2 Hz on uXRCE-DDS. The MC reads this topic and triggers RTL at < 20% remaining (FR-072).

> **Calibration:** Set `BAT1_V_DIV` and `BAT1_A_PER_V` in PX4 parameters to match the baseboard's sense resistor values. Calibration values are printed on the baseboard silkscreen or in the Holybro documentation.

### 7.7 Grounding

All ground signals (GND, DGND) are connected to a single star point on the baseboard. Do not add additional ground connections between the baseboard and airframe structure — this may create ground loops that induce noise on UART and CSI-2 signals.

---

## 8. HIF-006: Debug and Maintenance Interfaces

### 8.1 USB-C Port (Jetson)

| Parameter | Value |
|---|---|
| Connector | USB-C (24-pin reversible) on baseboard |
| USB standard | USB 3.2 Gen 2 (10 Gbps) + USB 2.0 (device mode) |
| Modes | Device: UMS (mass storage), ADB, flashing (recovery mode) |
| Host: USB-A hub for keyboard/mouse/storage | |
| Power delivery | 5 V @ 3 A from baseboard (no external USB PD required) |

**Recovery mode (firmware flash):**
1. Hold the Jetson Force-Recovery (FC) button on the baseboard.
2. Apply power or press Reset.
3. Release FC button after 2 s.
4. Connect USB-C to host PC.
5. Run `sudo ./flash.sh jetson-orin-nano-devkit mmcblk0p1` from NVIDIA SDK Manager.

> **Do not connect to USB-C while drone is armed** — drawing bus power from a flight laptop during flight is a ground loop hazard.

**Initial JetPack 6.2.2 flash (before first assembly, or full re-image):**

This is a one-time, host-side procedure per unit. It must complete — and the
Jetson must boot to a login prompt — before running `scripts/01-install-os.sh`
(§13.2), which configures Netplan and udev on top of a working JetPack image.

1. On an x86_64 Ubuntu 22.04 host, install NVIDIA SDK Manager
   (`sdkmanager`, matching the JetPack 6.2.2 release).
2. Put the Jetson in recovery mode (steps 1–4 above).
3. Launch SDK Manager, select:
   - **Hardware:** Jetson Orin NX 16GB module
   - **JetPack version:** 6.2.2
   - **Target OS:** Ubuntu 22.04 (L4T), aarch64
4. Flash the OS image only (host-side ML/CV SDK components are not required —
   CUDA/TensorRT are provided by the `ros2-core` Docker image; see CLAUDE.md
   Build System).
5. On first boot, complete the Ubuntu OOBE (hostname, user account, locale).
6. Verify the flash before proceeding to software configuration:
   ```bash
   cat /etc/nv_tegra_release          # confirms JetPack/L4T version
   uname -m                           # expect aarch64
   nvcc --version                     # expect CUDA 12.x
   ```

CLI-only flash (no GUI, e.g. CI or headless re-imaging host) is also supported
via `sdkmanager --cli`; see NVIDIA's SDK Manager CLI documentation for the
flag set matching JetPack 6.2.2 / Orin NX.

### 8.2 UART Debug Console

| Parameter | Value |
|---|---|
| Connector | 6-pin JST-GH on baseboard (labelled CONSOLE or DBG) |
| Device node | `/dev/ttyTHS0` or direct to UART0 on Jetson module |
| Baud rate | 115200 bps, 8N1 |
| Logic level | 3.3 V CMOS |
| Function | U-Boot console, Linux kernel boot log, login shell |

**Pinout (6-pin JST-GH, looking into connector):**

| Pin | Signal |
|---|---|
| 1 | +3.3 V (output from baseboard — do not connect) |
| 2 | TX (baseboard output → host RX) |
| 3 | RX (baseboard input ← host TX) |
| 4 | GND |
| 5 | (NC) |
| 6 | (NC) |

Connect via a USB-to-UART adapter (3.3 V compatible, e.g., FTDI FT232R) to a host PC.

### 8.3 Pixhawk Debug Port

| Parameter | Value |
|---|---|
| Connector | 6-pin JST-SH 1.0 mm (Pixhawk Debug Mini connector) |
| Location | On the Pixhawk 6X module edge |
| Function | NuttX system console, PX4 MAVLink shell, SWD JTAG |
| Baud rate | 57600 bps (console), 1 Mbps (SWD) |

See Pixhawk 6X documentation for full pinout.

### 8.4 MicroSD Card Slot

| Parameter | Value |
|---|---|
| Location | On Holybro baseboard, accessible from the side |
| Interface | SDMMC (Jetson native) |
| Capacity | ≤ 256 GB (limited by Jetson SDMMC controller) |
| Speed class | UHS-I U3 minimum recommended |
| Use | OS rootfs boot (if eMMC backup fails), log archive, mission data |

> **Flight use:** The eMMC on the Jetson module is the primary OS storage. The MicroSD is optional for log archiving and should not be relied upon as the primary OS drive due to vibration sensitivity.

### 8.5 Pixhawk MicroSD Slot

Separate MicroSD slot on the Pixhawk 6X module:

| Parameter | Value |
|---|---|
| Use | PX4 flight log recording (`.ulg` files) |
| Capacity | ≤ 64 GB |
| Speed class | UHS-I U3 recommended |

PX4 writes a new `.ulg` file on each arm event. Logs are essential for post-flight analysis. Ensure the MicroSD is seated before each flight.

---

## 9. HIF-007: MC ↔ Gimbal IP Camera (ETH Port 1)

**Primary function:** Ethernet transport for the front gimbal IP camera's RTSP/H.265 video stream, ingested by `perception_front_node` (`video_downlink` package) for onboard AI detection/tracking and re-streamed to the GCS (ICD IFD-006/IFD-007).

### 9.1 Physical Layer

| Parameter | Value |
|---|---|
| Interface type | 1000BASE-TX Ethernet |
| On-board path | JST-GH port on the Holybro baseboard's **internal Ethernet switch** — the same switch fabric used by HIF-002 (Jetson↔Pixhawk) and HIF-004 (MANET radio AU), §2.2 |
| Connector type | JST-GH 6-pin, 1.25 mm pitch — labelled **ETH port 1** on baseboard silkscreen |
| Jetson device node | `eth0` — the Jetson has a single Ethernet NIC; the camera is simply another switched host reachable over the existing HIF-002/HIF-004 interface, not a dedicated Jetson port |
| Cable type | JST-GH 6-pin to RJ45 pigtail (camera end terminates in standard RJ45), Cat 5e, shielded, ≤ 2 m |
| Camera connector | RJ45 (integrated in gimbal camera housing) |

> **Note:** HIF-007 is **not** a second Jetson network interface. The JST-GH ETH port 1 connector wires into the baseboard's existing internal Ethernet switch alongside the Pixhawk Ethernet port (HIF-002) and the MANET radio AU port (HIF-004). The gimbal camera is simply one more host on the 192.168.10.0/24 switched segment, reached through Jetson's single `eth0` interface (192.168.10.11) — no new Netplan interface or kernel device is introduced.

### 9.2 Connector Pinout (6-pin JST-GH)

| Pin | Signal | Description |
|---|---|---|
| 1 | TX+ | Transmit data, positive |
| 2 | TX− | Transmit data, negative |
| 3 | RX+ | Receive data, positive |
| 4 | RX− | Receive data, negative |
| 5 | GND | Ground |
| 6 | GND | Ground |

### 9.3 Network Configuration

| Interface | IP Address | Subnet | Notes |
|---|---|---|---|
| Jetson `eth0` (shared) | `192.168.10.11` | 255.255.255.0 | Existing HIF-002/HIF-004 interface — reused as-is, no change |
| Gimbal IP camera | `192.168.10.200` | 255.255.255.0 | Static, configured on the camera itself. First address of the `.200`+ range reserved for gimbal/camera peripherals on the shared switch |

No Netplan changes are required — the camera is just another host on the existing 192.168.10.0/24 switched segment already configured for `eth0` (HID §4.2). It is reachable, and reaches the Jetson, through the baseboard's internal switch alongside the Pixhawk (HIF-002) and MANET radio AU (HIF-004).

### 9.4 Video Source

| Parameter | Value |
|---|---|
| Protocol | RTSP 1.0, video codec H.265/HEVC |
| Camera RTSP URL | `rtsp://192.168.10.200:8554/stream1` |
| Resolution / frame rate | 1920×1080 @ 30 FPS (configurable on camera) |
| Ingested by | `perception_front_node` (GStreamer `rtspsrc`), see SDD §9a |

### 9.5 Link Verification

```bash
# On Jetson — confirm eth0 is up and camera is reachable
ip -4 addr show eth0
ping -c 4 192.168.10.200

# Confirm RTSP stream is being served
gst-launch-1.0 rtspsrc location=rtsp://192.168.10.200:8554/stream1 ! fakesink -v
```

---

## 10. HIF-008: FCC ↔ H-Flow Optical Flow / Rangefinder Sensor (CAN1)

**Revision note (2026-09-09):** this section originally (incorrectly) assumed a UART-connected PX4FLOW module read directly by the Jetson MC. **Corrected**: the actual selected/wired part is the **Holybro H-Flow**, a DroneCAN peripheral connected to the Pixhawk FCC's existing **CAN1** port — not a new Jetson connector at all. Consequently this is a **Pixhawk-side** interface (like the GPS/RC/TELEM ports already covered only as "Per Pixhawk 6X docs" in §12's J9–J14 row); it is called out with full detail here, as an exception to that usual brevity, because the data it carries is directly consumed by this project's own software (`gnss_sim_node`, IGS-001) — same rationale HIF-007 (gimbal camera) already established for going beyond a generic peripheral's usual one-line treatment.

**Primary function:** Downward-facing optical-flow velocity and time-of-flight rangefinder input, intended for `gnss_sim_node` (`indoor_nav` package, IGS-001) — the dead-reckoning core that lets the drone hold MISSION mode indoors, ahead of `visual_localisation_node`'s (Track B) camera-based VO landing.

> **Decision (2026-09-09): Option 1 — new dedicated MAVLink channel.** H-Flow is read by the **Pixhawk FCC** over DroneCAN, not by the Jetson MC directly. PX4's `uavcan_flow` driver ingests it into the `sensor_optical_flow` uORB topic, and the always-running `vehicle_optical_flow` module further derives `vehicle_optical_flow` (rotated by `SENS_FLOW_ROT`) and `vehicle_optical_flow_vel` (gyro-compensated, distance-scaled body-frame velocity, m/s — logging-only, not bridged anywhere) from it — confirmed against the real PX4 **v1.14.4 tag** source originally (`src/drivers/uavcan/sensors/flow.cpp`, `src/modules/sensors/vehicle_optical_flow/VehicleOpticalFlow.cpp`), re-confirmed unchanged (same paths, same behavior) against **v1.17.0** once this project's target firmware moved (2026-09-11). **Neither topic is bridged out to the companion computer by uXRCE-DDS in stock PX4** — confirmed against the same tag's `src/modules/uxrce_dds_client/dds_topics.yaml`: `sensor_optical_flow` is listed only as `/fmu/in/*` (companion-computer-to-FCC); `distance_sensor` does not appear in the file at all (correcting this note's own first-pass claim that it, too, was `/fmu/in/*` — it simply isn't bridged either direction).
>
> Getting this data onto the MC needed one of:
> 1. **A new dedicated MAVLink channel** (config-only, no firmware rebuild) — **chosen**. PX4 already has `OPTICAL_FLOW_RAD`/`DISTANCE_SENSOR` MAVLink stream implementations (`src/modules/mavlink/streams/OPTICAL_FLOW_RAD.hpp`, `.../DISTANCE_SENSOR.hpp`, confirmed unchanged at the v1.14.4 tag, re-confirmed unchanged at v1.17.0) that can be enabled on a new MAVLink instance/port, mirroring the existing mission-protocol channel precedent (`ICD-MC-001.md` §3.1, IR-007). This is this repo's second MC↔FCC MAVLink exception (a third, IGS-004's HIL_GPS sender, landed later — `ICD-MC-001.md` §3.3b) — `ICD-MC-001.md` §3.3a, `SDD-MC-001.md` §3.2, and `CLAUDE.md`'s architecture summary have all been updated accordingly.
> 2. A custom PX4 `dds_topics.yaml` addition (adds `/fmu/out/sensor_optical_flow` and/or `/fmu/out/vehicle_optical_flow_vel`) — rejected: requires a PX4 firmware rebuild/flash pipeline this repo does not currently have (all other sections in this document assume stock PX4 v1.17 plus documented parameter changes only).
>
> **Axis mapping confirmed safe, not just assumed:** `OPTICAL_FLOW_RAD.integrated_x/y` and `.integrated_xgyro/ygyro` are sourced from the `vehicle_optical_flow` topic, which PX4 rotates by `SENS_FLOW_ROT` (`rotate_3f(...)`, `VehicleOpticalFlow.cpp`) **before** publishing — i.e. before this MAVLink stream ever sees the values. The MC-side relay therefore does not need to re-apply any sensor-mount rotation itself, only the sign flip and gyro-compensation PX4's own `vehicle_optical_flow_vel` derivation performs on the same fields (replicated exactly — see `ICD-MC-001.md` §3.3a and `fcc_bridge/optical_flow_relay.cpp`'s doc comment for the formula and source citation).
>
> **Implemented:** `fcc_bridge`'s `OpticalFlowRelay` (shared by `XrceDdsBackend`'s dedicated port and `MavlinkUdpBackend`'s SITL socket) decodes both messages and publishes `mc_msgs/OpticalFlow` on `/mc/sensors/optical_flow`. See `ICD-MC-001.md` §3.3a for the wire format and velocity formula, and `INDOOR-GNSS-SIMULATION-BACKLOG.md` (Open Item 5) for the ticket-tracking side of this decision.

### 10.1 Physical Layer

| Parameter | Value |
|---|---|
| Interface type | CAN bus (DroneCAN protocol) |
| Connector | Pixhawk 6X **CAN1** port — already exists on the baseboard/Pixhawk today, no new connector required (contrast this section's original, incorrect assumption) |
| Connector type | JST-GH 4-pin, 1.25 mm pitch — "Pixhawk Standard CAN Connector" (Holybro H-Flow product documentation) |
| Cable type | JST-GH 4-pin, per Pixhawk standard CAN cable | 
| Bus | Shared with any other DroneCAN peripheral on CAN1 (none currently documented in this assembly) |
| Termination | H-Flow supports software-controlled CAN termination (per Holybro documentation) — confirm whether it or the Pixhawk-end termination should be enabled for this bus segment during bring-up, not assumed here |

### 10.2 Connector Pinout (4-pin JST-GH, Pixhawk CAN1)

| Pin | Signal | Description |
|---|---|---|
| 1 | VCC | Bus power (Pixhawk standard DroneCAN connector convention) |
| 2 | CAN_H | CAN bus, differential high |
| 3 | CAN_L | CAN bus, differential low |
| 4 | GND | Ground |

> **Verify pin order against the current Holybro pinout diagram before fabrication** (`docs.holybro.com` H-Flow DroneCAN page's pinout figure) — this table reflects the standard Pixhawk CAN connector convention this document assumes elsewhere, not a pin-by-pin confirmation from that image.

### 10.3 Electrical Characteristics

| Parameter | Value |
|---|---|
| Optical flow sensor | PixArt PAA3905E1 — tracks down to > 5 lux, ≤ 42° FOV, 80 mm – 30 m working range, max angular rate 7.4 rad/s, built-in 40 mW/120° IR illumination LED for low-light operation |
| Rangefinder | Broadcom AFBR-S50LV85D time-of-flight — up to 30 m typical, 12.4° × 6.2° FOV (32-pixel array), 850 nm laser, rated for ambient light up to 200k lux, "works on all surface conditions" per vendor documentation |
| Onboard IMU | InvenSense ICM-42688-P, 6-axis (H-Flow's own, independent of the FCC's IMU) |
| MCU | STM32F412CEU6 |
| Weight | 15.2 g with case; 3.5 g PCB only |
| Supply voltage / current | **Not published on Holybro's product/overview pages as of this section's writing** — confirm against the physical unit's silkscreen or a direct support inquiry before bring-up, rather than assume a value. |

### 10.4 Configuration

PX4-side DroneCAN + EKF2 fusion parameters (Holybro H-Flow PX4 setup guide, `docs.holybro.com`):

```
UAVCAN_SUB_FLOW = 1        # enable optical-flow DroneCAN subscription
UAVCAN_SUB_RNG = 1         # enable rangefinder DroneCAN subscription
EKF2_OF_CTRL = 1           # enable EKF2 optical-flow fusion
UAVCAN_RNG_MIN = 0.08      # m — matches the sensor's 80 mm minimum range
UAVCAN_RNG_MAX = 30        # m
EKF2_RNG_A_HMAX = 10       # m — EKF2 rangefinder aiding altitude ceiling
EKF2_RNG_QLTY_T = 0.2
SENS_FLOW_MINHGT = 0.08    # m
SENS_FLOW_MAXHGT = 30      # m
SENS_FLOW_MAXR = 7.4       # rad/s, matches the sensor's max angular rate
SENS_FLOW_ROT = 0          # default: connectors mounted toward the rear of the vehicle
EKF2_OF_POS_X/Y/Z          # sensor offset from vehicle CG — measure and set per airframe
EKF2_GPS_CTRL = 0          # only if intentionally disabling GPS aiding indoors — verify
                           # this doesn't conflict with IGS's own GNSS-lockout design
                           # (INDOOR-GNSS-SIMULATION-BACKLOG.md Safety Considerations §3)
                           # before setting on a real airframe
```

**Mounting:** connectors toward the rear of the vehicle (matches `SENS_FLOW_ROT = 0`'s default assumption — a different mount orientation needs a different value), downward-facing, lens axis normal to the ground plane. Measure and set `EKF2_OF_POS_X/Y/Z` (offset from vehicle center of gravity) per airframe.

**No Jetson-side udev rule or serial parameter is needed** — this section's original UART-based draft's `ttyTHS2`/`optical_flow_driver_node` parameters no longer apply.

### 10.5 Link Verification

**FCC-side (always possible, independent of the MC-relay decision above):**

```
# QGroundControl -> MAVLink Inspector -> DISTANCE_SENSOR
# Confirm current_distance is non-zero and updating.
```

**MC-side (requires the FCC's flow MAVLink instance to be configured and streaming — a deployment step, not yet part of any smoke-test script):**

```
ros2 topic echo /mc/sensors/optical_flow
# Confirm quality > 0 and ground_distance non-zero while the sensor has a
# textured surface and valid range in view.
```

**Bench validation:** see `test-results/IGS-002-hif-008-bench-validation.md` for the current status — the wire-decode/velocity-math layer (`OpticalFlowRelay`) is unit- and integration-tested without hardware; physical bench validation is still blocked on procuring a real H-Flow unit and rig.

---

## 11. Environmental and Mechanical

### 11.1 Operating Limits

| Parameter | Specification |
|---|---|
| Operating temperature (ambient) | −10 °C to +55 °C |
| Storage temperature | −20 °C to +70 °C |
| Relative humidity | 5 % to 95 % non-condensing |
| Maximum altitude | 4500 m MSL (reduced air cooling above 3000 m — monitor Jetson GPU temp via tegrastats) |
| IP rating | Not rated — open-frame PCB assembly requires weather shroud on airframe |

### 11.2 Vibration and Shock

| Parameter | Specification |
|---|---|
| Vibration (operating) | 5–500 Hz, 2 g RMS per axis (typical airframe) |
| Shock (operating) | ≤ 30 g, 11 ms half-sine |
| Vibration isolation | The Holybro baseboard includes anti-vibration standoffs for the Pixhawk module. Mount the baseboard itself on 4× M3 silicone grommets (shore 40A) or equivalent dampers |

> **Critical:** Vibration is the primary cause of FPC connector failure on the CSI-2 camera interface. Use the cable retainer clip or foam bracket as described in §5.5. After any hard landing, inspect the CSI-2 connector before the next flight.

### 11.3 Mechanical Dimensions

| Parameter | Value |
|---|---|
| Baseboard length | 120 mm (approximate — see Holybro drawing) |
| Baseboard width | 40 mm (approximate) |
| Assembly height (board + modules) | 30 mm (approximate, excluding connectors) |
| Mounting holes | 4× M3 through-holes at 30.5 mm × 30.5 mm grid (standard Pixhawk mounting) |
| Mass (board + modules, no cables) | ≈ 85 g |
| Mass (complete assembly with camera and cabling) | ≈ 130 g |

> Use current Holybro datasheet for certified dimension values.

### 11.4 Thermal Management

The Jetson Orin NX SoM includes a heatspreader but no fan. The baseboard does not include an integrated fan. For sustained AI inference loads at ambient temperatures > 40 °C, attach an active cooling solution:

| Cooling solution | Recommended use |
|---|---|
| Passive heatsink on Jetson SoM | Ground lab testing, ambient < 30 °C |
| 30 mm or 40 mm 5 V blower fan | Airborne operation, all conditions |
| nvpmodel mode 0 (MAXN) | Default — maximum performance |
| nvpmodel mode 2 (15 W) | Thermal-limited fallback: reduces GPU clock to limit heat |

Monitor GPU temperature via tegrastats:
```bash
tegrastats --interval 1000 | grep -o 'GPU@[0-9.]*C'
```

Trigger health_monitor_node alert at GPU > 85 °C (SDD §10.1). Switch to nvpmodel mode 2 at 90 °C.

---

## 12. Connector Quick Reference

| ID | Connector label | Type | Pitch | Pins | Function | Mating connector |
|---|---|---|---|---|---|---|
| J1 | POWER IN | XT60 (male) | — | 2 | VBAT input from battery | XT60 female |
| J2 | JETSON CSI | FPC ZIF | 0.5 mm | 22 | IMX219 CSI-2 camera | 22-pin FPC cable |
| J3 | ETH / LAN | RJ45 (8P8C) | — | 8 | External Ethernet to MANET AU | Cat 5e STP straight |
| J4 | USB-C | USB-C | — | 24 | Jetson USB 3.2 Gen2 / debug | USB-C cable |
| J5 | CONSOLE | JST-GH | 1.25 mm | 6 | Jetson UART0 debug console | JST-GH 6-pin |
| J6 | PIXHAWK DBG | JST-SH | 1.0 mm | 6 | Pixhawk NuttX console / SWD | JST-SH 6-pin |
| J7 | MICROSD (J) | MicroSD slot | — | — | Jetson OS / log storage | MicroSD card |
| J8 | MICROSD (P) | MicroSD slot | — | — | PX4 flight log | MicroSD card |
| J9–J14 | PX4 peripherals | JST-GH | 1.25 mm | 4–10 | GPS · RC · TELEM1 · TELEM2 · AUX · I²C | Per Pixhawk 6X docs |
| J15 | ETH port 1 | JST-GH | 1.25 mm | 6 | Gimbal IP camera (shared internal switch port) | JST-GH 6-pin to RJ45 pigtail |

> **HIF-008 (§10) note:** Holybro H-Flow connects to the Pixhawk's existing **CAN1** port (one of the J9–J14 PX4-peripheral connectors above) — no new baseboard connector, unlike this table's original (incorrect) J16 entry.

> **Label positions:** Refer to the Holybro Pixhawk Jetson Baseboard silkscreen for the physical location of each connector. Connector designators (J1–J8) used in this document match the Holybro schematic reference designators.

---

## 13. Configuration and Bring-Up Checklist

Use this checklist for initial lab bring-up and after any hardware change. Each item references the relevant HIF section.

![Diagram 27 — Hardware Bring-Up and Pre-Flight Sequence](docs/diagrams/27-hid-bringup-sequence.png)

*Diagram 27 — Hardware Bring-Up and Pre-Flight Sequence: Ordered flow from first-time assembly through software configuration to pre-flight verification and post-flight inspection.*

### 13.1 First-Time Assembly

- [ ] **HIF-006** Flash the Jetson Orin NX module with JetPack 6.2.2 via NVIDIA SDK Manager (§8.1). Verify it boots to a login prompt before proceeding.
- [ ] **HIF-005** Verify XT60 polarity before connecting battery. Apply power via bench PSU (current limit 5 A) on first connection.
- [ ] **HIF-001** Check baseboard for ttyTHS1 ↔ Pixhawk UART B solder jumpers / board revision compatibility. Confirm on Holybro documentation.
- [ ] **HIF-003** Insert MicroSD cards (Jetson and Pixhawk). Lock the CSI-2 FPC cable into the ZIF connector before applying power.
- [ ] **HIF-006** Connect USB-C to host PC; verify Jetson appears as USB device. Boot to recovery mode if reflashing OS.
- [ ] **HIF-007** Connect the gimbal IP camera's JST-GH-to-RJ45 pigtail to ETH port 1 (J15). Verify camera is powered per its own mounting/power documentation (out of scope here).
- [ ] **HIF-008** Wire the Holybro H-Flow to the Pixhawk's existing CAN1 port (§10.1/§10.2) — no new baseboard connector needed. Mount per §10.4: connectors toward the rear of the vehicle (unless `SENS_FLOW_ROT` will be set differently), downward-facing, lens axis normal to the ground plane.

### 13.2 Software Configuration (one-time, per drone)

- [ ] **HIF-001**, **HIF-002** Run `sudo scripts/01-install-os.sh` on the Jetson — installs the dual-IP Netplan config (192.168.10.11/24 + 192.168.12.1/24 on eth0) and the `/dev/ttyTHS1` udev rule. Verify with `scripts/01b-smoke-test-os.sh` (TP-SM-001).
- [ ] **HIF-001** Set PX4 parameters: `UXRCE_DDS_CFG = 102`, `UXRCE_DDS_BAUD = 921600` via QGroundControl.
- [ ] **HIF-002** Set PX4 Ethernet IP: `UXRCE_DDS_AG_IP = 3232238593`, `UXRCE_DDS_PORT_CFG = 8888`.
- [ ] **HIF-003** Apply Jetson-IO device tree overlay for IMX219 on CSI port 0. Reboot.
- [ ] **HIF-005** Calibrate power monitor: set `BAT1_V_DIV` and `BAT1_A_PER_V` in PX4 to baseboard values.
- [ ] **HIF-007** Configure the gimbal camera with static IP `192.168.10.200/24` (on the camera itself). No Netplan changes on the Jetson are required — verify `ip -4 addr show eth0` still shows `192.168.10.11/24`.
- [ ] **HIF-008** Set the PX4 DroneCAN/EKF2 parameters listed in §10.4 (`UAVCAN_SUB_FLOW`, `UAVCAN_SUB_RNG`, `EKF2_OF_CTRL`, height/rate limits, `EKF2_OF_POS_X/Y/Z` offsets) via QGroundControl and reboot the FCC. Also start a second MAVLink instance on the FCC streaming `OPTICAL_FLOW_RAD`/`DISTANCE_SENSOR` to the MC on `FCC_FLOW_PORT` (`:14556` by default, ICD §3.3a) — no MC-side config needed, `fcc_bridge` binds this port automatically.

### 13.3 Pre-Flight Verification (each flight)

- [ ] **HIF-001** `ros2 topic hz /fmu/out/vehicle_status` — expect ≥ 5 Hz.
- [ ] **HIF-002** (if Ethernet fallback active) `ping 192.168.12.31` from Jetson — expect < 1 ms RTT.
- [ ] **HIF-003** `gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=10 ! fakesink` — no errors.
- [ ] **HIF-004** `ping 192.168.10.10` from Jetson — GCS reachable.
- [ ] **HIF-007** `ping 192.168.10.200` from Jetson — gimbal camera reachable; confirm RTSP stream per §9.5.
- [ ] **HIF-005** Check `battery_status.remaining` in QGroundControl — > 80% before flight.
- [ ] **HIF-006** MicroSD (Pixhawk) seated — PX4 log recording active (green SD light on Pixhawk).
- [ ] **HIF-008** QGroundControl → MAVLink Inspector → `DISTANCE_SENSOR.current_distance` non-zero and updating (§10.5). MC-side: `ros2 topic echo /mc/sensors/optical_flow` shows `quality` non-zero and updating (§10.5).
- [ ] **ALL** Run `10b-smoke-test-networking.sh` — all checks pass.

### 13.4 Post-Flight Inspection

- [ ] **HIF-003** Inspect CSI-2 FPC cable and ZIF connector for signs of wear or displacement.
- [ ] **HIF-007** Inspect the JST-GH-to-RJ45 pigtail and gimbal mount for connector wear or strain.
- [ ] **HIF-008** Inspect the CAN1 JST-GH cable and H-Flow's optical/ToF windows for debris/scratches; re-seat the connector if loose.
- [ ] **HIF-005** Measure battery voltage at XT60 with multimeter. Verify no cell imbalance > 100 mV.
- [ ] **HIF-004** Copy PX4 flight log from Pixhawk MicroSD for archiving.
- [ ] **HIF-006** Archive Jetson system log: `journalctl --since "1 hour ago" > flight_YYYYMMDDHHmm.log`.

---

*MC-HID-001 · v0.1-DRAFT · 2026-06-20*
