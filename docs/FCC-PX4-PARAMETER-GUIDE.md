# FCC Parameter & Configuration Guide (PX4 v1.17, Pixhawk 6X)

Every FCC-side setting this project needs, grouped by category, with the indoor (`HIL_GPS` / `gnss_sim_node`) and outdoor (real GNSS receiver) values side by side.

This guide collects settings that are otherwise spread across `HID-MC-001.md` §3.3, §4.2, §10.4, §10.6 and §10.7, `ICD-MC-001.md` §3.1–3.3b, and `docs/FT-006-BENCH-RUNBOOK.md`. Those sections remain the source of the reasoning behind each value. This page is the checklist. Parameter names and defaults were checked against the real PX4 **v1.17.0** tag (`~/PX4-Autopilot`) on 2026-10-20. Where this page and an older doc disagree, see [§12](#12-corrections-made-to-other-docs).

Run `param` and `mavlink` commands from QGC → Analyze Tools → MAVLink Console, or `mavlink_shell.py`. Finish every batch with `param save` and `reboot`. Most of the parameters below are `reboot_required`.

## Contents

1. [Profiles at a glance](#1-profiles-at-a-glance)
2. [Vehicle identity](#2-vehicle-identity)
3. [MC↔FCC uXRCE-DDS, serial (primary)](#3-mcfcc-uxrce-dds-serial-primary)
4. [Ethernet: FCC address, Channel A, uXRCE-DDS fallback](#4-ethernet-fcc-address-channel-a-uxrce-dds-fallback)
5. [MC↔FCC MAVLink exceptions (`extras.txt`)](#5-mcfcc-mavlink-exceptions-extrastxt)
6. [DroneCAN: H-Flow optical flow + rangefinder](#6-dronecan-h-flow-optical-flow--rangefinder)
7. [GNSS source & EKF2](#7-gnss-source--ekf2)
8. [Barometer](#8-barometer)
9. [Geofence, RTL & failsafes](#9-geofence-rtl--failsafes)
10. [Flight envelope (mission speeds & takeoff)](#10-flight-envelope-mission-speeds--takeoff)
11. [Power monitoring](#11-power-monitoring)
12. [Corrections made to other docs](#12-corrections-made-to-other-docs)
13. [Switching profiles](#13-switching-profiles)
14. [Future: OFFBOARD mode](#14-future-offboard-mode)

---

## 1. Profiles at a glance

| Category | Common (every drone, every session) | Indoor (`HIL_GPS`) | Outdoor (real GNSS) |
|---|---|---|---|
| Identity | `MAV_SYS_ID = DRONE_ID` | — | — |
| uXRCE-DDS | `UXRCE_DDS_CFG 102`, `SER_TEL2_BAUD 921600`, `UXRCE_DDS_PTCFG 2` | — | — |
| Ethernet | `net.cfg` `IPADDR = FCC_ETH_IP`; `MAV_2_*` Channel A board defaults | — | — |
| `extras.txt` | `:14555` mission, `:14556` flow + baro relay | `:14557` `HIL_GPS` instance | `:14557` may stay (inert with `MAV_USEHILGPS 0`) |
| DroneCAN / H-Flow | `UAVCAN_*`, `SENS_FLOW_*`, `EKF2_RNG_*`, `EKF2_OF_*` | — | — |
| GNSS | `EKF2_GPS_CTRL 7` (never 0) | `MAV_USEHILGPS 1`; optional `GPS_1_CONFIG 0` | `MAV_USEHILGPS 0`; `GPS_1_CONFIG` = GPS1 |
| Barometer | — | `SENS_BAR_AUTOCAL 0` | `SENS_BAR_AUTOCAL 1` |
| Geofence / RTL / failsafe | — | Room-sized: `GF_*` on, `RTL_*` ≈ 2 m, Land on link loss | Site-specific (risk assessment) |
| Envelope | — | Slow: 1 m/s cruise, 1.5 m takeoff | Site-specific |
| Power | `BAT1_V_DIV`, `BAT1_A_PER_V` from baseboard | — | — |

---

## 2. Vehicle identity

| Parameter | Default | Value | Notes |
|---|---|---|---|
| `MAV_SYS_ID` | 1 | **= the MC's `DRONE_ID`** (`/etc/drone-mc/environment`) | Firmware-enforced: PX4 drops `HIL_GPS` whose sysid differs (`ICD-MC-001.md` §3.3b). `MissionProtocolClient` targets the same ID. Must be unique per drone in a swarm. |

---

## 3. MC↔FCC uXRCE-DDS, serial (primary)

Jetson `ttyTHS1` ↔ Pixhawk TELEM2, 921600 8N1 (`HID-MC-001.md` §3). Used when `FCC_TRANSPORT=serial`, which is the production default.

| Parameter | Default | Value | Notes |
|---|---|---|---|
| `UXRCE_DDS_CFG` | 0 (disabled) | **102** (TELEM2) | Starts the uXRCE-DDS client on TELEM2. |
| `SER_TEL2_BAUD` | 921600 | **921600** | Already the TELEM2 default (`Tools/serial/generate_config.py`), but check it on FCCs set up for another TELEM2 device. The client's baud comes from the port's own `SER_*_BAUD`. There is no `UXRCE_DDS_BAUD` parameter in v1.17.0 (see §12). |
| `UXRCE_DDS_PTCFG` | 0 | **2** (custom `px4_participant`) | **Mandatory.** At 0, PX4 creates `/fmu/*` topics that the MC's CycloneDDS never discovers. Must match `docker/fcc-bridge/fastdds-agent.xml`. |
| `MAV_1_CONFIG` | 0 (disabled) | **0**, or any port other than TELEM2 | A MAVLink instance must not also claim TELEM2. Check this on FCCs previously set up for a companion over MAVLink. |
| `UXRCE_DDS_DOM_ID` | 0 | 0 | Leave at default. |

**Verify** (on the Jetson): `ros2 topic hz /fmu/out/vehicle_status_v1` gives about 2 Hz. If the Agent logs `topic created` but there are no `/fmu/*` topics, `UXRCE_DDS_PTCFG` is not 2. The Jetson side also needs the INFRA-009 device-tree overlay (`HID-MC-001.md` §3.3).

---

## 4. Ethernet: FCC address, Channel A, uXRCE-DDS fallback

### 4.1 FCC address (SD card, not a parameter)

`/fs/microsd/net.cfg`. The factory default is `192.168.0.3`. PX4/NuttX holds exactly **one** IPv4 address.

```
DEVICE=eth0
BOOTPROTO=static
IPADDR=<FCC_ETH_IP>        # default 192.168.100.31; orin-nx bench = 192.168.100.19
NETMASK=255.255.255.0
```

Reboot, then verify with `netman show`. `IPADDR` must equal the MC's `FCC_ETH_IP`.

### 4.2 Channel A (GCS ↔ FCC via the MC's mavlink-router, UDP 14550)

On the Pixhawk 6X, `boards/px4/fmu-v6x/init/rc.board_defaults` already sets up an Ethernet MAVLink instance on **`MAV_2`**. **Keep these board defaults:**

| Parameter | 6X board default | Notes |
|---|---|---|
| `MAV_2_CONFIG` | 1000 (Ethernet) | |
| `MAV_2_UDP_PRT` | 14550 | mavlink-router's `FCC_CHANNEL_A` endpoint sends here. |
| `MAV_2_REMOTE_PRT` | 14550 | |
| `MAV_2_MODE` | 0 (Normal) | |
| `MAV_2_BROADCAST` | 1 | |
| `MAV_2_RATE` | 100000 B/s | |

Don't configure a second Ethernet instance on 14550 via `MAV_0`. See §12 for why.

### 4.3 uXRCE-DDS over Ethernet (fallback, `FCC_TRANSPORT=ethernet`)

Only needed when you switch the DDS link off serial. `UXRCE_DDS_CFG` selects **one** port, so serial and Ethernet DDS are mutually exclusive.

| Parameter | Default | Value | Notes |
|---|---|---|---|
| `UXRCE_DDS_CFG` | 0 | **1000** (Ethernet) | Replaces 102 while in fallback. |
| `UXRCE_DDS_AG_IP` | 2130706433 (127.0.0.1) | **`MC_ETH_IP` as signed int32**: `192.168.100.11` → `-1062706165` | Recompute it for any other MC address: `python3 -c "import ipaddress,struct;print(struct.unpack('>i',ipaddress.ip_address('192.168.100.11').packed)[0])"` |
| `UXRCE_DDS_PRT` | 8888 | 8888 | Matches the Agent's `udp4 --port 8888`. |
| `UXRCE_DDS_PTCFG` | 0 | **2** | Same requirement as §3. |

---

## 5. MC↔FCC MAVLink exceptions (`extras.txt`)

These are three dedicated instances that run regardless of `FCC_TRANSPORT`, all to and from the FCC's single `FCC_ETH_IP`. They can't be expressed as `MAV_n_*` parameters: none of them can fix a destination IP, and `-t` exists only on the `mavlink start` command line. They therefore go in **`/fs/microsd/etc/extras.txt`**, which `rcS` sources on every boot. This is a **per-drone file**: `-t` must be *this* drone's `MC_ETH_IP`.

```
# /fs/microsd/etc/extras.txt — MC<->FCC MAVLink exceptions
# PER-DRONE: -t = this drone's MC_ETH_IP. Edit + reboot after any MC IP change.

# [Common] Mission protocol, MC <-> FCC (ICD-MC-001 §3.1). PROPOSED, see note below.
mavlink start -u 14555 -o 14555 -t 192.168.100.11 -m custom -r 10000

# [Common] Optical-flow + baro relay, FCC -> MC only (ICD-MC-001 §3.3a)
mavlink start -u 14556 -o 14556 -t 192.168.100.11 -m custom -r 10000
mavlink stream -u 14556 -s OPTICAL_FLOW_RAD -r 20
mavlink stream -u 14556 -s DISTANCE_SENSOR -r 10
mavlink stream -u 14556 -s ALTITUDE -r 10

# [Indoor] HIL_GPS receive, MC -> FCC only (ICD-MC-001 §3.3b)
mavlink start -u 14557 -m custom -r 1000
```

| Instance | Profile | Status | Notes |
|---|---|---|---|
| `:14555` mission | Common | **Proposed, not yet bench-verified.** No FCC has had this instance so far (`HID-MC-001.md` §10.6 "Known gap"; FT-006 findings). | `fcc_bridge` binds local 14555 and sends to `FCC_ETH_IP:14555`. A `custom`-mode instance still handles the mission microservice. Verify an `upload_mission()` round trip before relying on it. |
| `:14556` flow + baro | Common | Verified (FT-006 bench) | `ALTITUDE` is required even with mocked flow: it is `gnss_sim_node`'s vertical input. Raise `-r 20` if `vehicle_optical_flow` publishes faster (`HID-MC-001.md` §10.6 C). |
| `:14557` `HIL_GPS` | Indoor | Verified (FT-006 §3.2) | Inert outdoors as long as `MAV_USEHILGPS 0`, so it can stay in the file. |

**Verify:** `mavlink status` lists UDP 14555, 14556 and 14557. `mavlink status streams` shows the three relay streams. PX4 allows at most **6** MAVLink instances (`MAVLINK_COMM_NUM_BUFFERS`), shared with TELEM1, USB, Channel A and these three. Check the boot log if one is missing. On the MC, `ros2 topic hz /mc/sensors/baro_altitude` should report about 10 Hz.

---

## 6. DroneCAN: H-Flow optical flow + rangefinder

Holybro H-Flow on Pixhawk **CAN1** (`HID-MC-001.md` §10.4). These are common to both profiles: the flow and range sensors stay useful outdoors too.

| Parameter | Default | Value | Notes |
|---|---|---|---|
| `UAVCAN_ENABLE` | 0 | **2** (sensors, auto config) | Reboot. Don't use 3 unless the ESCs are DroneCAN. |
| `UAVCAN_SUB_FLOW` | 0 | **1** | Reboot. |
| `UAVCAN_SUB_RNG` | 0 | **1** | Reboot. |
| `UAVCAN_RNG_MIN` | 0.0 | **0.08** m | Sensor minimum. |
| `UAVCAN_RNG_MAX` | 999 | **30** m | |
| `SENS_FLOW_MINHGT` | 0.08 | 0.08 m | Already the default. |
| `SENS_FLOW_MAXHGT` | 100 | **30** m | |
| `SENS_FLOW_MAXR` | 8 | **7.4** rad/s | Sensor's maximum angular rate. |
| `SENS_FLOW_ROT` | 0 | **0** | Connectors toward the rear. A different mount needs a different value. |
| `EKF2_OF_CTRL` | 1 | 1 | Flow fusion on. See the open question below. |
| `EKF2_OF_POS_X/Y/Z` | 0 | **Measured per airframe** | Sensor offset from the CG, in metres. |
| `EKF2_RNG_CTRL` | 1 | 1 | Conditional range aid. |
| `EKF2_RNG_A_HMAX` | 5 | **10** m | Maximum allowed. |
| `EKF2_RNG_QLTY_T` | 1.0 | **0.2** s | |

**Open question (decide before TP-FT-002):** indoors, EKF2 fuses the H-Flow directly *and* fuses `HIL_GPS`, which `gnss_sim_node` dead-reckons from that same flow. The two aren't independent, but EKF2 treats them as if they were. Keeping `EKF2_OF_CTRL = 1` is an inherited default, not a reviewed decision.

**Verify:** `uavcan status` shows the H-Flow node. `listener sensor_optical_flow` shows quality > 0, and `listener distance_sensor` agrees with a tape measure.

---

## 7. GNSS source & EKF2

| Parameter | Default | Indoor | Outdoor | Notes |
|---|---|---|---|---|
| `MAV_USEHILGPS` | 0 | **1** | **0** | Indoor: accepts `gnss_sim_node`'s `HIL_GPS`. Leave it at 0 until FT-006's negative-path check has run on a new FCC. |
| `GPS_1_CONFIG` | GPS1 | GPS1, or **0** (optional) | GPS1 | Indoor 0 is defense-in-depth only, giving a single `sensor_gps` instance. The enforced interlock is IGS-005's real-GPS lockout on the MC. |
| `EKF2_GPS_CTRL` | 7 | **7** | **7** | **Never 0.** `HIL_GPS` reaches EKF2 through this same GNSS path, so 0 silently discards it. |
| `SENS_GPS_MASK` | 7 | 7 (record it) | 7 | Blending picks the better-accuracy instance. |
| `EKF2_HGT_REF` | 1 (GPS) | 1 | 1 | Indoor works because `HIL_GPS` altitude is built from the FCC baro (requires §8). |
| `EKF2_REQ_EPH` / `EKF2_REQ_NSATS` | 3.0 m / 6 | Default | Default | Initial-fix checks only. They pass right after anchoring (eph 1.0 m, 10 sats). |
| `EKF2_REQ_VDRIFT` | 0.2 m/s | Default | Default | Trips for about 20–30 s after an altitude-changing re-anchor. |
| `COM_POS_FS_EPH` | 5 m | Default | Default | Matches IGS-005's `max_eph_before_fault_m` (5.0 m). |

**Indoor flight window:** eph grows 0.02 m/s from 1.0 m, and IGS-005 latches at 5.0 m, about 200 s after the anchor. Arm, fly and land within about **150 s of anchoring**. Re-anchor on the ground only, and only inside the loaded mission's area.

---

## 8. Barometer

| Parameter | Default | Indoor | Outdoor | Notes |
|---|---|---|---|---|
| `SENS_BAR_AUTOCAL` | 1 | **0** | **1** | Indoor 1 creates a feedback loop. PX4 shifts the baro to match GPS altitude, but `HIL_GPS` altitude *is* the baro. On the bench (2026-10-02) this gave a +40 m height jump and `height estimate not stable`. After switching back to 1 outdoors, PX4 re-calibrates `CAL_BAROx_OFF` against real GNSS on the next boot. |

If autocal already fired this boot: `param set SENS_BAR_AUTOCAL 0`, `param save`, then re-anchor `gnss_sim_node`.

---

## 9. Geofence, RTL & failsafes

The indoor values are `HID-MC-001.md` §10.7 B: a starting point for a small room, to be fitted to the actual space and recorded in the test log. Outdoor values are **not yet defined in this repo**. TP-FT-005 requires a "conservative geofence" and a site risk assessment (`INTEGRATION-AND-FLIGHT-TEST-PLAN.md` §3), so fill them per site. The v1.17 defaults are listed as the baseline.

| Parameter | v1.17 default | Indoor | Outdoor | Notes |
|---|---|---|---|---|
| `GF_ACTION` | 2 (Hold) | **5** (Land) | Per site | |
| `GF_MAX_HOR_DIST` | 0 (off) | **8** m | Per site (set it, don't leave it off) | Measured from Home. Indoor: room half-width minus margin. |
| `GF_MAX_VER_DIST` | 0 (off) | **3** m | Per site / regulatory ceiling | Indoor: ceiling height minus margin. |
| `RTL_RETURN_ALT` | 60 m | **2** m | Per site (clear of obstacles) | Indoor: the stock value climbs into the ceiling. |
| `RTL_DESCEND_ALT` | 30 m | **1.5** m | Per site | |
| `RTL_TYPE` | 0 | 0 | 0 | Returns to the closer of Home or a rally point. |
| `NAV_RCL_ACT` | 2 (Return) | **3** (Land) | 2 (Return) | Loss of manual control. |
| `NAV_DLL_ACT` | 0 (off) | **3** (Land) | Per site (e.g. 2, Return) | Loss of the GCS data link. |
| `COM_DL_LOSS_T` | 10 s | Default | Default | Data-link loss timeout. |
| `COM_LOW_BAT_ACT` | 0 (Warning) | 0 | Per site | Setting 3 (Return) makes the `RTL_*` values above active on low battery. |
| `MIS_DIST_1WP` | 10000 m | **30** m | Per site | Warns when a mission is planned for somewhere else. Warning only. |

**Not a parameter, but still required:** an RC transmitter with a kill switch bound and tested, a props-off arm/disarm check first, and a tether or cage for the first indoor sorties.

---

## 10. Flight envelope (mission speeds & takeoff)

| Parameter | v1.17 default | Indoor | Outdoor |
|---|---|---|---|
| `MIS_TAKEOFF_ALT` | 2.5 m | **1.5** m | Per site |
| `MPC_XY_CRUISE` | 5 m/s | **1.0** m/s | Per site (default is a reasonable start) |
| `MPC_XY_VEL_MAX` | 12 m/s | **2.0** m/s | Per site |
| `MPC_Z_VEL_MAX_UP` | 3 m/s | **1.0** m/s | Default |
| `MPC_Z_VEL_MAX_DN` | 1.5 m/s | **0.7** m/s | Default |

---

## 11. Power monitoring

| Parameter | Value | Notes |
|---|---|---|
| `BAT1_V_DIV` | From the baseboard silkscreen or Holybro docs | `HID-MC-001.md` §7 (HIF-005). Calibrate once per airframe. |
| `BAT1_A_PER_V` | From the baseboard silkscreen or Holybro docs | |

---

## 12. Corrections made to other docs

These errors turned up while compiling this guide against the v1.17.0 source. All were fixed on 2026-10-20. They're listed here so anyone holding an older copy, or an FCC set up from one, knows what to recheck.

| Where | Used to say | Corrected to |
|---|---|---|
| `HID-MC-001.md` §3.3 and §13.2, `FT-006-BENCH-RUNBOOK.md` §1.2 | `UXRCE_DDS_BAUD = 921600` | `SER_TEL2_BAUD = 921600`. There is no `UXRCE_DDS_BAUD`. This closes the INFRA-009 HIF-001 doc item in `PROJECT-PLAN-5-MONTH.md`. |
| `HID-MC-001.md` §3.3 and §13.2 | (omitted) | `MAV_1_CONFIG = 0`: no MAVLink instance may also claim TELEM2. |
| `HID-MC-001.md` §3.1 and §3.3 | Agent launched with `--rts-cts`; RTS/CTS flow control | No flow control: PX4 clears `CRTSCTS` on TELEM2, matching `docker/fcc-bridge/entrypoint.sh`. |
| `HID-MC-001.md` §4.2 and §13.2 | `UXRCE_DDS_PORT_CFG = 8888` | `UXRCE_DDS_PRT = 8888`. |
| `HID-MC-001.md` §4.2 | `MAV_0_CONFIG = 1000 (Ethernet)` | On the 6X the Ethernet MAVLink instance is `MAV_2` by board default. `MAV_0` stays on TELEM1. **An FCC configured from the old text may have two instances on 14550: check `mavlink status`.** |
| `HID-MC-001.md` §4.2 | Ethernet fallback set only `UXRCE_DDS_AG_IP` | It also needs `UXRCE_DDS_CFG = 1000`. |
| `OFFBOARD-MODE-SWITCHING-BACKLOG.md` | `COM_OBL_ACT` / `COM_OBL_RT_ACT` | `COM_OBL_RC_ACT` (action) and `COM_OF_LOSS_T` (timeout). See §14. |

---

## 13. Switching profiles

### Outdoor → Indoor (before FT-006 / TP-FT-002)

```
param set MAV_USEHILGPS 1          # leave at 0 until FT-006 §3.1's negative path has run on this FCC
param set SENS_BAR_AUTOCAL 0
# optional: param set GPS_1_CONFIG 0
# §9 + §10 indoor values (GF_*, RTL_*, NAV_RCL_ACT/NAV_DLL_ACT, MIS_*, MPC_*)
param save
reboot
```

Then check that `extras.txt` has the `:14557` block, re-anchor `gnss_sim_node` (the anchor does not survive an FCC reboot), and fly within about 150 s of the anchor.

### Indoor → Outdoor (before TP-FT-005)

```
param set MAV_USEHILGPS 0
param set SENS_BAR_AUTOCAL 1       # re-calibrates CAL_BAROx_OFF against real GNSS next boot
param set GPS_1_CONFIG <GPS1>      # if it was set to 0 indoors (value per QGC's enum)
# §9 + §10 site-specific outdoor values
param save
reboot
```

Stop `drone-mc-indoor-nav` on the MC (`sudo systemctl stop drone-mc-indoor-nav`).

### Record every session

Save `param show -c` and `cat /fs/microsd/etc/extras.txt` with the test log. The SD card and parameter state are per-drone, and nothing in this repo deploys them.

---

## 14. Future: OFFBOARD mode

This is not used yet: all flight stages through TP-FT-006 are MISSION-only. When Phase 6 (`OFFBOARD-MODE-SWITCHING-BACKLOG.md`) lands, PX4's own OFFBOARD-loss failsafe is:

| Parameter | v1.17 default | Notes |
|---|---|---|
| `COM_OF_LOSS_T` | 1.0 s | Setpoint-stream loss timeout. The MC's own `SFR-009` stale-stream fault fires at 500 ms, before this. |
| `COM_OBL_RC_ACT` | 0 (Position) | Action on loss. Options: 0 Position, 1 Altitude, 2 Stabilized, 3 Return, 4 Land, 5 Hold, 6 Terminate, 7 Disarm. Pick a value per the backlog's design. |
