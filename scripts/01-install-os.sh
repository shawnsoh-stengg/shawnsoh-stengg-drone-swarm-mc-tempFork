#!/usr/bin/env bash
# 01-install-os.sh — Base OS configuration on the Jetson: eth0 Netplan
# (HIF-002/HIF-004), the ttyTHS1 udev rule (HIF-001), the CycloneDDS
# socket buffer sysctl floor (INFRA-004), and the ttyTHS1 PIO device-tree
# overlay + extlinux boot entry (INFRA-009 — needs a reboot to take effect).
# Companion smoke test: 01b-smoke-test-os.sh
#
# Prerequisite: the Jetson module has already been flashed with JetPack 6.2.2
# via NVIDIA SDK Manager (HID-MC-001 §8.1) and boots to a login prompt.
# This script runs directly on the Jetson host (not inside Docker) — it
# configures files under /etc that must exist before the drone-mc containers
# ever start.
#
# Usage:
#   sudo ./01-install-os.sh
#
# Set ALLOW_NON_JETSON=1 to skip the aarch64/Ubuntu 22.04 precondition check
# (useful when dry-running the netplan/udev install on a dev workstation).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "${SCRIPT_DIR}")"

if [[ "${EUID}" -ne 0 ]]; then
    echo "ERROR: must run as root (sudo ./01-install-os.sh)" >&2
    exit 1
fi

if [[ "${ALLOW_NON_JETSON:-0}" -ne 1 ]]; then
    if [[ "$(uname -m)" != "aarch64" ]]; then
        echo "ERROR: expected aarch64 (Jetson), got $(uname -m). Set ALLOW_NON_JETSON=1 to override." >&2
        exit 1
    fi
    if ! grep -q 'VERSION_ID="22.04"' /etc/os-release 2>/dev/null; then
        echo "ERROR: expected Ubuntu 22.04. Set ALLOW_NON_JETSON=1 to override." >&2
        exit 1
    fi
fi

if ! command -v netplan &>/dev/null; then
    echo "==> netplan not found, installing netplan.io"
    if ! timeout 10 getent hosts archive.ubuntu.com &>/dev/null; then
        echo "ERROR: cannot resolve archive.ubuntu.com — check network connectivity" \
             "(eth0/wifi must be online before this script can fetch netplan.io) and retry." >&2
        exit 1
    fi
    export DEBIAN_FRONTEND=noninteractive
    apt-get update -o Acquire::Retries=2 -o Acquire::http::Timeout=10 -o Acquire::https::Timeout=10
    apt-get install -y --no-install-recommends netplan.io
fi

if ! command -v envsubst &>/dev/null; then
    echo "==> envsubst not found, installing gettext-base"
    if ! timeout 10 getent hosts archive.ubuntu.com &>/dev/null; then
        echo "ERROR: cannot resolve archive.ubuntu.com — check network connectivity" \
             "and retry." >&2
        exit 1
    fi
    export DEBIAN_FRONTEND=noninteractive
    apt-get update -o Acquire::Retries=2 -o Acquire::http::Timeout=10 -o Acquire::https::Timeout=10
    apt-get install -y --no-install-recommends gettext-base
fi

# INFRA-008: config/netplan/01-drone-mc.yaml is an envsubst template
# (MC_ETH_MAC/MC_ETH_IP) — /etc/drone-mc/environment is
# this drone's source of truth for those values, so it must exist before
# rendering. This mirrors 02-install-systemd-units.sh's own idempotent
# provisioning of the same file: install the repo default only if this
# drone has no per-drone copy yet, never overwrite an existing one.
echo "==> Provisioning /etc/drone-mc/environment (for netplan template rendering)"
mkdir -p /etc/drone-mc
if [[ -f /etc/drone-mc/environment ]]; then
    echo "    /etc/drone-mc/environment already exists — leaving per-drone values untouched."
else
    install -m 644 "${REPO_ROOT}/config/environment" /etc/drone-mc/environment
    echo "    Installed default environment. Edit DRONE_ID/MC_ETH_MAC/MC_ETH_IP/" \
         "FCC_ETH_IP/FCC_TRANSPORT for this vehicle, then re-run this" \
         "script to render netplan with the real values."
fi

# Single-IP FCC migration (2026-09-25): the Pixhawk's NuttX stack has
# exactly one IPv4 address, so the old dual-homed layout (FCC_ETH_IP on a
# 192.168.12.x internal subnet + FCC_GCS_IP on 192.168.10.x, and the MC's
# own second MC_ETH_INTERNAL_IP) was never real. FCC_ETH_IP is now the one
# FCC address. Fold an on-device FCC_GCS_IP into a still-192.168.12.x
# FCC_ETH_IP and drop both retired variables — before the backfill below,
# so it can't re-add them. Refuses to guess if the two disagree otherwise.
ENV_FILE=/etc/drone-mc/environment
old_fcc_eth="$(grep -E '^FCC_ETH_IP=' "${ENV_FILE}" | tail -n1 | cut -d= -f2- || true)"
old_fcc_gcs="$(grep -E '^FCC_GCS_IP=' "${ENV_FILE}" | tail -n1 | cut -d= -f2- || true)"
if [[ -n "${old_fcc_gcs}" ]]; then
    if [[ -z "${old_fcc_eth}" || "${old_fcc_eth}" == 192.168.12.* ]]; then
        sed -i '/^FCC_ETH_IP=/d' "${ENV_FILE}"
        echo "FCC_ETH_IP=${old_fcc_gcs}" >> "${ENV_FILE}"
        echo "    Migrated FCC_ETH_IP ${old_fcc_eth:-<unset>} -> ${old_fcc_gcs} (was FCC_GCS_IP; single-IP FCC)"
    elif [[ "${old_fcc_eth}" != "${old_fcc_gcs}" ]]; then
        echo "ERROR: ${ENV_FILE} has FCC_ETH_IP=${old_fcc_eth} and FCC_GCS_IP=${old_fcc_gcs}" \
             "— the FCC has one address now (FCC_ETH_IP). Set FCC_ETH_IP to the Pixhawk's" \
             "real address (\`netman show\`), delete FCC_GCS_IP, and re-run." >&2
        exit 1
    fi
    sed -i '/^FCC_GCS_IP=/d' "${ENV_FILE}"
    echo "    Retired FCC_GCS_IP from ${ENV_FILE}"
fi
if grep -qE '^FCC_ETH_IP=192\.168\.12\.' "${ENV_FILE}"; then
    echo "ERROR: ${ENV_FILE}'s FCC_ETH_IP is still on the retired 192.168.12.x subnet and" \
         "there's no FCC_GCS_IP to migrate from. Set it to the Pixhawk's real address" \
         "(\`netman show\` in the MAVLink console) and re-run." >&2
    exit 1
fi
if grep -q '^MC_ETH_INTERNAL_IP=' "${ENV_FILE}"; then
    sed -i '/^MC_ETH_INTERNAL_IP=/d' "${ENV_FILE}"
    echo "    Retired MC_ETH_INTERNAL_IP from ${ENV_FILE} (eth0 has a single address now)"
fi

# Subnet move (2026-10-06): 192.168.10.x collided with the AI.R hangar
# Wi-Fi, so the whole drone network moved to 192.168.100.x with every
# host octet unchanged (HID-MC-001 §4.2). An already-provisioned drone's
# per-drone values never come from the repo default again, so rewrite any
# 192.168.10.N on-device value to 192.168.100.N here — before the backfill
# below. The Pixhawk (net.cfg IPADDR, UXRCE_DDS_AG_IP), MANET radios,
# gimbal camera and GCS laptop must be moved too; this can't reach them.
if grep -qE '^[A-Za-z_][A-Za-z0-9_]*=.*\b192\.168\.10\.[0-9]' "${ENV_FILE}"; then
    while IFS= read -r line; do
        echo "    Migrating ${line} -> ${line//192.168.10./192.168.100.}"
    done < <(grep -E '^[A-Za-z_][A-Za-z0-9_]*=.*\b192\.168\.10\.[0-9]' "${ENV_FILE}")
    sed -i -E '/^[A-Za-z_][A-Za-z0-9_]*=/ s/\b192\.168\.10\.([0-9])/192.168.100.\1/g' "${ENV_FILE}"
    echo "    NOTE: this drone's Pixhawk (/fs/microsd/net.cfg IPADDR, UXRCE_DDS_AG_IP)," \
         "MANET radios, gimbal camera and GCS laptop must also move to 192.168.100.x" \
         "(docs/DEPLOYMENT.md, \"Moving the drone subnet\")."
fi

# Per-variable migration: the file-level check above only seeds
# /etc/drone-mc/environment once, so a repo change to config/environment
# (e.g. INFRA-008 adding MC_ETH_MAC/MC_ETH_IP)
# never reaches an already-provisioned Jetson on its own. Backfill any
# variable present in the repo default but missing on-device, without
# touching existing per-drone values.
while IFS='=' read -r key value; do
    if ! grep -q "^${key}=" /etc/drone-mc/environment; then
        echo "${key}=${value}" >> /etc/drone-mc/environment
        echo "    Migrated new variable ${key}=${value} into /etc/drone-mc/environment" \
             "(review/edit this per-drone value if it needs to differ from the repo default)."
    fi
done < <(grep -E '^[A-Za-z_][A-Za-z0-9_]*=' "${REPO_ROOT}/config/environment")

echo "==> Rendering /etc/netplan/01-drone-mc.yaml (HIF-002/HIF-004 eth0)"
set -a
# shellcheck disable=SC1091
source /etc/drone-mc/environment
set +a
for var in MC_ETH_MAC MC_ETH_IP; do
    if [[ -z "${!var:-}" ]]; then
        echo "ERROR: ${var} is unset/empty in /etc/drone-mc/environment — envsubst would" \
             "silently render a blank value into netplan. Set it and re-run." >&2
        exit 1
    fi
done

# INFRA-008: MC_ETH_MAC must match this drone's own NIC, not a stale/copied
# value from another unit's provisioning (config/environment's repo default
# is jetson-nx-001's MAC). A mismatch means netplan's `match: macaddress:`
# matches zero interfaces — set-name never fires, eth0 never gets its
# addresses, and netplan's own duplicate-match resolution can hit an
# internal GLib assertion (g_ascii_strcasecmp on a NULL MAC) rather than
# failing cleanly. Catch it here instead, with the local MACs to compare
# against. Skippable with ALLOW_NON_JETSON=1 for a dev-workstation dry run,
# same escape hatch as the aarch64/Ubuntu precondition check above.
if [[ "${ALLOW_NON_JETSON:-0}" -ne 1 ]]; then
    mac_found=0
    for addr_file in /sys/class/net/*/address; do
        iface="$(basename "$(dirname "${addr_file}")")"
        [[ "${iface}" == "lo" ]] && continue
        if [[ "$(cat "${addr_file}")" == "${MC_ETH_MAC,,}" ]]; then
            mac_found=1
            break
        fi
    done
    if [[ "${mac_found}" -ne 1 ]]; then
        echo "ERROR: MC_ETH_MAC=${MC_ETH_MAC} in /etc/drone-mc/environment does not match" \
             "any local interface's MAC — netplan's macaddress match would select" \
             "nothing, eth0 would never get its addresses, and netplan apply may fail" \
             "with an opaque internal error instead of a clean one. Local interfaces:" >&2
        for addr_file in /sys/class/net/*/address; do
            iface="$(basename "$(dirname "${addr_file}")")"
            [[ "${iface}" == "lo" ]] && continue
            echo "    ${iface}: $(cat "${addr_file}")" >&2
        done
        echo "Set MC_ETH_MAC to this drone's real NIC MAC (usually the wired interface" \
             "above) and re-run. Set ALLOW_NON_JETSON=1 to skip this check for a" \
             "dev-workstation dry run." >&2
        exit 1
    fi

    # Other network config claiming the same NIC: netplan apply only adds
    # 01-drone-mc.yaml's address, it never removes ones another source
    # (a hand-made NetworkManager profile, a second netplan file) puts on
    # the same interface — observed on a bench unit as eth0 carrying a
    # stale 192.168.10.11 + 192.168.12.2 pair from an NM profile alongside
    # MC_ETH_IP, failing 01b's single-address check. Warn rather than
    # delete: the other config may be what this SSH session rides on.
    mc_iface="${iface}"
    claimants=()
    for f in /etc/netplan/*.yaml; do
        [[ -f "${f}" && "${f}" != /etc/netplan/01-drone-mc.yaml ]] || continue
        if grep -qiF "${MC_ETH_MAC}" "${f}" ||
           grep -qE "^[[:space:]]*(eth0|${mc_iface}):|name:[[:space:]]*\"?(eth0|${mc_iface})\"?[[:space:]]*$" "${f}"; then
            claimants+=("netplan file ${f}")
        fi
    done
    if command -v nmcli &>/dev/null && nmcli -t general status &>/dev/null; then
        while IFS=: read -r nm_name nm_device nm_file; do
            # netplan's own NM rendering of 01-drone-mc.yaml, not a rival.
            [[ "${nm_file}" == */netplan-* ]] && continue
            if [[ "${nm_device}" == "${mc_iface}" || "${nm_device}" == eth0 ]] ||
               { [[ -f "${nm_file}" ]] &&
                 grep -qiE "^(mac-address=${MC_ETH_MAC}|interface-name=(eth0|${mc_iface}))$" "${nm_file}"; }; then
                claimants+=("NetworkManager profile '${nm_name}' (${nm_file:-in-memory})")
            fi
        done < <(nmcli -t -f NAME,DEVICE,FILENAME connection show 2>/dev/null || true)
    fi
    if [[ "${#claimants[@]}" -gt 0 ]]; then
        echo "WARNING: other network config also claims ${mc_iface} (MC_ETH_MAC=${MC_ETH_MAC})" \
             "— its addresses will sit alongside MC_ETH_IP=${MC_ETH_IP} and fail" \
             "01b-smoke-test-os.sh's single-address check:" >&2
        for c in "${claimants[@]}"; do echo "    ${c}" >&2; done
        echo "    Remove it (\`nmcli connection delete <name>\` / delete the extra netplan" \
             "file), then \`ip addr flush dev ${mc_iface}\` + re-run, or reboot. Do this from" \
             "the console if this session is on ${mc_iface}." >&2
    fi
fi

# Optional default route + DNS (bench internet). Both empty is the flight
# default: no route out, no resolver. envsubst has no conditionals, so
# the routes block is built here and substituted whole (or as nothing).
ipv4_re='^([0-9]{1,3}\.){3}[0-9]{1,3}$'
MC_ETH_GATEWAY="${MC_ETH_GATEWAY:-}"
MC_ETH_DNS="${MC_ETH_DNS:-}"
if [[ -n "${MC_ETH_GATEWAY}" && ! "${MC_ETH_GATEWAY}" =~ ${ipv4_re} ]]; then
    echo "ERROR: MC_ETH_GATEWAY=${MC_ETH_GATEWAY} in /etc/drone-mc/environment is not" \
         "an IPv4 address. Fix or empty it and re-run." >&2
    exit 1
fi
MC_ETH_DNS_LIST=""
if [[ -n "${MC_ETH_DNS}" ]]; then
    IFS=',' read -ra dns_servers <<<"${MC_ETH_DNS// /}"
    for dns in "${dns_servers[@]}"; do
        [[ -z "${dns}" ]] && continue
        if [[ ! "${dns}" =~ ${ipv4_re} ]]; then
            echo "ERROR: MC_ETH_DNS entry '${dns}' in /etc/drone-mc/environment is not" \
                 "an IPv4 address (expected a comma-separated list). Fix and re-run." >&2
            exit 1
        fi
        MC_ETH_DNS_LIST+="${MC_ETH_DNS_LIST:+, }${dns}"
    done
fi
MC_ETH_ROUTES_YAML=""
if [[ -n "${MC_ETH_GATEWAY}" ]]; then
    MC_ETH_ROUTES_YAML="      routes:
        - to: default
          via: ${MC_ETH_GATEWAY}"
    echo "    eth0 default route via ${MC_ETH_GATEWAY}, DNS [${MC_ETH_DNS_LIST:-none}]"
elif [[ -n "${MC_ETH_DNS_LIST}" ]]; then
    echo "    NOTE: MC_ETH_DNS is set but MC_ETH_GATEWAY is empty — names will only" \
         "resolve if the DNS server is on 192.168.100.x itself."
fi
export MC_ETH_DNS_LIST MC_ETH_ROUTES_YAML

RENDERED_NETPLAN="$(mktemp)"
trap 'rm -f "${RENDERED_NETPLAN}"' EXIT
envsubst '$MC_ETH_MAC $MC_ETH_IP $MC_ETH_DNS_LIST $MC_ETH_ROUTES_YAML' \
    < "${REPO_ROOT}/config/netplan/01-drone-mc.yaml" > "${RENDERED_NETPLAN}"
install -m 600 "${RENDERED_NETPLAN}" /etc/netplan/01-drone-mc.yaml
if [[ -n "${SSH_CONNECTION:-}" ]] &&
   ! ip -4 -o addr show 2>/dev/null | grep -qF " ${MC_ETH_IP}/"; then
    echo "    NOTE: eth0 is moving to MC_ETH_IP=${MC_ETH_IP} — this SSH session will drop" \
         "if it rides the old address. Reconnect on ${MC_ETH_IP} (or use the console)."
fi
netplan apply

# Even with no rival config left, netplan apply doesn't drop addresses a
# since-deleted profile already put on the live interface — name any
# that remain so the 01b failure isn't a surprise.
if [[ -n "${mc_iface:-}" ]]; then
    live_iface=eth0
    ip link show eth0 &>/dev/null || live_iface="${mc_iface}"
    stale_addrs="$(ip -4 -o addr show dev "${live_iface}" 2>/dev/null |
        awk '{print $4}' | grep -vxF "${MC_ETH_IP}/24" || true)"
    if [[ -n "${stale_addrs}" ]]; then
        echo "WARNING: ${live_iface} still carries address(es) other than MC_ETH_IP=${MC_ETH_IP}:" \
             "${stale_addrs//$'\n'/ } — reboot or \`ip addr flush dev ${live_iface}\` +" \
             "\`netplan apply\` to clear them." >&2
    fi
fi

echo "==> Installing /etc/udev/rules.d/99-drone-mc.rules (HIF-001 ttyTHS1 permissions)"
install -m 644 "${REPO_ROOT}/config/udev/99-drone-mc.rules" /etc/udev/rules.d/99-drone-mc.rules
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty

echo "==> Installing /etc/sysctl.d/99-drone-mc.conf (CycloneDDS socket buffer floor)"
install -m 644 "${REPO_ROOT}/config/sysctl/99-drone-mc.conf" /etc/sysctl.d/99-drone-mc.conf
sysctl --system &>/dev/null

# INFRA-009: ttyTHS1 RX DMA zeroes every 16-byte burst on this baseboard, so
# the uXRCE-DDS Agent never gets a session — the overlay switches the UART to
# PIO (root cause in config/dt/ttyTHS1-rx-pio.dts). Installs the .dtbo plus a
# new extlinux `LABEL rxpio` cloned from the current DEFAULT entry, which
# stays in the boot menu as the fallback. Takes effect on the next reboot.
# Skipped under ALLOW_NON_JETSON=1: a dev workstation's /boot isn't L4T's.
TTYTHS1_DT_PATH=/bus@0/serial@3100000
TTYTHS1_DTBO=/boot/ttyTHS1-rx-pio.dtbo
EXTLINUX_CONF=/boot/extlinux/extlinux.conf
ttyths1_dma_names() {
    tr '\0' ' ' < "/proc/device-tree${TTYTHS1_DT_PATH}/dma-names" 2>/dev/null | sed 's/ $//'
}
if [[ "${ALLOW_NON_JETSON:-0}" -eq 1 ]]; then
    echo "==> Skipping ttyTHS1 PIO overlay (ALLOW_NON_JETSON=1)"
else
    echo "==> Installing ${TTYTHS1_DTBO} (INFRA-009 ttyTHS1 RX/TX PIO overlay)"
    # The overlay targets the controller by DT path, so confirm ttyTHS1
    # really is that controller before pointing a boot entry at it.
    ttyths1_dev="$(basename "$(readlink -f /sys/class/tty/ttyTHS1/device 2>/dev/null)")"
    if [[ "${ttyths1_dev}" != "${TTYTHS1_DT_PATH##*@}.serial" ]]; then
        echo "ERROR: /dev/ttyTHS1 is backed by '${ttyths1_dev:-nothing}', not" \
             "${TTYTHS1_DT_PATH##*@}.serial (${TTYTHS1_DT_PATH}) — the overlay would" \
             "change the wrong UART. Check the carrier board/L4T release (HID-MC-001 §3)." >&2
        exit 1
    fi
    if ! command -v dtc &>/dev/null; then
        echo "==> dtc not found, installing device-tree-compiler"
        if ! timeout 10 getent hosts archive.ubuntu.com &>/dev/null; then
            echo "ERROR: cannot resolve archive.ubuntu.com — check network connectivity" \
                 "and retry." >&2
            exit 1
        fi
        export DEBIAN_FRONTEND=noninteractive
        apt-get update -o Acquire::Retries=2 -o Acquire::http::Timeout=10 -o Acquire::https::Timeout=10
        apt-get install -y --no-install-recommends device-tree-compiler
    fi

    JETSON_DT_COMPATIBLE="$(tr '\0' '\n' < /proc/device-tree/compatible | head -n1)"
    if [[ -z "${JETSON_DT_COMPATIBLE}" ]]; then
        echo "ERROR: /proc/device-tree/compatible is empty — can't match the overlay to this board." >&2
        exit 1
    fi
    export JETSON_DT_COMPATIBLE
    RENDERED_DTBO="$(mktemp)"
    trap 'rm -f "${RENDERED_NETPLAN}" "${RENDERED_DTBO}"' EXIT
    envsubst '$JETSON_DT_COMPATIBLE' < "${REPO_ROOT}/config/dt/ttyTHS1-rx-pio.dts" |
        dtc -@ -q -I dts -O dtb -o "${RENDERED_DTBO}" -
    if cmp -s "${RENDERED_DTBO}" "${TTYTHS1_DTBO}"; then
        echo "    ${TTYTHS1_DTBO} already up to date (compatible ${JETSON_DT_COMPATIBLE})"
    else
        install -m 644 "${RENDERED_DTBO}" "${TTYTHS1_DTBO}"
        echo "    Installed ${TTYTHS1_DTBO} (compatible ${JETSON_DT_COMPATIBLE})"
    fi

    if [[ ! -f "${EXTLINUX_CONF}" ]]; then
        echo "ERROR: ${EXTLINUX_CONF} not found — expected L4T's extlinux boot config." >&2
        exit 1
    fi
    extlinux_entry() {
        awk -v l="$1" '/^[[:space:]]*LABEL[[:space:]]/{in_e=($2==l)} in_e' "${EXTLINUX_CONF}"
    }
    base_label="$(awk '/^[[:space:]]*DEFAULT[[:space:]]/{print $2; exit}' "${EXTLINUX_CONF}")"
    base_entry="$(extlinux_entry "${base_label}")"
    if grep -qE "^[[:space:]]*OVERLAYS[[:space:]].*${TTYTHS1_DTBO}" <<<"${base_entry}"; then
        # DEFAULT already loads it — LABEL rxpio itself, or an entry that
        # loads other overlays too, merged by hand per the error below.
        echo "    DEFAULT entry '${base_label}' already loads ${TTYTHS1_DTBO}"
    elif grep -qE '^[[:space:]]*LABEL[[:space:]]+rxpio[[:space:]]*$' "${EXTLINUX_CONF}"; then
        # Already added (by a previous run, or by hand on orin-nx 2026-09-24
        # with the same label/path) — just confirm it loads this overlay.
        if ! extlinux_entry rxpio | grep -qE "^[[:space:]]*OVERLAYS[[:space:]].*${TTYTHS1_DTBO}"; then
            echo "ERROR: ${EXTLINUX_CONF} has a LABEL rxpio entry without" \
                 "'OVERLAYS ${TTYTHS1_DTBO}'. Fix or delete that entry and re-run." >&2
            exit 1
        fi
        echo "    ${EXTLINUX_CONF} already has LABEL rxpio"
        [[ -f "${EXTLINUX_CONF}.pre-infra-009" ]] ||
            cp -p "${EXTLINUX_CONF}" "${EXTLINUX_CONF}.pre-infra-009"
        sed -i -E 's/^([[:space:]]*DEFAULT[[:space:]]+).*/\1rxpio/' "${EXTLINUX_CONF}"
        echo "    Set DEFAULT rxpio in ${EXTLINUX_CONF} (was '${base_label}')"
    else
        if [[ -z "${base_entry}" ]]; then
            echo "ERROR: ${EXTLINUX_CONF}'s DEFAULT '${base_label}' has no LABEL entry to clone." >&2
            exit 1
        fi
        # A DEFAULT that already loads overlays (e.g. a Jetson-IO entry for
        # HIF-003's camera) needs both merged by hand, not silently dropped.
        if grep -qE '^[[:space:]]*OVERLAYS[[:space:]]' <<<"${base_entry}"; then
            echo "ERROR: DEFAULT entry '${base_label}' in ${EXTLINUX_CONF} already loads" \
                 "overlays — add ${TTYTHS1_DTBO} to it by hand (HID-MC-001 §3.3), then re-run." >&2
            exit 1
        fi
        fdt_line="$(grep -E '^[[:space:]]*FDT[[:space:]]' <<<"${base_entry}" || true)"
        if [[ -z "${fdt_line}" ]]; then
            # L4T's stock entry omits FDT (UEFI passes its own DTB), but
            # extlinux OVERLAYS only apply on top of an explicit FDT.
            dtbs=(/boot/dtb/kernel_*.dtb)
            if [[ "${#dtbs[@]}" -ne 1 || ! -f "${dtbs[0]}" ]]; then
                echo "ERROR: expected exactly one /boot/dtb/kernel_*.dtb for the rxpio entry's" \
                     "FDT line, found: ${dtbs[*]}" >&2
                exit 1
            fi
            fdt_line="      FDT ${dtbs[0]}"
        fi
        [[ -f "${EXTLINUX_CONF}.pre-infra-009" ]] ||
            cp -p "${EXTLINUX_CONF}" "${EXTLINUX_CONF}.pre-infra-009"
        {
            echo
            echo "LABEL rxpio"
            echo "      MENU LABEL ttyTHS1 RX/TX PIO (drone-mc INFRA-009)"
            grep -E '^[[:space:]]*(LINUX|INITRD)[[:space:]]' <<<"${base_entry}"
            echo "${fdt_line}"
            grep -E '^[[:space:]]*APPEND[[:space:]]' <<<"${base_entry}"
            echo "      OVERLAYS ${TTYTHS1_DTBO}"
        } >> "${EXTLINUX_CONF}"
        sed -i -E 's/^([[:space:]]*DEFAULT[[:space:]]+).*/\1rxpio/' "${EXTLINUX_CONF}"
        echo "    Added LABEL rxpio as DEFAULT (cloned from '${base_label}', which stays" \
             "as the boot-menu fallback; backup at ${EXTLINUX_CONF}.pre-infra-009)"
    fi
    if [[ "$(ttyths1_dma_names)" != "rx-off tx-off" ]]; then
        echo "    NOTE: reboot to load the overlay (live dma-names:" \
             "'$(ttyths1_dma_names)'), then run 01b-smoke-test-os.sh. If it fails to boot," \
             "pick the previous entry in the extlinux menu (HDMI/debug UART);" \
             "${EXTLINUX_CONF}.pre-infra-009 has the pre-overlay config if this script" \
             "made the change."
    fi
fi

TARGET_USER="${SUDO_USER:-${USER}}"
if ! id -nG "${TARGET_USER}" | grep -qw dialout; then
    echo "==> Adding ${TARGET_USER} to the dialout group"
    usermod -aG dialout "${TARGET_USER}"
    echo "    NOTE: ${TARGET_USER} must log out and back in for the group change to take effect."
fi

echo "==> OS configuration complete"
if ip -4 addr show eth0 &>/dev/null; then
    ip -4 addr show eth0 | grep inet || true
else
    echo "    NOTE: eth0 not present yet — the netplan match/set-name rename of the" \
         "physical NIC (e.g. enP8p1s0) to eth0 can require a reboot to take effect" \
         "if the interface was already up. Reboot and re-run 01b-smoke-test-os.sh to confirm."
fi
ls -l /dev/ttyTHS1 2>/dev/null || echo "    NOTE: /dev/ttyTHS1 not present yet — expected before the ttyTHS1<->Pixhawk link is wired up."
