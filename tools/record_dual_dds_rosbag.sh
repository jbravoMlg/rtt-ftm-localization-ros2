#!/usr/bin/env bash
set -Eeuo pipefail

# Record a complete ROS 2 capture from both DDS stacks used in this project.
#
# Why two bags?
#   A ros2 process can use only one RMW_IMPLEMENTATION at a time. If Android/FastDDS
#   topics and desktop/CycloneDDS topics are not mutually discovered on the VPN,
#   one single `ros2 bag record` process can silently miss one side. This script
#   starts two recorders in parallel and stores them under one session directory.
#
# Output layout:
#   ~/.local/share/rtt-searchagent/rosbags/dual_dds/dual_dds_YYYY_MM_DD-HH_MM_SS/
#     manifest.env
#     topics_fastdds_before.txt
#     topics_cyclonedds_before.txt
#     graph_fastdds_before.txt
#     graph_cyclonedds_before.txt
#     fastdds/rosbag2_fastdds_YYYY_MM_DD-HH_MM_SS/
#     cyclonedds/rosbag2_cyclonedds_YYYY_MM_DD-HH_MM_SS/

usage() {
    cat <<'EOF'
Usage:
  bash tools/record_dual_dds_rosbag.sh [options]

Options:
  -o, --output-root DIR       Root directory for session folders.
                                                            Default: $RTT_GUI_DATA_ROOT/rosbags/dual_dds,
                                                            or ~/.local/share/rtt-searchagent/rosbags/dual_dds
  -d, --duration SEC          Stop automatically after SEC seconds.
                              Default: record until Ctrl+C.
  -s, --storage STORAGE       rosbag2 storage backend: sqlite3 or mcap.
                              Default: sqlite3
  --domain ID                 ROS_DOMAIN_ID for both recorders.
                              Default: current ROS_DOMAIN_ID or 0.
  --fast-rmw NAME             Fast DDS RMW implementation.
                              Default: rmw_fastrtps_cpp
  --cyclone-rmw NAME          Cyclone DDS RMW implementation.
                              Default: rmw_cyclonedds_cpp
  --cyclonedds-uri URI        CYCLONEDDS_URI for the Cyclone recorder.
                              Default: current CYCLONEDDS_URI, if set.
  --fastrtps-profile FILE     FASTRTPS_DEFAULT_PROFILES_FILE for the FastDDS recorder.
                              Default: current FASTRTPS_DEFAULT_PROFILES_FILE, if set.
  --no-hidden                 Do not include hidden topics.
  --split-size BYTES          Split each bag when it reaches BYTES.
  --topic TOPIC               Record one topic. May be repeated. If omitted, records all.
  --                          Extra arguments passed to both `ros2 bag record` commands.

Examples:
  bash tools/record_dual_dds_rosbag.sh
  bash tools/record_dual_dds_rosbag.sh --duration 300 --storage mcap
  bash tools/record_dual_dds_rosbag.sh --domain 7 --topic /tf --topic /pixel_7_pro_60da1823/ftm_rtt

Notes:
  - This creates one complete session folder with two synchronized bags, not one
    physical bag file. That is intentional: each recorder must run with a different RMW.
  - Use Ctrl+C once to stop both recorders cleanly.
EOF
}

DATA_ROOT="${RTT_GUI_DATA_ROOT:-${XDG_DATA_HOME:-$HOME/.local/share}/rtt-searchagent}"
OUTPUT_ROOT="$DATA_ROOT/rosbags/dual_dds"
DURATION=""
STORAGE="sqlite3"
ROS_DOMAIN="${ROS_DOMAIN_ID:-0}"
FAST_RMW="${RTT_FASTDDS_RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
CYCLONE_RMW="${RTT_CYCLONEDDS_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
CYCLONEDDS_URI_ARG="${CYCLONEDDS_URI:-}"
FASTRTPS_PROFILE_ARG="${FASTRTPS_DEFAULT_PROFILES_FILE:-}"
INCLUDE_HIDDEN=1
SPLIT_SIZE=""
TOPICS=()
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)
            usage
            exit 0
            ;;
        -o|--output-root)
            OUTPUT_ROOT="$2"
            shift 2
            ;;
        -d|--duration)
            DURATION="$2"
            shift 2
            ;;
        -s|--storage)
            STORAGE="$2"
            shift 2
            ;;
        --domain)
            ROS_DOMAIN="$2"
            shift 2
            ;;
        --fast-rmw)
            FAST_RMW="$2"
            shift 2
            ;;
        --cyclone-rmw)
            CYCLONE_RMW="$2"
            shift 2
            ;;
        --cyclonedds-uri)
            CYCLONEDDS_URI_ARG="$2"
            shift 2
            ;;
        --fastrtps-profile)
            FASTRTPS_PROFILE_ARG="$2"
            shift 2
            ;;
        --no-hidden)
            INCLUDE_HIDDEN=0
            shift
            ;;
        --split-size)
            SPLIT_SIZE="$2"
            shift 2
            ;;
        --topic)
            TOPICS+=("$2")
            shift 2
            ;;
        --)
            shift
            EXTRA_ARGS+=("$@")
            break
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: ros2 is not in PATH. Source your ROS 2 setup first, e.g.:" >&2
    echo "  source /opt/ros/humble/setup.bash" >&2
    exit 1
fi

if [[ "$STORAGE" != "sqlite3" && "$STORAGE" != "mcap" ]]; then
    echo "ERROR: --storage must be sqlite3 or mcap" >&2
    exit 2
fi

timestamp="$(date +%Y_%m_%d-%H_%M_%S)"
session_dir="$OUTPUT_ROOT/dual_dds_$timestamp"
fast_bag="$session_dir/fastdds/rosbag2_fastdds_$timestamp"
cyclone_bag="$session_dir/cyclonedds/rosbag2_cyclonedds_$timestamp"
mkdir -p "$session_dir/fastdds" "$session_dir/cyclonedds"

record_args=("bag" "record")
if [[ ${#TOPICS[@]} -eq 0 ]]; then
    record_args+=("-a")
else
    record_args+=("${TOPICS[@]}")
fi
record_args+=("--storage" "$STORAGE")
if [[ "$INCLUDE_HIDDEN" -eq 1 ]]; then
    record_args+=("--include-hidden-topics")
fi
if [[ -n "$SPLIT_SIZE" ]]; then
    record_args+=("--max-bag-size" "$SPLIT_SIZE")
fi
record_args+=("${EXTRA_ARGS[@]}")

run_with_rmw() {
    local rmw="$1"
    local output="$2"
    local log_file="$3"
    shift 3
    (
        export RMW_IMPLEMENTATION="$rmw"
        export ROS_DOMAIN_ID="$ROS_DOMAIN"
        export ROS_LOCALHOST_ONLY=0
        exec ros2 "$@" -o "$output"
    ) >"$log_file" 2>&1 &
    echo $!
}

snapshot_with_rmw() {
    local rmw="$1"
    local prefix="$2"
    (
        export RMW_IMPLEMENTATION="$rmw"
        export ROS_DOMAIN_ID="$ROS_DOMAIN"
        export ROS_LOCALHOST_ONLY=0
        if [[ "$rmw" == rmw_fastrtps* ]]; then
            unset CYCLONEDDS_URI
            [[ -n "$FASTRTPS_PROFILE_ARG" ]] && export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTRTPS_PROFILE_ARG"
        else
            unset FASTRTPS_DEFAULT_PROFILES_FILE
            [[ -n "$CYCLONEDDS_URI_ARG" ]] && export CYCLONEDDS_URI="$CYCLONEDDS_URI_ARG"
        fi
        ros2 topic list -t >"$session_dir/topics_${prefix}_before.txt" 2>"$session_dir/topics_${prefix}_before.err" || true
        ros2 node list >"$session_dir/nodes_${prefix}_before.txt" 2>"$session_dir/nodes_${prefix}_before.err" || true
    )
}

cat >"$session_dir/manifest.env" <<EOF
SESSION_DIR=$session_dir
TIMESTAMP=$timestamp
ROS_DOMAIN_ID=$ROS_DOMAIN
ROS_LOCALHOST_ONLY=0
FAST_RMW=$FAST_RMW
CYCLONE_RMW=$CYCLONE_RMW
STORAGE=$STORAGE
INCLUDE_HIDDEN=$INCLUDE_HIDDEN
CYCLONEDDS_URI=$CYCLONEDDS_URI_ARG
FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_PROFILE_ARG
TOPICS=${TOPICS[*]:-ALL}
EXTRA_ARGS=${EXTRA_ARGS[*]:-}
FAST_BAG=$fast_bag
CYCLONE_BAG=$cyclone_bag
EOF

echo "Session: $session_dir"
echo "Taking graph snapshots..."
snapshot_with_rmw "$FAST_RMW" "fastdds"
snapshot_with_rmw "$CYCLONE_RMW" "cyclonedds"

echo "Starting FastDDS recorder -> $fast_bag"
(
    export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTRTPS_PROFILE_ARG"
    if [[ -z "$FASTRTPS_PROFILE_ARG" ]]; then
        unset FASTRTPS_DEFAULT_PROFILES_FILE
    fi
    unset CYCLONEDDS_URI
    fast_pid="$(run_with_rmw "$FAST_RMW" "$fast_bag" "$session_dir/fastdds_record.log" "${record_args[@]}")"
    echo "$fast_pid" >"$session_dir/fastdds.pid"
)

echo "Starting CycloneDDS recorder -> $cyclone_bag"
(
    export CYCLONEDDS_URI="$CYCLONEDDS_URI_ARG"
    if [[ -z "$CYCLONEDDS_URI_ARG" ]]; then
        unset CYCLONEDDS_URI
    fi
    unset FASTRTPS_DEFAULT_PROFILES_FILE
    cyclone_pid="$(run_with_rmw "$CYCLONE_RMW" "$cyclone_bag" "$session_dir/cyclonedds_record.log" "${record_args[@]}")"
    echo "$cyclone_pid" >"$session_dir/cyclonedds.pid"
)

fast_pid="$(cat "$session_dir/fastdds.pid")"
cyclone_pid="$(cat "$session_dir/cyclonedds.pid")"

stop_recorders() {
    echo
    echo "Stopping recorders..."
    for pid in "$fast_pid" "$cyclone_pid"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -INT "$pid" 2>/dev/null || true
        fi
    done
    wait "$fast_pid" 2>/dev/null || true
    wait "$cyclone_pid" 2>/dev/null || true
    echo "Done. Bags saved under: $session_dir"
    echo "FastDDS log:    $session_dir/fastdds_record.log"
    echo "CycloneDDS log: $session_dir/cyclonedds_record.log"
}
trap stop_recorders INT TERM EXIT

echo "Recording. Press Ctrl+C to stop."
echo "FastDDS PID:    $fast_pid"
echo "CycloneDDS PID: $cyclone_pid"

if [[ -n "$DURATION" ]]; then
    sleep "$DURATION"
else
    while kill -0 "$fast_pid" 2>/dev/null && kill -0 "$cyclone_pid" 2>/dev/null; do
        sleep 1
    done
fi