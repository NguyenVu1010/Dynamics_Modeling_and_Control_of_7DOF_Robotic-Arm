#!/usr/bin/env bash
# Records the demo video docs/media/demo-v1.{mp4,gif}: waypoints placed and previewed in the RViz2
# panel, the run in Gazebo, RViz and cobot_studio at the same time, then the live tracking plots
# in cobot_studio.
#
# Run in WSL from the repository root, after tools/wsl/sim.sh build and
# tools/wsl/build.sh --studio:
#   tools/demo/record_demo.sh [OUT_DIR]          (default: docs/media)
# Needs Xvfb, Mesa (llvmpipe), python-xlib and an ffmpeg with x11grab, libx264 and drawtext
# (FFMPEG=/path/to/ffmpeg picks one). It runs on its own X display (:97), ROS domain and Gazebo
# partition, so it needs no desktop and does not talk to another simulation. The GUI is driven
# with XTest events at fixed screen positions (tools/demo/xinput.py), for the window layout set
# below. HOLD=1 stops after the setup and keeps everything running; KEEP=1 keeps the raw
# recording and the logs in /tmp/cobot_demo.*.
set -euo pipefail

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
demo="$repo/tools/demo"
out="$(realpath -m "${1:-$repo/docs/media}")"
export COBOT_ROS_DOMAIN_ID="${COBOT_ROS_DOMAIN_ID:-97}" COBOT_GZ_PARTITION="${COBOT_GZ_PARTITION:-cobot_demo}"
set +u
# shellcheck source=SCRIPTDIR/../wsl/env.sh
source "$repo/tools/wsl/env.sh"
set -u
studio="$(cobot_build_dir)/src/studio/app/cobot_studio"
ffmpeg="${FFMPEG:-ffmpeg}"
export DISPLAY=:97 LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1
work="$(mktemp -d /tmp/cobot_demo.XXXXXX)"
wp="$work/waypoints.yaml"         # saved by the RViz panel, opened by the studio
segments=/tmp/cobot_cobot_controller   # where the controller exports the studio's live runs
pids=()

x() { python3 "$demo/xinput.py" "$@"; }
t() { echo "$1 $(date +%s.%N)" >> "$work/marks"; }   # time stamp of a step, for the cut
bg() { setsid "$@" >> "$work/log.txt" 2>&1 & pids+=($!); }   # own process group
cleanup() {
  for p in "${pids[@]}"; do kill -INT -- "-$p" 2>/dev/null || true; done
  sleep 8
  for p in "${pids[@]}"; do kill -KILL -- "-$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
  if [ "${KEEP:-0}" != 1 ]; then rm -rf "$work"; fi
}
trap cleanup EXIT
# Returns once a joint of /joint_states moves faster than 0.05 rad/s (the run has started).
wait_motion() {
  python3 - <<'PY'
import rclpy
from sensor_msgs.msg import JointState
rclpy.init()
node = rclpy.create_node("demo_wait_motion")
moving = []
node.create_subscription(
    JointState, "/joint_states", lambda m: moving.append(max(map(abs, m.velocity), default=0) > 0.05), 10)
while not any(moving):
    rclpy.spin_once(node, timeout_sec=0.1)
PY
}

# 1. Display, Gazebo server + controller with the studio bridge, Gazebo GUI, RViz2, studio.
bg Xvfb :97 -screen 0 1920x1080x24 -nolisten tcp
sleep 2
bg ros2 launch cobot_sim sim.launch.py controller:=cobot_controller studio:=true gui:=false \
  imperfections_path:="$repo/config/imperfections/realistic.yaml" seed:=7
for _ in $(seq 120); do [ -e /dev/shm/cobot_cobot_controller ] && break; sleep 1; done
bg gz sim -g --gui-config "$demo/gazebo_gui.config"
bg ros2 run rviz2 rviz2 -d "$demo/demo.rviz" --ros-args -p use_sim_time:=true
bg "$studio" --bridge cobot_controller --config-dir "$repo/config" --work-dir "$work/runs"
sleep 25
x place "RViz" 0 0 820 1080
x place "Gazebo" 820 0 1100 470
x place "cobot_studio" 820 470 1100 610
sleep 3
x click 1045 485; sleep 1          # studio Tracking tab: widen the chart, 3D view to the side
x drag 1259 692 1700 692; sleep 0.5
x click 969 485; sleep 0.5         # Control tab, waypoint page
x click 981 554
sleep 5
if [ "${HOLD:-0}" = 1 ]; then echo "HOLD: $work"; sleep infinity; fi

# 2. Record the screen while driving the GUI.
"$ffmpeg" -loglevel error -f x11grab -framerate 25 -video_size 1920x1080 -i :97 \
  -c:v libx264 -preset ultrafast -crf 12 -pix_fmt yuv420p "$work/raw.mkv" < /dev/null &
rec=$!
t rec
sleep 2
# RViz panel: load four waypoints, copy the last one and drag it 10 cm up, preview, save.
t load
x click 732 975; sleep 1
x type "$demo/demo_waypoints.yaml"; x key Return
t loaded
sleep 1.5
x click 16 895; sleep 0.8          # select point 4
x click 87 975; sleep 1.2          # Add from marker: point 5 on point 4
x drag 376 407 376 350           # its up arrow: about 12 cm up
sleep 1.5
t preview
x click 248 1006; sleep 3.5
t save
x click 87 1006; sleep 1
x type "$wp"; x key Return; sleep 1
# Studio: open the same file, take control, preview and run; Gazebo and RViz follow.
x click 1093 883; sleep 1
x type "$wp"; x key Return; sleep 1
x click 1779 520; sleep 0.8        # Take control
x click 1265 883; sleep 2          # Preview
n0="$(find "$segments" -name '*.csv' 2>/dev/null | wc -l)"
x click 1351 883                   # Run; the arm starts some seconds later
wait_motion
t run
for _ in $(seq 240); do
  [ "$(find "$segments" -name '*.csv' 2>/dev/null | wc -l)" -gt "$n0" ] && break
  sleep 0.5
done
sleep 8                            # the studio loads the exported segment
# Tracking tab: joint 2 q ref / q, then q', the tracking error and tau.
t plots
x click 1045 485; sleep 1
x click 1086 520; sleep 0.5; x click 1080 546; sleep 3.5   # Joint 2
x click 1256 520; x click 1136 520; x click 1197 520; sleep 3.5   # q'
x click 1510 520; x click 1256 520; sleep 3.5                     # error
x click 1574 520; x click 1510 520; sleep 4                       # tau
t end
kill -INT "$rec"; wait "$rec" || true

# 3. Cut: four scenes, captions in a band below the screen; dead time (file dialogs, the wait
# for the run to start, the export and loading of the segment) is left out.
declare -A at
while read -r k v; do at[$k]=$v; done < "$work/marks"
rel() { awk -v a="${at[$1]}" -v r="${at[rec]}" -v d="$2" 'BEGIN { printf "%.2f", a - r + d }'; }
# scene: start end speed caption
scenes=(
  "$(rel load -0.3) $(rel loaded 0) 4 Place waypoints in RViz"
  "$(rel loaded 0.5) $(rel preview 0) 1 Place waypoints in RViz"
  "$(rel preview -0.3) $(rel save 0) 1 Preview the path"
  "$(rel run -1.5) $(rel run 16) 1 Run: Gazebo and RViz in sync, 1 kHz computed torque"
  "$(rel plots 0.8) $(rel end 0) 1 Tracking plots of the run in cobot_studio"
)
font=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf
inputs=() filters="" concat=""
for i in "${!scenes[@]}"; do
  read -r a b speed caption <<< "${scenes[$i]}"
  inputs+=(-ss "$a" -to "$b" -i "$work/raw.mkv")
  printf '%s' "$caption" > "$work/caption$i.txt"
  filters+="[$i:v]setpts=(PTS-STARTPTS)/$speed,fps=25,pad=1920:1170:0:0:0x1e1e1e,drawtext=fontfile=$font:textfile=$work/caption$i.txt:"
  filters+="fontsize=44:fontcolor=white:x=(w-text_w)/2:y=1080+(90-text_h)/2[v$i];"
  concat+="[v$i]"
done
mkdir -p "$out"
"$ffmpeg" -loglevel error -y "${inputs[@]}" \
  -filter_complex "${filters}${concat}concat=n=${#scenes[@]}:v=1:a=0[v]" -map "[v]" \
  -c:v libx264 -preset slow -crf 24 -pix_fmt yuv420p -movflags +faststart "$out/demo-v1.mp4"
"$ffmpeg" -loglevel error -y -i "$out/demo-v1.mp4" -filter_complex \
  "fps=12,scale=960:-2:flags=lanczos,split[a][b];[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle" \
  "$out/demo-v1.gif"
ls -l "$out/demo-v1.mp4" "$out/demo-v1.gif"
