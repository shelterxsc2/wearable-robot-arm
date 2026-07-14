#!/usr/bin/env bash
set -u
duration="${1:-600}"
label="${2:-run}"
out="/tmp/fourth_perf_${label}_metrics.csv"
echo "epoch,elapsed_s,frames,rss_kib,cpu_ticks,temp_max_mc,voice_rss_kib,voice_cpu_ticks" >"$out"
start=$(date +%s)
while true; do
  now=$(date +%s)
  elapsed=$((now-start))
  (( elapsed > duration )) && break
  pid=$(pgrep -n -f '/build/cc|./build/cc' || true)
  [[ -z "$pid" ]] && sleep 1 && continue
  frames=$(rg -o 'count=[0-9]+' "/tmp/fourth_perf_${label}.log" 2>/dev/null | tail -1 | cut -d= -f2)
  rss=$(awk '/VmRSS/{print $2}' "/proc/$pid/status" 2>/dev/null || true)
  ticks=$(awk '{print $14+$15}' "/proc/$pid/stat" 2>/dev/null || true)
  temp=$(awk 'BEGIN{m=0} {if($1>m)m=$1} END{print m}' /sys/class/thermal/thermal_zone*/temp 2>/dev/null)
  vpid=$(pgrep -n -f 'scripts/voice_kws_server.py' || true)
  vrss=0; vticks=0
  if [[ -n "$vpid" ]]; then
    vrss=$(awk '/VmRSS/{print $2}' "/proc/$vpid/status" 2>/dev/null || echo 0)
    vticks=$(awk '{print $14+$15}' "/proc/$vpid/stat" 2>/dev/null || echo 0)
  fi
  echo "$now,$elapsed,${frames:-0},${rss:-0},${ticks:-0},${temp:-0},${vrss:-0},${vticks:-0}" >>"$out"
  sleep 5
done
