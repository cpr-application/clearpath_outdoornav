#!/usr/bin/env bash
# Connect this machine to a Clearpath robot using Zenoh or Cyclone DDS.
#
#   source connect_robot.sh 10.27.205.101 cpr-banzai       # prompts for middleware
#   source connect_robot.sh zenoh 10.27.205.101 cpr-banzai # explicit middleware
#   source connect_robot.sh zenoh --ssh-tunnel robot@cpr-rooftop
#   source connect_robot.sh cyclone_dds 10.27.205.101
#
# The robot must use the selected middleware and the same ROS_DOMAIN_ID.

_cpr_middleware_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_cpr_gen_dir="${XDG_CACHE_HOME:-$HOME/.cache}/clearpath_remote"

cpr_zenoh_tunnel_stop() {
  if [[ -z "${CPR_ZENOH_TUNNEL_CONTROL_PATH:-}" ]]; then
    echo "No managed Zenoh SSH tunnel is running."
    return 0
  fi

  if ! ssh -S "$CPR_ZENOH_TUNNEL_CONTROL_PATH" -O check \
      -p "$CPR_ZENOH_TUNNEL_SSH_PORT" "$CPR_ZENOH_TUNNEL_SSH_TARGET" >/dev/null 2>&1; then
    rm -f -- "$CPR_ZENOH_TUNNEL_CONTROL_PATH"
    unset CPR_ZENOH_TUNNEL_CONTROL_PATH CPR_ZENOH_TUNNEL_SSH_PORT CPR_ZENOH_TUNNEL_SSH_TARGET
    echo "The managed Zenoh SSH tunnel was not running."
    return 0
  fi

  ssh -S "$CPR_ZENOH_TUNNEL_CONTROL_PATH" -O exit \
    -p "$CPR_ZENOH_TUNNEL_SSH_PORT" "$CPR_ZENOH_TUNNEL_SSH_TARGET" || return $?
  rm -f -- "$CPR_ZENOH_TUNNEL_CONTROL_PATH"
  unset CPR_ZENOH_TUNNEL_CONTROL_PATH CPR_ZENOH_TUNNEL_SSH_PORT CPR_ZENOH_TUNNEL_SSH_TARGET
  echo "Stopped the managed Zenoh SSH tunnel."
}

_cpr_is_ipv4() {
  local octet
  local -a octets

  [[ "$1" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]] || return 1
  IFS=. read -r -a octets <<< "$1"
  for octet in "${octets[@]}"; do
    ((10#$octet <= 255)) || return 1
  done
}

_cpr_is_hostname() {
  local label
  local -a labels

  [[ ${#1} -le 253 && "$1" =~ ^[A-Za-z0-9]([A-Za-z0-9.-]*[A-Za-z0-9])?$ ]] || return 1
  IFS=. read -r -a labels <<< "$1"
  for label in "${labels[@]}"; do
    [[ ${#label} -le 63 && "$label" =~ ^[A-Za-z0-9]([A-Za-z0-9-]*[A-Za-z0-9])?$ ]] || return 1
  done
}

_cpr_resolve_robot() {
  local address

  if _cpr_is_ipv4 "$ROBOT_IP"; then
    ROBOT_HOST="${ROBOT_HOST:-$(getent hosts "$ROBOT_IP" 2>/dev/null | awk '{print $2; exit}')}"
  else
    ROBOT_HOST="$ROBOT_IP"
    address="$(getent ahostsv4 "$ROBOT_HOST" 2>/dev/null | awk '{print $1; exit}')"
    if [[ -z "$address" ]]; then
      echo "ERROR: could not resolve '$ROBOT_HOST' to an IPv4 address." >&2
      return 1
    fi
    ROBOT_IP="$address"
  fi

  if [[ "$ROBOT_MIDDLEWARE" == zenoh && -z "$ROBOT_HOST" ]]; then
    echo "ERROR: nothing resolves $ROBOT_IP. Supply the robot hostname as the third argument:" >&2
    echo "       source connect_robot.sh zenoh $ROBOT_IP <hostname>" >&2
    return 1
  fi
}

_cpr_pin_hosts_entry() {
  local short="${ROBOT_HOST%%.*}"
  local existing_ips
  local existing_ip

  _cpr_is_ipv4 "$ROBOT_IP" || return 0
  [[ "$short" =~ ^[A-Za-z0-9]([A-Za-z0-9-]*[A-Za-z0-9])?$ ]] || return 0

  existing_ips="$(awk -v name="$short" '
    $1 !~ /^#/ {
      for (i = 2; i <= NF; i++) {
        if (tolower($i) == tolower(name)) print $1
      }
    }
  ' /etc/hosts | sort -u)"
  while IFS= read -r existing_ip; do
    [[ -z "$existing_ip" || "$existing_ip" == "$ROBOT_IP" ]] && continue
    echo "ERROR: '$short' already maps to $existing_ip in /etc/hosts, not $ROBOT_IP." >&2
    echo "       Resolve this conflict in /etc/hosts before connecting." >&2
    return 1
  done <<< "$existing_ips"

  [[ -n "$existing_ips" || "${ROBOT_PIN_HOST:-1}" != 1 ]] && return 0

  echo "hosts      : adding '$ROBOT_IP $short' to /etc/hosts (needs sudo)"
  printf '%s\t%s\n' "$ROBOT_IP" "$short" | sudo tee -a /etc/hosts > /dev/null \
    || echo "WARNING: could not write /etc/hosts; continuing without it." >&2
}

_cpr_setup_zenoh() {
  unset CYCLONEDDS_URI ROBOT_LOCAL_IP
  ROBOT_ZENOH_ENDPOINT="${ROBOT_ZENOH_ENDPOINT:-tcp/${ROBOT_IP}:7447}"
  export ROBOT_IP ROBOT_HOST ROBOT_ZENOH_ENDPOINT ROS_DOMAIN_ID
  envsubst '${ROBOT_ZENOH_ENDPOINT}' \
    < "$_cpr_middleware_dir/zenoh_session.json5.in" \
    > "$_cpr_gen_dir/zenoh_session.json5"
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  export ZENOH_SESSION_CONFIG_URI="$_cpr_gen_dir/zenoh_session.json5"
}

_cpr_setup_cyclone_dds() {
  local route

  unset ZENOH_SESSION_CONFIG_URI
  route="$(ip -4 route get "$ROBOT_IP" 2>/dev/null)"
  ROBOT_LOCAL_IP="$(awk '{for (i = 1; i <= NF; i++) if ($i == "src") {print $(i + 1); exit}}' <<< "$route")"
  if [[ -z "$ROBOT_LOCAL_IP" ]]; then
    echo "ERROR: could not determine the local IPv4 address used to reach $ROBOT_IP." >&2
    return 1
  fi

  export ROBOT_IP ROBOT_LOCAL_IP ROS_DOMAIN_ID
  envsubst '${ROBOT_IP} ${ROBOT_LOCAL_IP}' \
    < "$_cpr_middleware_dir/cyclonedds.xml.in" \
    > "$_cpr_gen_dir/cyclonedds.xml"
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="file://$_cpr_gen_dir/cyclonedds.xml"
}

_cpr_detect_namespace() {
  timeout "${_cpr_graph_timeout:-20}" ros2 topic list --no-daemon 2>/dev/null \
    | awk -F/ 'NF>2 && $2!="" {print $2}' \
    | grep -vE '^(parameter_events|rosout|diagnostics|foxglove_bridge|tf|tf_static|clicked_point)$' \
    | sort | uniq -c | sort -rn | head -1 | awk '{print $2}'
}

_cpr_start_zenoh_tunnel() {
  local ssh_target="$1" ssh_port="$2" router_ip="$3" local_port="$4"
  local control_path="${_cpr_gen_dir}/zenoh-ssh-${local_port}-$$.sock" attempt

  if [[ -n "${CPR_ZENOH_TUNNEL_CONTROL_PATH:-}" ]] &&
      ssh -S "$CPR_ZENOH_TUNNEL_CONTROL_PATH" -O check \
        -p "$CPR_ZENOH_TUNNEL_SSH_PORT" "$CPR_ZENOH_TUNNEL_SSH_TARGET" >/dev/null 2>&1; then
    echo "ERROR: a managed Zenoh SSH tunnel is already running." >&2
    echo "       Run cpr_zenoh_tunnel_stop before starting another." >&2
    return 1
  fi
  unset CPR_ZENOH_TUNNEL_CONTROL_PATH CPR_ZENOH_TUNNEL_SSH_PORT CPR_ZENOH_TUNNEL_SSH_TARGET
  mkdir -p "$_cpr_gen_dir" || return $?
  if ((${#control_path} >= 100)); then
    echo "ERROR: SSH control socket path is too long: $control_path" >&2
    return 1
  fi

  ssh -M -S "$control_path" -fN -p "$ssh_port" \
    -L "127.0.0.1:${local_port}:${router_ip}:7447" \
    -o ExitOnForwardFailure=yes \
    -o ServerAliveInterval=30 \
    -o ServerAliveCountMax=3 \
    "$ssh_target" || return $?
  CPR_ZENOH_TUNNEL_CONTROL_PATH="$control_path"
  CPR_ZENOH_TUNNEL_SSH_PORT="$ssh_port"
  CPR_ZENOH_TUNNEL_SSH_TARGET="$ssh_target"

  for ((attempt = 0; attempt < 5; attempt++)); do
    if ! ssh -S "$control_path" -O check -p "$ssh_port" "$ssh_target" >/dev/null 2>&1; then
      echo "ERROR: the SSH tunnel master did not remain available." >&2
      cpr_zenoh_tunnel_stop >&2
      return 1
    fi
    if timeout 1 bash -c ":</dev/tcp/127.0.0.1/${local_port}" 2>/dev/null; then
      ROBOT_ZENOH_ENDPOINT="tcp/127.0.0.1:${local_port}"
      echo "ssh tunnel : 127.0.0.1:${local_port} -> ${router_ip}:7447 via ${ssh_target}:${ssh_port}"
      return 0
    fi
    sleep 1
  done

  echo "ERROR: the SSH tunnel did not open local port ${local_port}." >&2
  cpr_zenoh_tunnel_stop >&2
  return 1
}

_cpr_main() {
  local middleware="${1:-}"
  local ssh_target="" ssh_port=1122 router_ip=192.168.131.1 local_port=7447 port octet
  local tunnel_requested=0
  local -a router_octets

  case "$middleware" in
    zenoh|cyclone_dds|cyclone|cyclonedds)
      shift
      ;;
    *)
      middleware=""
      if [[ ! -t 0 ]]; then
        echo "ERROR: middleware selection requires an interactive terminal." >&2
        echo "       Pass 'zenoh' or 'cyclone_dds' as the first argument." >&2
        return 2
      fi
      printf 'Which middleware would you like to use?\n  1) Zenoh\n  2) Cyclone DDS\nChoice [1-2]: ' >&2
      IFS= read -r middleware || return 2
      printf '\n' >&2
      ;;
  esac

  case "$middleware" in
    1|zenoh)
      ROBOT_MIDDLEWARE=zenoh
      ;;
    2|cyclone|cyclonedds|cyclone_dds)
      ROBOT_MIDDLEWARE=cyclone_dds
      ;;
    *)
      echo "ERROR: choose 1 (Zenoh) or 2 (Cyclone DDS)." >&2
      return 2
      ;;
  esac

  while (($#)); do
    case "$1" in
      --ssh-tunnel)
        if [[ "$ROBOT_MIDDLEWARE" != zenoh || $# -lt 2 || -n "$ssh_target" ]]; then
          echo "Usage: source connect_robot.sh zenoh --ssh-tunnel <user@ssh-host> [--ssh-port port] [--router-ip ip] [--local-port port]" >&2
          return 2
        fi
        ssh_target="$2"
        tunnel_requested=1
        shift 2
        ;;
      --ssh-port|--router-ip|--local-port)
        if [[ $# -lt 2 ]]; then
          echo "ERROR: $1 requires a value." >&2
          return 2
        fi
        case "$1" in
          --ssh-port) ssh_port="$2" ;;
          --router-ip) router_ip="$2" ;;
          --local-port) local_port="$2" ;;
        esac
        shift 2
        ;;
      *)
        break
        ;;
    esac
  done

  if ((tunnel_requested)); then
    if [[ "$ssh_target" != *@* || ! "$ssh_target" =~ ^[A-Za-z0-9._-]+@[A-Za-z0-9][A-Za-z0-9.-]*[A-Za-z0-9]?$ ]]; then
      echo "ERROR: SSH target must be in user@host form (for example robot@cpr-rooftop)." >&2
      return 2
    fi
    for port in "$ssh_port" "$local_port"; do
      if [[ ! "$port" =~ ^[0-9]+$ ]] || ((10#$port < 1 || 10#$port > 65535)); then
        echo "ERROR: SSH and local ports must be between 1 and 65535." >&2
        return 2
      fi
    done
    if ! _cpr_is_ipv4 "$router_ip"; then
      echo "ERROR: router IP must be an IPv4 address." >&2
      return 2
    fi
    IFS=. read -r -a router_octets <<< "$router_ip"
    for octet in "${router_octets[@]}"; do
      if ((10#$octet > 255)); then
        echo "ERROR: router IP must be a valid IPv4 address." >&2
        return 2
      fi
    done
    if (($#)); then
      echo "ERROR: do not supply a direct robot address when using --ssh-tunnel." >&2
      return 2
    fi
    ROBOT_IP="$router_ip"
    ROBOT_HOST="${ssh_target#*@}"
  else
    ROBOT_IP="${1:-${ROBOT_IP:-}}"
    ROBOT_HOST="${2:-${ROBOT_HOST:-}}"
  fi
  ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  if [[ -z "$ROBOT_IP" ]]; then
    if [[ ! -t 0 ]]; then
      echo "ERROR: provide the robot IP address or hostname after the middleware choice." >&2
      return 2
    fi
    printf 'Robot IP address or hostname: ' >&2
    IFS= read -r ROBOT_IP || return 2
    printf '\n' >&2
  fi

  if _cpr_is_ipv4 "$ROBOT_IP"; then
    if [[ -z "$ROBOT_HOST" ]]; then
      ROBOT_HOST="$(getent hosts "$ROBOT_IP" 2>/dev/null | awk '{print $2; exit}')"
    fi
    if [[ -z "$ROBOT_HOST" && -t 0 ]]; then
      printf 'Robot hostname (press Enter to use reverse lookup): ' >&2
      IFS= read -r ROBOT_HOST || return 2
      printf '\n' >&2
    fi
    if [[ -n "$ROBOT_HOST" ]] && ! _cpr_is_hostname "$ROBOT_HOST"; then
      echo "ERROR: '$ROBOT_HOST' is not a valid hostname." >&2
      return 2
    fi
  elif [[ "$ROBOT_IP" =~ ^[0-9.]+$ ]] || ! _cpr_is_hostname "$ROBOT_IP"; then
    echo "ERROR: '$ROBOT_IP' is not a valid IPv4 address or hostname." >&2
    return 2
  fi

  _cpr_resolve_robot || return 1
  if ((!tunnel_requested)); then
    _cpr_pin_hosts_entry || return 1
  fi
  export ROBOT_MIDDLEWARE ROBOT_HOST ROS_DOMAIN_ID
  if [[ "$ROBOT_MIDDLEWARE" == zenoh ]]; then
    _cpr_graph_timeout=20
  else
    _cpr_graph_timeout=30
  fi

  source /opt/ros/jazzy/setup.bash
  unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT ROS_AUTOMATIC_DISCOVERY_RANGE
  unset ROS_STATIC_PEERS ROS_LOCALHOST_ONLY
  unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
  mkdir -p "$_cpr_gen_dir"

  if [[ "$ROBOT_MIDDLEWARE" == zenoh ]]; then
    if ((tunnel_requested)); then
      _cpr_start_zenoh_tunnel "$ssh_target" "$ssh_port" "$router_ip" "$local_port" || return $?
    else
      ROBOT_ZENOH_ENDPOINT="tcp/${ROBOT_IP}:7447"
    fi
    _cpr_setup_zenoh || return 1
  else
    _cpr_setup_cyclone_dds || return 1
  fi
  ros2 daemon stop > /dev/null 2>&1

  ROBOT_NAMESPACE="$(_cpr_detect_namespace)"
  if [[ -z "$ROBOT_NAMESPACE" ]]; then
    echo "ERROR: could not determine namespace from the topic graph." >&2
    if [[ "$ROBOT_MIDDLEWARE" == zenoh ]]; then
      echo "       Is the Zenoh router reachable at $ROBOT_IP:7447?" >&2
    else
      echo "       Confirm both hosts use Cyclone DDS, domain $ROS_DOMAIN_ID, and can exchange DDS UDP traffic." >&2
    fi
    if ((tunnel_requested)); then
      cpr_zenoh_tunnel_stop >&2
    fi
    return 1
  fi
  export ROBOT_NAMESPACE

  echo
  echo "robot      : $ROBOT_NAMESPACE @ ${ROBOT_HOST:-$ROBOT_IP} ($ROBOT_IP, domain $ROS_DOMAIN_ID)"
  echo "middleware : $ROBOT_MIDDLEWARE ($RMW_IMPLEMENTATION)"
  if [[ "$ROBOT_MIDDLEWARE" == zenoh ]]; then
    echo "endpoint   : $ROBOT_ZENOH_ENDPOINT"
    echo "config     : $ZENOH_SESSION_CONFIG_URI"
  else
    echo "interface  : $ROBOT_LOCAL_IP"
    echo "config     : $CYCLONEDDS_URI"
  fi
  echo "launch     :"
  printf '      ros2 launch %q namespace:=%q\n' \
    "$_cpr_middleware_dir/../rviz/rviz_navigation.launch.py" "$ROBOT_NAMESPACE"
}

_cpr_main "$@"
_cpr_rc=$?
unset -f _cpr_is_ipv4 _cpr_is_hostname _cpr_resolve_robot _cpr_pin_hosts_entry \
         _cpr_setup_zenoh _cpr_setup_cyclone_dds _cpr_detect_namespace \
         _cpr_start_zenoh_tunnel _cpr_main
unset _cpr_middleware_dir _cpr_gen_dir _cpr_graph_timeout
if ((_cpr_rc != 0)); then
  unset _cpr_rc
  return 1 2>/dev/null || exit 1
fi
unset _cpr_rc
