# generated from colcon_zsh/shell/template/prefix_chain.zsh.em

# This script extends the environment with the environment of other prefix
# paths which were sourced when this file was generated as well as all packages
# contained in this prefix path.

# function to source another script with conditional trace output
# first argument: the path of the script
_colcon_prefix_chain_zsh_source_script() {
  if [ -f "$1" ]; then
    if [ -n "$COLCON_TRACE" ]; then
      echo "# . \"$1\""
    fi
    . "$1"
  else
    echo "not found: \"$1\"" 1>&2
  fi
}

# source chained prefixes
# setting COLCON_CURRENT_PREFIX avoids determining the prefix in the sourced script
COLCON_CURRENT_PREFIX="/opt/ros/jazzy"
_colcon_prefix_chain_zsh_source_script "$COLCON_CURRENT_PREFIX/local_setup.zsh"
# Resolve ros2_ws/install relative to this script's location (portable fix)
_bb_ws_install="$(builtin cd "$(dirname "${(%):-%x}")/../ros2_ws/install" > /dev/null 2>&1 && pwd)"
if [[ -n "$_bb_ws_install" && -d "$_bb_ws_install" ]]; then
  COLCON_CURRENT_PREFIX="$_bb_ws_install"
  _colcon_prefix_chain_zsh_source_script "$COLCON_CURRENT_PREFIX/local_setup.zsh"
fi
unset _bb_ws_install

# source this prefix
# setting COLCON_CURRENT_PREFIX avoids determining the prefix in the sourced script
COLCON_CURRENT_PREFIX="$(builtin cd -q "`dirname "${(%):-%N}"`" > /dev/null && pwd)"
_colcon_prefix_chain_zsh_source_script "$COLCON_CURRENT_PREFIX/local_setup.zsh"

unset COLCON_CURRENT_PREFIX
unset _colcon_prefix_chain_zsh_source_script
