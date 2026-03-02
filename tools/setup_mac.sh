#!/usr/bin/env bash
# setup_mac.sh — macOS developer environment setup for Drone Swarm SLAM
# Usage: ./tools/setup_mac.sh [--check] [--all]
#
# Flags:
#   --check   Only report tool status, install nothing
#   --all     Also install optional tools (ESP-IDF native prereqs, colcon)
#
# Output:
#   ✅  <tool>  — already installed
#   ❌  <tool>  — missing (required)
#   ⚠️  <tool>  — missing (optional)
#
# Exit codes:
#   0 — all required tools present (or user confirmed installs)
#   1 — one or more required tools missing and not installed

# DESIGN: No 'set -e' — we deliberately continue after each tool check failure
# so we can produce a complete status report before prompting the user.
set -uo pipefail

# ---------------------------------------------------------------------------
# Colors / symbols
# ---------------------------------------------------------------------------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
RESET='\033[0m'

MARK_OK="✅ "
MARK_MISSING="❌ "
MARK_OPTIONAL="⚠️  "

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
CHECK_ONLY=false
INSTALL_ALL=false

for arg in "$@"; do
    case "${arg}" in
        --check) CHECK_ONLY=true ;;
        --all)   INSTALL_ALL=true ;;
        *)
            echo "Usage: $0 [--check] [--all]" >&2
            exit 1
            ;;
    esac
done

# ---------------------------------------------------------------------------
# State tracking
# ---------------------------------------------------------------------------
declare -a STATUS_OK=()
declare -a STATUS_MISSING=()
declare -a STATUS_OPTIONAL=()
declare -a STATUS_INSTALLED=()
declare -a STATUS_FAILED=()
declare -a STATUS_SKIPPED=()

# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------
log_ok()       { printf "${GREEN}${MARK_OK}${RESET}  %-30s %s\n" "$1" "${2:-}"; }
log_missing()  { printf "${RED}${MARK_MISSING}${RESET} %-30s %s\n" "$1" "${2:-}"; }
log_optional() { printf "${YELLOW}${MARK_OPTIONAL}${RESET}%-30s %s\n" "$1" "${2:-}"; }
log_info()     { printf "   ${CYAN}→${RESET} %s\n" "$*"; }
log_section()  { printf "\n${BOLD}%s${RESET}\n" "$*"; printf '%0.s─' {1..50}; printf '\n'; }

cmd_exists() { command -v "$1" >/dev/null 2>&1; }

# check_tool name command [version_flag]
# Sets STATUS_OK or STATUS_MISSING and echoes the version string.
check_tool() {
    local name="$1"
    local cmd="$2"
    local vflag="${3:---version}"

    if cmd_exists "${cmd}"; then
        local ver
        ver="$(${cmd} ${vflag} 2>&1 | head -1 || true)"
        log_ok "${name}" "${ver}"
        STATUS_OK+=("${name}")
        return 0
    else
        log_missing "${name}"
        STATUS_MISSING+=("${name}")
        return 1
    fi
}

# check_optional name command [version_flag]
check_optional() {
    local name="$1"
    local cmd="$2"
    local vflag="${3:---version}"

    if cmd_exists "${cmd}"; then
        local ver
        ver="$(${cmd} ${vflag} 2>&1 | head -1 || true)"
        log_ok "${name}" "${ver}"
        STATUS_OK+=("${name}")
        return 0
    else
        log_optional "${name} (optional)"
        STATUS_OPTIONAL+=("${name}")
        return 1
    fi
}

brew_install() {
    local name="$1"
    local pkg="${2:-${1}}"
    echo ""
    log_info "Installing ${name} via brew..."
    if brew install "${pkg}"; then
        STATUS_INSTALLED+=("${name}")
        return 0
    else
        echo "  ${RED}Failed to install ${name}.${RESET}" >&2
        STATUS_FAILED+=("${name}")
        return 1
    fi
}

brew_cask_install() {
    local name="$1"
    local pkg="${2:-${1}}"
    echo ""
    log_info "Installing ${name} via brew --cask..."
    if brew install --cask "${pkg}"; then
        STATUS_INSTALLED+=("${name}")
        return 0
    else
        echo "  ${RED}Failed to install ${name}.${RESET}" >&2
        STATUS_FAILED+=("${name}")
        return 1
    fi
}

pip_install() {
    local name="$1"
    local pkg="${2:-${1}}"
    echo ""
    log_info "Installing ${name} via pip3..."
    if pip3 install --quiet "${pkg}"; then
        STATUS_INSTALLED+=("${name}")
        return 0
    else
        echo "  ${RED}Failed to install ${name}.${RESET}" >&2
        STATUS_FAILED+=("${name}")
        return 1
    fi
}

confirm() {
    local prompt="$1"
    read -r -p "${prompt} [y/N] " REPLY
    echo ""
    case "${REPLY}" in
        y|Y) return 0 ;;
        *)   return 1 ;;
    esac
}

# ---------------------------------------------------------------------------
# Header
# ---------------------------------------------------------------------------
echo ""
printf "${BOLD}================================================${RESET}\n"
printf "${BOLD} Drone Swarm SLAM — macOS Environment Setup${RESET}\n"
printf "${BOLD}================================================${RESET}\n"
if ${CHECK_ONLY}; then
    printf "  Mode: ${CYAN}check only${RESET} (no installs)\n"
elif ${INSTALL_ALL}; then
    printf "  Mode: ${CYAN}install required + optional${RESET}\n"
else
    printf "  Mode: ${CYAN}install required tools${RESET}\n"
fi
echo ""

# ---------------------------------------------------------------------------
# 1. Homebrew (prerequisite for everything else)
# ---------------------------------------------------------------------------
log_section "1. Homebrew"

BREW_MISSING=false
if cmd_exists brew; then
    BREW_VER="$(brew --version 2>&1 | head -1)"
    log_ok "Homebrew" "${BREW_VER}"
    STATUS_OK+=("Homebrew")
else
    log_missing "Homebrew"
    STATUS_MISSING+=("Homebrew")
    BREW_MISSING=true
fi

if ${BREW_MISSING} && ! ${CHECK_ONLY}; then
    echo ""
    log_info "Homebrew is required to install most tools."
    log_info "Install command: /bin/bash -c \"\$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)\""
    echo ""
    if confirm "Install Homebrew now?"; then
        /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
        # Re-check after install
        if cmd_exists brew; then
            STATUS_INSTALLED+=("Homebrew")
            BREW_MISSING=false
            # On Apple Silicon, brew is in /opt/homebrew/bin — may need PATH update
            eval "$(/opt/homebrew/bin/brew shellenv 2>/dev/null || true)"
        else
            STATUS_FAILED+=("Homebrew")
        fi
    else
        STATUS_SKIPPED+=("Homebrew")
        echo ""
        echo "  Skipping Homebrew. Many tools cannot be installed without it."
    fi
fi

# ---------------------------------------------------------------------------
# 2. Docker Desktop
# ---------------------------------------------------------------------------
log_section "2. Docker Desktop (required)"

DOCKER_MISSING=false
if cmd_exists docker && docker --version >/dev/null 2>&1; then
    DOCKER_VER="$(docker --version 2>&1)"
    log_ok "Docker" "${DOCKER_VER}"
    STATUS_OK+=("Docker")
else
    log_missing "Docker Desktop"
    STATUS_MISSING+=("Docker Desktop")
    DOCKER_MISSING=true
    log_info "Docker Desktop cannot be auto-installed (requires DMG / App Store)."
    log_info "Download: https://www.docker.com/products/docker-desktop/"
    log_info "Or via brew: brew install --cask docker"
fi

if ${DOCKER_MISSING} && ! ${CHECK_ONLY}; then
    echo ""
    if confirm "Attempt to install Docker Desktop via brew --cask?"; then
        brew_cask_install "Docker Desktop" "docker"
        # Remind user they still need to launch the app once
        log_info "Docker Desktop installed. Launch the app once to complete setup."
    else
        STATUS_SKIPPED+=("Docker Desktop")
    fi
fi

# ---------------------------------------------------------------------------
# 3. Python 3
# ---------------------------------------------------------------------------
log_section "3. Python 3 (required)"

PYTHON_MISSING=false
if cmd_exists python3; then
    PYTHON_VER="$(python3 --version 2>&1)"
    log_ok "Python 3" "${PYTHON_VER}"
    STATUS_OK+=("Python 3")
else
    log_missing "Python 3"
    STATUS_MISSING+=("Python 3")
    PYTHON_MISSING=true
fi

if ${PYTHON_MISSING} && ! ${CHECK_ONLY} && ! ${BREW_MISSING}; then
    echo ""
    if confirm "Install Python 3 via brew?"; then
        brew_install "Python 3" "python3"
    else
        STATUS_SKIPPED+=("Python 3")
    fi
fi

# ---------------------------------------------------------------------------
# 4. MAVProxy
# ---------------------------------------------------------------------------
log_section "4. MAVProxy (required — upload_params.sh / backup_params.sh)"

MAVPROXY_MISSING=false
if cmd_exists mavproxy.py; then
    MAVPROXY_VER="$(mavproxy.py --version 2>&1 | head -1 || true)"
    log_ok "MAVProxy" "${MAVPROXY_VER}"
    STATUS_OK+=("MAVProxy")
else
    log_missing "MAVProxy"
    STATUS_MISSING+=("MAVProxy")
    MAVPROXY_MISSING=true
fi

if ${MAVPROXY_MISSING} && ! ${CHECK_ONLY}; then
    if ! cmd_exists python3; then
        log_info "Python 3 is required to install MAVProxy — skipping."
        STATUS_SKIPPED+=("MAVProxy")
    else
        echo ""
        if confirm "Install MAVProxy via pip3?"; then
            pip_install "MAVProxy" "MAVProxy"
        else
            STATUS_SKIPPED+=("MAVProxy")
        fi
    fi
fi

# ---------------------------------------------------------------------------
# 5. GitHub CLI (gh)
# ---------------------------------------------------------------------------
log_section "5. GitHub CLI — gh (required — PR workflow)"

GH_MISSING=false
if cmd_exists gh; then
    GH_VER="$(gh --version 2>&1 | head -1)"
    log_ok "gh (GitHub CLI)" "${GH_VER}"
    STATUS_OK+=("gh")
else
    log_missing "gh (GitHub CLI)"
    STATUS_MISSING+=("gh")
    GH_MISSING=true
fi

if ${GH_MISSING} && ! ${CHECK_ONLY} && ! ${BREW_MISSING}; then
    echo ""
    if confirm "Install gh via brew?"; then
        brew_install "gh" "gh"
    else
        STATUS_SKIPPED+=("gh")
    fi
fi

# ---------------------------------------------------------------------------
# 6. tmux
# ---------------------------------------------------------------------------
log_section "6. tmux (required — ao agent sessions)"

TMUX_MISSING=false
if cmd_exists tmux; then
    TMUX_VER="$(tmux -V 2>&1)"
    log_ok "tmux" "${TMUX_VER}"
    STATUS_OK+=("tmux")
else
    log_missing "tmux"
    STATUS_MISSING+=("tmux")
    TMUX_MISSING=true
fi

if ${TMUX_MISSING} && ! ${CHECK_ONLY} && ! ${BREW_MISSING}; then
    echo ""
    if confirm "Install tmux via brew?"; then
        brew_install "tmux" "tmux"
    else
        STATUS_SKIPPED+=("tmux")
    fi
fi

# ---------------------------------------------------------------------------
# 7. Foxglove Studio
# ---------------------------------------------------------------------------
log_section "7. Foxglove Studio (required — visualization)"

FOXGLOVE_MISSING=false
# DESIGN: Foxglove is a .app bundle, not a CLI — check /Applications directly.
if [[ -d "/Applications/Foxglove Studio.app" ]] || [[ -d "${HOME}/Applications/Foxglove Studio.app" ]]; then
    log_ok "Foxglove Studio" "(installed)"
    STATUS_OK+=("Foxglove Studio")
elif cmd_exists foxglove-studio; then
    log_ok "Foxglove Studio" "(CLI found)"
    STATUS_OK+=("Foxglove Studio")
else
    log_missing "Foxglove Studio"
    STATUS_MISSING+=("Foxglove Studio")
    FOXGLOVE_MISSING=true
    log_info "Install via: brew install --cask foxglove-studio"
    log_info "Or download: https://foxglove.dev/download"
fi

if ${FOXGLOVE_MISSING} && ! ${CHECK_ONLY} && ! ${BREW_MISSING}; then
    echo ""
    if confirm "Install Foxglove Studio via brew --cask?"; then
        brew_cask_install "Foxglove Studio" "foxglove-studio"
    else
        STATUS_SKIPPED+=("Foxglove Studio")
    fi
fi

# ---------------------------------------------------------------------------
# 8. Optional tools (shown always, installed only with --all)
# ---------------------------------------------------------------------------
log_section "8. Optional: ESP-IDF native prerequisites"
printf "   ${YELLOW}(install with --all for native firmware dev outside Docker)${RESET}\n\n"

CMAKE_MISSING=false
NINJA_MISSING=false

check_optional "cmake" "cmake" "--version" || CMAKE_MISSING=true
check_optional "ninja" "ninja" "--version" || NINJA_MISSING=true

if ${INSTALL_ALL} && ! ${CHECK_ONLY} && ! ${BREW_MISSING}; then
    if ${CMAKE_MISSING}; then
        if confirm "Install cmake via brew?"; then
            brew_install "cmake" "cmake"
        else
            STATUS_SKIPPED+=("cmake")
        fi
    fi
    if ${NINJA_MISSING}; then
        if confirm "Install ninja via brew?"; then
            brew_install "ninja" "ninja"
        else
            STATUS_SKIPPED+=("ninja")
        fi
    fi
elif (${CMAKE_MISSING} || ${NINJA_MISSING}) && ! ${INSTALL_ALL} && ! ${CHECK_ONLY}; then
    echo ""
    log_info "Run with --all to install optional ESP-IDF native prerequisites."
fi

log_section "9. Optional: colcon (ROS 2 native dev)"
printf "   ${YELLOW}(install with --all for native ROS 2 build outside Docker)${RESET}\n\n"

COLCON_MISSING=false
check_optional "colcon" "colcon" "--help" || COLCON_MISSING=true

if ${INSTALL_ALL} && ! ${CHECK_ONLY}; then
    if ${COLCON_MISSING}; then
        if ! cmd_exists python3; then
            log_info "Python 3 required for colcon — skipping."
            STATUS_SKIPPED+=("colcon")
        else
            if confirm "Install colcon via pip3?"; then
                pip_install "colcon" "colcon-common-extensions"
            else
                STATUS_SKIPPED+=("colcon")
            fi
        fi
    fi
elif ${COLCON_MISSING} && ! ${INSTALL_ALL} && ! ${CHECK_ONLY}; then
    log_info "Run with --all to install colcon."
fi

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
echo ""
printf "${BOLD}================================================${RESET}\n"
printf "${BOLD} Summary${RESET}\n"
printf "${BOLD}================================================${RESET}\n"

if [[ ${#STATUS_OK[@]} -gt 0 ]]; then
    printf "${GREEN}Already installed (${#STATUS_OK[@]}):${RESET} %s\n" "${STATUS_OK[*]}"
fi

if [[ ${#STATUS_INSTALLED[@]} -gt 0 ]]; then
    printf "${GREEN}Installed now    (${#STATUS_INSTALLED[@]}):${RESET} %s\n" "${STATUS_INSTALLED[*]}"
fi

if [[ ${#STATUS_SKIPPED[@]} -gt 0 ]]; then
    printf "${YELLOW}Skipped          (${#STATUS_SKIPPED[@]}):${RESET} %s\n" "${STATUS_SKIPPED[*]}"
fi

if [[ ${#STATUS_OPTIONAL[@]} -gt 0 ]]; then
    printf "${YELLOW}Optional missing (${#STATUS_OPTIONAL[@]}):${RESET} %s\n" "${STATUS_OPTIONAL[*]}"
fi

if [[ ${#STATUS_FAILED[@]} -gt 0 ]]; then
    printf "${RED}Failed           (${#STATUS_FAILED[@]}):${RESET} %s\n" "${STATUS_FAILED[*]}"
fi

if [[ ${#STATUS_MISSING[@]} -gt 0 ]]; then
    # Remove entries we installed or skipped from the still-missing list
    STILL_MISSING=()
    for item in "${STATUS_MISSING[@]}"; do
        found=false
        # DESIGN: Use [@]+ expansion to safely handle empty arrays with set -u.
        for inst in "${STATUS_INSTALLED[@]+"${STATUS_INSTALLED[@]}"}" "${STATUS_SKIPPED[@]+"${STATUS_SKIPPED[@]}"}"; do
            [[ "${item}" == "${inst}" ]] && found=true && break
        done
        ${found} || STILL_MISSING+=("${item}")
    done
    if [[ ${#STILL_MISSING[@]} -gt 0 ]]; then
        printf "${RED}Still missing    (${#STILL_MISSING[@]}):${RESET} %s\n" "${STILL_MISSING[*]}"
        echo ""
        echo "  Some required tools are missing. Run without --check to install."
        exit 1
    fi
fi

echo ""
echo "  ${GREEN}${BOLD}All required tools are present.${RESET}"
echo ""
echo "  Next steps:"
echo "    docker compose -f docker/docker-compose.yml run ground-build"
echo "    docker compose -f docker/docker-compose.yml run firmware-build"
echo ""
