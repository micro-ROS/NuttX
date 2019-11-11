#!/bin/bash


# Returns if a command is available in the current machine
function has_command() {
    command -v "$1" >/dev/null 2>&1
}

# Sets multiple variables with system specifics.
# INSTALL_COMMAND: is the package manager of the linux distribution
# INSTALL_OPTIONS: are the options to install packages without any input from the user required
# INSTALL_QUERY: command and options to use for checking the existence of packages.
function set_host_installer() {
    if [ -f /etc/arch-release ]; then
        if has_command pacman; then
            INSTALL_COMMAND="pacman"
            INSTALL_OPTIONS="-S"
            INSTALL_QUERY="pacman -Qi"
        fi
    elif [ -f /etc/debian_version ]; then
        if has_command apt-get; then
            INSTALL_COMMAND="apt-get"
            INSTALL_OPTIONS="install -y"
            INSTALL_QUERY="dpkg -s"
        fi
    fi
}

# Takes a list of pakages and sets MISSING_PKGS with those missing in the current system.
function filter_installed() {
    for package in "$@"
    do
        if $INSTALL_QUERY $package; then
            echo $package "Already installed"
        else
            MISSING_PKGS+=" "$package
        fi
    done
}

# Install the list of packages in the current system.
function install_dep() {
    set_host_installer
    filter_installed "$@"
    if ! test -z MISSING_PKGS; then
        $INSTALL_COMMAND $INSTALL_OPTIONS $MISSING_PKGS
    fi

}