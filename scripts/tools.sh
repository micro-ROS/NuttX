#!/bin/bash

function has_command() {
    command -v "$1" >/dev/null 2>&1
}

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


function install_dep() {
    set_host_installer
    filter_installed "$@"
    if ! test -z MISSING_PKGS; then
        $INSTALL_COMMAND $INSTALL_OPTIONS $MISSING_PKGS
    fi

}