#!/bin/bash

# build_package.sh
#
# Questo script compila il pacchetto "cooperative_localization"
# nel workspace ROS2 corrispondente, rilevato automaticamente.
#
# authors: Lorenzo Nobili, Leonardo Bacciocchi, Cosmin Alessandro Minut
# date: 2025

# Nome del pacchetto
PACKAGE_NAME="cooperative_localization"

# Se viene passato un argomento, viene usato come nome del workspace
if [[ -n "$1" ]]; then
    ROS_WORKSPACE="$1"
    WORKSPACE_PATH=$(find "$HOME" -type d -name "$ROS_WORKSPACE" -print -quit)
else
    # Se non viene passato, risale le directory fino a trovarne una con src/ al suo interno
    WORKSPACE_PATH="$PWD"
    while [[ "$WORKSPACE_PATH" != "/" && ! -d "$WORKSPACE_PATH/src" ]]; do
        WORKSPACE_PATH=$(dirname "$WORKSPACE_PATH")
    done
fi

# Check esistenza e validità del workspace
if [[ -z "$WORKSPACE_PATH" || ! -d "$WORKSPACE_PATH/src" ]]; then
    echo "Impossibile trovare un workspace ROS2 valido (manca src/)"
    echo "Usage: ./build_package.sh <ros2_workspace_name>  # oppure eseguilo da dentro il pacchetto"
    exit 1
fi

# Cerca di spostarsi all'interno del workspace, se non ci riesce termina con exit code 2
cd "$WORKSPACE_PATH" || exit 2

# Esecuzione build
colcon build --packages-select "$PACKAGE_NAME"

# Check risultato build
if [[ $? -eq 0 ]]; then
    echo "Build completata con successo!"
    echo "Esegui ora: source install/setup.bash"
    exit 0
else
    echo "Errore durante la fase di building del progetto."
    exit 3
fi
