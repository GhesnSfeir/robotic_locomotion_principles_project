#!/bin/bash
USERNAME="robotica"
HOST="192.168.1.25"
REMOTE_PATH="~/GhesnSfeir/Seminar_12"

#scp run main_ghesn_sfeir.py ${USERNAME}@${HOST}:${REMOTE_PATH}
sshpass -p 'upmRobotica' scp run main_ghesn_sfeir.py ${USERNAME}@${HOST}:${REMOTE_PATH}