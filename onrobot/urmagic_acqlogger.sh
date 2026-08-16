#!/bin/sh
# UR "magic file": PolyScope runs this as root the moment the USB key with
# this file at its root is inserted. Job: launch acq_logger_daemon.py from
# the same key and return immediately, so the insertion handler never blocks.
# Reviewed by a human before first insertion (project rule): keep it short,
# no network call, no download, no write outside the key.

# Resolve this script's own directory from $0 rather than hardcoding a mount
# point: the mount path (e.g. /media/usb, /programs/usb) differs across CB3
# images and PolyScope versions.
DIR=$(cd "$(dirname "$0")" && pwd)

# Daemon source and log file both live on the key, next to this script.
DAEMON="$DIR/acq_logger_daemon.py"
LOG="$DIR/acq_logger_daemon.log"

# Find any already-running instance of THIS daemon, matched narrowly on its
# own filename. Never grep/kill on a bare "python": the controller runs its
# own Python processes for the robot arm and killing those halts the robot.
# The "[a]cq_logger_daemon.py" bracket trick keeps this grep call itself out
# of its own match, so the script cannot accidentally kill itself either.
OLDPID=$(ps -ef | grep "[a]cq_logger_daemon.py" | awk '{print $2}')

# Stop the previous instance, if one is running, before starting a new one.
if [ -n "$OLDPID" ]; then
    kill $OLDPID 2>/dev/null
fi

# Pick an interpreter: CB3 ships Python 2.7 as "python2"; fall back to
# "python" for images where it is aliased instead.
if command -v python2 >/dev/null 2>&1; then
    PYTHON=python2
elif command -v python >/dev/null 2>&1; then
    PYTHON=python
else
    # Neither interpreter is on PATH: record why in the log and fail loudly
    # (non-zero exit) instead of silently doing nothing on insertion.
    echo "$(date) [ACQ] no python2 or python interpreter found on PATH" >> "$LOG"
    exit 1
fi

# Launch the daemon detached from this shell (nohup + background "&"), with
# its stdout and stderr appended to the log file on the key.
nohup "$PYTHON" "$DAEMON" >> "$LOG" 2>&1 &

# Return immediately: the insertion handler must not wait on a long-running
# daemon, and the daemon keeps running in the background after this exits.
exit 0
