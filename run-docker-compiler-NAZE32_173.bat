@rem -rm = automatically clean up the container and remove the file system when the container exits
@rem -it = starts the container in the interactive mode that allows you to interact with /bin/bash of the container
@rem -v = link local forder to folder inside machine
rem docker run --rm -it -u root -v //d/Github/inav/inav:/src inav-build FLYWOOF411
docker run --rm -it -u root -v //d/Github/inav/inav:/home/src inav-build173 make TARGET=NAZE


