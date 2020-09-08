
# Arduino software for soil moisture and temperature sensors

This is mostly a grab-bag of Arduino IDE sketches and some data
regarding to different LoRa client/server configurations.

The directory 'old' has lots of Arduino IDE .ino programs written
to test low power operations and different RTCs (DS3231, DS1370).

The rs-simple-lora-client is the code for the RocketScream board
(ARM Cortex M0+, effectively the same as an Arduino Zero).

Some libaries are clones of github repos. Making minor edits will
cause git to refer to them as 'dirty' and make annoying note in 
git status. Use 'git config --global diff.ignoreSubmodules dirty'
to turn that off.
