# Todo
# wifi can be setup from bluetooth
- returns lines, or whole documents?
- shared routing of commands with bluetooth and serial via put command
- get command returns web page with set of all put commands
# commands
put/get wifi_config
{ssid, password, port}

put/get wifi_enable
{enable}

put/get twist
{linear{x,y,z},angular{x,y,z}} // all default to zero, z must be zero

get pose
{position{x,y,z}, orientation{x,y,z,w}}

get/set position
{x,y,z}

get ypr
{yaw,pitch,roll}

# Done:
- wifi task keeps wifi asynchronous
- auto connects to wifi after disconnect
