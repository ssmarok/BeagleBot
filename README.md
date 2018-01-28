# BeagleBot - Autonomous Nerf Ball Shooter Bot
Roborodentia 2018
Competition Code developed for the Cal Poly Roborodentia 2018 robotics competition.

Authors: Sean Wallace, Sukhman Marok, and Hayden Hudgins.


#Internal Development Notes 

Use this. 

###Remote Development 

Run the latest build
```
alias run="sudo /home/robot/Beaglebot/./main"
```

Rebuild and run
```
alias build="make clean && make && sudo /home/robot/Beaglebot/./main"
```

Wipe all remote directory contents (following this, you should push local workspace content onto the default BeagleBone remote working directory as described under "Local Development")
```
alias wipe="cd /home/robot && rm -rf /home/robot/Beaglebot && mkdir /home/robot/BeagleBot && cd /home/robot/BeagleBot && ls -la"
```

Quality of life default working remote directory
```
cd /home/robot/Beaglebot && clear && ls
```

### Local Development
Using ssh, remoteLogin into the BeagleBone Blue
```
alias rl="ssh robot@192.168.6.2"

```

Push local workspace content onto the default BeagleBone remote working directory
```
alias pushLocal="rsync -avzh ~/workspace/BeagleBot/ robot@192.168.6.2:/home/robot/Beaglebot/"
```

Pull remote workspace content onto your local working directory
```
alias pullRemote="rsync -avzh robot@192.168.6.2:/home/robot/Beaglebot/ ~/workspace/BeagleBot/"

```
