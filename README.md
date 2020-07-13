# community-projects

[![Chat](https://img.shields.io/discord/565154702715518986)](https://discordapp.com/invite/nTWbN9m)

This repo contains Webots projects (PROTO files, controllers, simulation worlds, etc.) made by the community.

Any contribution is welcome!

## Structure of the Repositiory

This repository should respect such hierarchy:
```
appearances
  -> docs
  -> protos
    -> icons
    -> textures
devices
  -> <brand>
    -> docs
    -> protos
      -> icons
      -> textures
objects 
  -> <category>
    -> controllers
      -> <controller name>
    -> docs
    -> protos 
      -> icons
      -> textures
robots 
  -> <brand>
    -> controllers
      -> <controller name>
    -> docs
    -> libraries
      -> <library name>
    -> plugins
      -> remote_controls
        -> <remote control name>
      -> robot_windows
        -> <robot windows name>
    -> protos 
      -> icons
      -> textures
    -> worlds
      -> textures
samples
  -> <category>
    -> controllers
      -> <controller name>
    -> docs
    -> worlds
      -> textures
vehicles
  -> controllers
    -> <controller name>
  -> docs
  -> libraries
    -> <library name>
  -> plugins
    -> remote_controls
      -> <remote control name>
    -> robot_windows
      -> <robot windows name>
  -> protos 
    -> icons
    -> textures
  -> worlds
    -> textures
```

## How to Use it

The simplest solution to use all the content of this repository is to use the Webots 'Extra projects path' from the [Webots Preferences](https://cyberbotics.com/doc/guide/preferences#general).
Some executables may need to be compiled.

## Disclaimer

This repository is not maintained by Cyberbotics, some projects may not work or not be up to date to be used with the latest version of Webots.
