# 3D Car Physics Simulation Focused on Games
In this study, different 3D and vehicle dynamics concepts are discussed in an attempt to simulate car physics. The fact that the software developed is focused on games means that the software and the research conducted have tried to take into account concepts that can be useful for other domains as well.

For a 3D car, **acceleration**, **braking**, **cornering**, **gear shifting**, **weight transfer** and **suspension system** were considered.

For easier implementation of the research, the **Unity** game engine was preferred. However, in order to make the study more inclusive, except for collision detection and collision response, the other calculations were done from the scratch. Since collision situations are very extensive and not the focus of this study, the physics engine is given **the final linear and angular velocity** of the car and an interactive 3D simulation can be achieved.

In this way, game developers can use the parameters of their favorite car to experience a 3D car with the characteristics of that car.

### A Successor to 2 Old References
This project can arguably be considered as a successor to these studies:
1. Monster, M. 2003. Car Physics for Games: https://www.asawicki.info/Mirror/Car%20Physics%20for%20Games/Car%20Physics%20for%20Games.html
2. Srisuchat, P. 2012. Development of a car physics engine for games: https://nccastaff.bournemouth.ac.uk/jmacey/MastersProject/MSc12/Srisuchat

## Documentation & Demo Project
* Click [Here](/3D_Car_Physics_Simulation_Focused_on_Games.pdf) for the full documentation.
* Demo project files are located at `Assets` folder.

## Notes
If you want to check the demo scene, Make sure you're using the Built-In Render Pipeline; You can activate it this way:
* In your project settings → graphics, remove the default render pipeline asset.
* In your project settings → quality settings, remove the render pipeline asset for each quality level.

### How to Play
* Press `mouse left` button to increase the throttle position.
* Press `mouse right` button to decrease the throttle position.
* Press `mouse middle` button to brake.
* Use `A` and `D` keys to turn left and right.
* Press `Enter` key to restart the scene.
