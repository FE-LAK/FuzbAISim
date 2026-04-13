# FuzbAI Competition manager - v2

The aim is to update the FuzbAI competition manager to support new simulators and improve the management interface.

There are multiple teams that must play a set of matches against each other. Each team must play against every other team twice - once as red player and once as blue player.

To incrase the speed of the competition, the manager should support multiple simulators in parallel and forward the player requests to the appropriate simulator (and the player side - red/blue).

The manager should feature one REST interface for each registered team. This interface should be used by the team to control the game. It should behave like a proxy to the simulator's player interface.

The manager is responsible for managing the competition and the teams. It should provide a flexible interface for managing competitions and teams. It should also provide a flexible interface for managing pairings. The manager should also provide a flexible interface for managing results. 

## New simulator
The new simulator is built in MuJoCo and exposes three REST interfaces on three different ports:
- red player
- blue player
- management

If the player logic is connected to any of the two game interfaces (red/blue) it behaves as if it is running on the red side - 
data for the blue player is inverted for both input and output. One player logic can therefore play against its own version simply
by running two versions of it, one connected to red, other to blue player interface.

The management interface is used for competition management commands and exposes the following endpoints
* /status
Returns the status of the game - current score, time and ball status together with information on connection state and request rate of both red and blue players

* /reset
Resets the game to the initial state, resets the timer to 0 and score to 0:0. The ball is kept out of the field.

* /start
Starts the game by starting the timer, triggering the ball to be inserted into initial position.

## New features

The existing manager is preconfigured for selected number of teams. Its logic for pairing teams is fixed and not very flexible. It also does not support the new simulator. The new manager should support the new simulator and provide a more flexible interface for managing competitions.

The new manager should support the following features:
* Support for the new simulator
* Flexible team management
* Flexible pairing management
* Flexible competition management - allow the game to be played again, allow the specific team to be temporarily disabled 
* Web interface for competition management
* Web interface for team management
* Web interface for pairing management

The web interfaces should be more dynamic and provide more information about the competition and the teams. They should also be more interactive and allow the user to control the competition and the teams.

## Results storing, display and analysis

Results should be stored in a database and should be displayed in a web interface. The results should be displayed in a way that allows the user to see the results of the competition and the teams. Check the existing competition manager for ideas on how to display the results.

