### What was the goal of your project?
* We were trying to prototype the [NASA sample challenge](http://www.nasa.gov/sites/default/files/files/SRR14-fact-Sheet.pdf), in which robots need to wander an area collecting samples that might simulate sample-taking on a planet. Simply put, our robot was supposed to leave a base station, run over samples that were tagged with fiducials, and return to the base station in a certain time limit.

### How did you solve the problem? (Note: this doesn't have to be super-detailed, you should try to explain what you did at a high level so that others in the class could reasonably understand what you did).
* We solved the problem by breaking it into finite states of Seek, Grab, and Return. The robot would explore the world in Seek, go into Grab mode to aqcuire samples once samples were seen, and would go into Return mode once a time limit was reached. Seek was a pattern search with occasional spins to get more camera coverage of the area, grab was a set of maneuvers that would try to line up the robot on the fiducial so it could be driven straight over, and return was just a waypoint placed at the origin that the robot would return to while obstacle avoiding.

### Describe a design decision you had to make when working on your project and what you ultimately did (and why)?  These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.
* One design decision we made was to really decentralize our data collection into a bunch of separate nodes that processed their respective input and spat out processed output that the robot just listened to. This is described more in the next section, and a list of the separate nodes can be seen in challenge_bot.launch. Decentralizing our data processing made testing simpler.

### How did you structure your code?
* We generally set up a string of separate nodes that pumped out current data about the world. For example,  
    * a node that processed /odom along our /reset_pos command to output /current_pos, 
    * a node that published a map and added new values to the from service calls, 
    * a node that independently watched the video stream for fiducials and added the fiducials to the map when seen, 
    * a node that independently watched /scan and published obstacle avoid vectors, etc. 
* This really simplified what the robot was doing, which was simply listening to all of this constantly updated data and perform operations based on that information. The robot code is in challenge_bot.py, and you can see all of our independent nodes in challenge_bot.launch.

### What if any challenges did you face along the way?
    * We had some trouble getting good measurements from the camera to the fiducial, related to lighting and orientation of the fiducial. These were generally overcomeable with additional lighting, and could be even better if we processed the image to really amp up the contrast.
    * Our obstacle avoid method of layering an obstacle avoid vector on top of a drive vector doesn't produce great results, especially with small objects that will still impede the path of the robot.
    * We had trouble for a while differentiating between getting new tf measurements and just taking the last tf measurement, which could be pretty old.
    * We thought we would have enough time to iterate on a couple of the simple sections, but that didn't really happen.

### What would you do to improve your project if you had more time?
    * A number of places in our code implement very simplistic logic. For example, the Grab case executes a series of moves that try to line up the robot with the fiducial and drive over it. More elegant would be a path planning function that takes the fiducial position and orientation into account from the beginning and just drives a course.
    * We could try to return to the base along a previously travelled path, instead of going straight to base like we are now. This would be a good goal to shoot for because in the actual competition teams want to make absolutely sure the robot will get back and won't run into any unexpected obstacles on the way.
    * Another area would be to construct a Kalman filter for the robot position, instead of doing what we do, which is using odom and overriding it periodically with user information. There are many areas like this. Our future work would be to choose one or two of these options that interest us and flesh them out.

### Did you learn any interesting lessons for future robotic programming projects?  These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.
    * Pay attention to whether information is local or global
    * Build tools to debug your process earlier rather than later (things that plug into RVIZ, generally)
    * Be clear about what states you are in and which states you will go to from there
    * Construct working printouts cleanly and in a similar style so they will be useful
    * Got more comfortable with ROS as a framework
    * Got more comfortable coding with a bunch of states (a lot of things happening in parallel)
    * Robots are non-deterministic (b/c of the environment), and you have to think more of robustness than a lot of code
    * We used a couple external libraries for fiducial tracking, could have done more of that for path planning
    * Learned a bit more about what a robot unit test looks like
