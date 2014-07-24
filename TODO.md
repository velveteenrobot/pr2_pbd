# TODO
- Fix robot speech (it's very well-known how; in other branches...) and add roslaunch param as to whether to use sounds or speech (right now it's a code option).

- Add notification to `ActionStepMarker._is_reachable(...)` to log IK vs FK + seed for reachability. Probably have to expose this somewhere else so it can be accessed.

- Tests:
	- Fix tests on real robot.
	- Add (simulated) interactive marker clicks to tests
		- moving arms to a particular step (is this really possible? There's a lot of code seemingly to support it.).
		- deleting an object
		- moving a pose to a valid position
		- moving a pose to an invalid position; check execution doesn't work
		- changing relativeness (robot vs obj both ways and obj to different obj)
	- Check whether freezing / relaxing arms really work.
	- Add additional object to Gazebo for tests.
	- Be relative to one object, automatically move (via gazebo), execute (to test relative poses move or at least work when moved)

- See if there's any way to access the PR2 gripper LED for object relativeness indication (while moving arms).

- Consider changing all instances of `base_link` to `/base_link`. Is this technically more correct?

- Trajectory business in Interaction.py maybe should be refactored into a new class. So should most differences between arm targets and trajectories; they should be subclasses of a "step" class. However, this is already happening in the newer versions of this system, so it probably isn't worth doing here.

- Fix the "Selected action step 5" text (should be like (2, RIGHT) or something to match the IK format. Currently just gives weird numbers so that each independent... or else the counting is broken when poses are deleted.)

- Documentation
	- Read about how to use [autodoc](http://sphinx-doc.org/ext/autodoc.html)
	- Make rst stubs probably for the code
	- Make commands / script to generate
	- Ignore genreated doc files in .gitignore (keep stubs probably?)
	- Add how to generate docs to README
	- Auto docs / hosting / linking?
