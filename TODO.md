# TODO
- Fix robot speech (it's very well-known how; in other branches...) and add roslaunch param as to whether to use sounds or speech (right now it's a code option).

- Tests:
	- Fix tests on real robot.
	- Add (simulated) interactive marker clicks to tests, especially moving arms to a particular step (is this really possible? There's a lot of code seemingly to support it.).
	- Check whether freezing / relaxing arms really work.

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
