# TODO
I'm noting things here that I run into but don't want to take the time quite yet to do.

- Remove the update loop in interaction.py (and the update(...) function in Interaction.py). It can only lead to death (race conditions).

- Remove the interaction.py code alltogether; this is just an unnecessary wrapper around Interaction.py.

- Does `ProgrammedAction.get_requested_targets(...)` ever return something? This drives a large part of the `Interaction.update(...)` loop, so would be nice to remove if not.

- Trajectory business in Interaction.py should be refactored into a new class.

- Change all instances of `base_link` to `/base_link`

- Fix the "Selected action step 5" text (should be like (2, RIGHT) or something to match the IK format. Currently just gives weird numbers so that each independent... or else the counting is broken when poses are deleted.)
