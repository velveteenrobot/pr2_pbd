# TODO
I'm noting things here that I run into but don't want to take the time quite yet to do.

- Replace all instances of 0 and 1 in code with some constant denoting Right / Left arm. Perhaps use a message constant?

- Unify speech and GUI commands. There's a ton of code duplication here.

- Remove the update loop in interaction.py (and the update(...) function in Interaction.py). It can only lead to death (race conditions).

- Remove the interaction.py code alltogether; this is just an unnecessary wrapper around Interaction.py.

- Does `ProgrammedAction.get_requested_targets(...)` ever return something? This drives a large part of the `Interaction.update(...)` loop, so would be nice to remove if not.

- Refactor open / close hand, freeze / relax arm in Interaction.py

- Trajectory business in Interaction.py should be refactored into a new class.

- Change all instances of `base_link` to `/base_link`
