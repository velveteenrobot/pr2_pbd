#!/usr/bin/env python

'''This runs the PbD system (i.e. the backend).'''

# Core ROS imports come first.
import roslib
roslib.load_manifest('pr2_pbd_interaction')
import rospy

if __name__ == '__main__':
    # Check whether we want code coverage, and start if so.
    use_coverage = rospy.get_param(
        '/pr2_pbd_interaction/coverage', default=False)
    if use_coverage:
        from coverage import coverage
        cov = coverage(
            include="*/pr2_pbd_interaction/src/*.py",  # source files
            omit="*/src/pr2_pbd_interaction/*"  # generated files
        )
        cov.start()

    # Run the system
    import interaction
    interaction_ = interaction.Interaction()
    rospy.spin()

    # System execution finished; generate coverage report if enabled.
    if use_coverage:
        cov.stop()
        cov.html_report(title='PR2 PbD')
