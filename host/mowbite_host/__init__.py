"""mowbite-host: installs signed bundles of MowBite OS and tells how the OS is, without ROS."""


class Refused(Exception):
    """A step does not go on; reason is a name of the catalog, the text says why. Nothing has been switched."""

    def __init__(self, reason, text):
        super().__init__(text)
        self.reason = reason
