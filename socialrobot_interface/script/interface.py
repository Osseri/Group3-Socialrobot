#!/usr/bin/env python
# [[Python Style Guide]]
# @ RISE LAB
# https://rise-lab.atlassian.net/wiki/spaces/ENG/pages/118358105/RISE+Python+3+Style+Guide
# @ Google
# https://google.github.io/styleguide/pyguide.html
from abc import ABCMeta
from six import with_metaclass


class InterfaceBase(with_metaclass(ABCMeta)):

    """InterfaceBase Definition"""

    def __init__(self):
        """
        Module for Knowledge manager
        """
        pass
