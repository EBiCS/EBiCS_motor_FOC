# -*- coding: cp1252 -*-
from py2exe.build_exe import py2exe 
from distutils.core import setup

setup(windows=[{"script":"deineDatei.py"}], options={"py2exe":{"includes":["sip", "PyQt4.QtSql"]}})
