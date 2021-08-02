#!/usr/bin/env python

# Copyright 1996-2020 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import os
import subprocess
import multiprocessing


def launch_world(worldPath, args=None):
    if args is not None:
        argList = args.split()
    else:
        argList = []
    if sys.platform == 'win32':
        webotsFullPath = os.environ['WEBOTS_HOME'] + os.sep + 'msys64' + \
            os.sep + 'mingw64' + os.sep + 'bin' + os.sep + 'webots.exe'
    else:
        webotsBinary = 'webots'
        if 'WEBOTS_HOME' in os.environ:
            webotsFullPath = os.environ['WEBOTS_HOME'] + os.sep + webotsBinary
        else:
            webotsFullPath = '..' + os.sep + '..' + os.sep + webotsBinary
        if not os.path.isfile(webotsFullPath):
            sys.exit('Error: ' + webotsBinary + ' binary not found')
        webotsFullPath = os.path.normpath(webotsFullPath)
    print(webotsFullPath, worldPath, argList)
    process = subprocess.Popen([webotsFullPath, worldPath] + argList)
    return process


if __name__ == '__main__':
    from controller import Supervisor
    worldPath = '/home/annantang/Desktop/agent_system/webots_rl/khr2/worlds/khr2_rl.wbt'
    args = '--mode=fast --minimize'

    webots_process = launch_world(worldPath, args)
    print('Launching Webots with pid: ', webots_process.pid)
    os.environ['WEBOTS_PID'] = str(webots_process.pid)
    print('init Robot within webots (pid): ', os.environ.get('WEBOTS_PID', 'Not Set'))
    robot1 = Supervisor()
    # sys.exit()
    webots_process2 = launch_world( worldPath)
    print('Launching Webots with pid: ', webots_process2.pid)
    os.environ['WEBOTS_PID'] = str(webots_process2.pid)
    print('init Robot within webots (pid): ', os.environ.get('WEBOTS_PID', 'Not Set'))
    robot2 = Supervisor()
